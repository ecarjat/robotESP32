#include "RpPassthrough.h"

#if RP_ENABLE_MDNS
#include <ESPmDNS.h>
#endif

static String htmlEscape(const String& input) {
  String out;
  out.reserve(input.length());
  for (size_t i = 0; i < input.length(); ++i) {
    const char c = input[i];
    switch (c) {
      case '&':
        out += "&amp;";
        break;
      case '<':
        out += "&lt;";
        break;
      case '>':
        out += "&gt;";
        break;
      case '"':
        out += "&quot;";
        break;
      case '\'':
        out += "&#39;";
        break;
      default:
        out += c;
        break;
    }
  }
  return out;
}

RpPassthrough::RpPassthrough(HardwareSerial& link)
    : link_(link), server_(RP_PASSTHROUGH_PORT), configServer_(RP_CONFIG_PORT) {
  snprintf(ipStr_, sizeof(ipStr_), "0.0.0.0");
  apSsid_[0] = '\0';
}

void RpPassthrough::begin() {
  wifiReady_ = false;
  apMode_ = false;
  serverStarted_ = false;
  snprintf(ipStr_, sizeof(ipStr_), "0.0.0.0");
  apSsid_[0] = '\0';
}

bool RpPassthrough::connectSta() {
  prefs_.begin(RP_WIFI_NAMESPACE, true);
  String ssid = prefs_.getString(RP_WIFI_SSID_KEY, "");
  String pass = prefs_.getString(RP_WIFI_PASS_KEY, "");
  prefs_.end();

  if (ssid.length() == 0) {
    return false;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), pass.c_str());

  const uint32_t start = millis();
  while (millis() - start < RP_WIFI_STA_TIMEOUT_MS) {
    if (WiFi.status() == WL_CONNECTED) {
      return true;
    }
    delay(100);
  }

  WiFi.disconnect(true, true);
  return false;
}

bool RpPassthrough::startAp() {
  uint64_t mac = ESP.getEfuseMac();
  uint16_t suffix = static_cast<uint16_t>(mac & 0xFFFFU);
  snprintf(apSsid_, sizeof(apSsid_), "%s-%04X", RP_AP_SSID_PREFIX, suffix);

  WiFi.mode(WIFI_AP_STA);
  bool ok = WiFi.softAP(apSsid_, RP_AP_PASSWORD);
  if (!ok) {
    Serial.println("[RP] SoftAP start failed");
  }
  return ok;
}

void RpPassthrough::setActive(bool enable) {
  if (enable == active_) {
    return;
  }

  active_ = enable;
  if (active_) {
    if (!wifiReady_) {
      wifiReady_ = startWifi();
    }
    if (!wifiReady_) {
      Serial.println("[RP] WiFi start failed");
      active_ = false;
      return;
    }
    startServer();
    flushUart();
    flushClient();
    lastActivityMs_ = millis();
  } else {
    stopServer();
    stopWifi();
  }
}

void RpPassthrough::tick() {
  if (!active_ || !wifiReady_) {
    return;
  }

  handleConfigClient();

  if (pendingStaConnect_ && !client_.connected()) {
    pendingStaConnect_ = false;
    if (attemptStaConnect(pendingStaSsid_, pendingStaPass_)) {
      lastConfigMessage_ = "Connected to " + pendingStaSsid_;
    } else {
      lastConfigMessage_ = "Connection failed. Check credentials.";
    }
    updateIpString();
  }

  acceptClient();
  if (client_.connected()) {
    bridge();
    if (RP_PASSTHROUGH_IDLE_TIMEOUT_MS > 0U &&
        (millis() - lastActivityMs_ >= RP_PASSTHROUGH_IDLE_TIMEOUT_MS)) {
      stopClient("Client idle timeout");
    }
  }
}

void RpPassthrough::startServer() {
  if (serverStarted_) {
    return;
  }
  server_.begin();
  serverStarted_ = true;
  Serial.printf("[RP] Server listening on %u\n", static_cast<unsigned>(RP_PASSTHROUGH_PORT));
}

void RpPassthrough::stopServer() {
  stopClient("Server stopping");
  if (serverStarted_) {
    server_.stop();
    serverStarted_ = false;
  }
}

bool RpPassthrough::startWifi() {
  wifiReady_ = false;
  apMode_ = false;
  if (connectSta()) {
    wifiReady_ = true;
    apMode_ = false;
    Serial.printf("[RP] WiFi STA connected: %s\n", WiFi.localIP().toString().c_str());
  } else {
    if (startAp()) {
      wifiReady_ = true;
      apMode_ = true;
      Serial.printf("[RP] WiFi AP started: %s (%s)\n", apSsid_, WiFi.softAPIP().toString().c_str());
    }
  }

  updateIpString();

#if RP_ENABLE_MDNS
  if (wifiReady_) {
    if (MDNS.begin("robot-esp32")) {
      MDNS.addService("robotproto", "tcp", RP_PASSTHROUGH_PORT);
    } else {
      Serial.println("[RP] mDNS init failed");
    }
  }
#endif

  if (apMode_) {
    startConfigServer();
  }

  return wifiReady_;
}

void RpPassthrough::stopWifi() {
  if (!wifiReady_) {
    return;
  }
#if RP_ENABLE_MDNS
  MDNS.end();
#endif
  stopConfigServer();
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
  wifiReady_ = false;
  apMode_ = false;
  snprintf(ipStr_, sizeof(ipStr_), "0.0.0.0");
}

void RpPassthrough::stopClient(const char* reason) {
  if (!client_.connected()) {
    return;
  }
  if (reason && reason[0] != '\0') {
    Serial.printf("[RP] %s\n", reason);
  }
  client_.stop();
  if (rxDropCount_ || txDropCount_) {
    Serial.printf("[RP] Bridge drops rx=%lu tx=%lu\n", static_cast<unsigned long>(rxDropCount_),
                  static_cast<unsigned long>(txDropCount_));
  }
  rxDropCount_ = 0;
  txDropCount_ = 0;
}

void RpPassthrough::acceptClient() {
  WiFiClient newClient = server_.available();
  if (!newClient) {
    return;
  }

  if (client_.connected()) {
    stopClient("Client replaced");
  }

  client_ = newClient;
  client_.setNoDelay(true);
  flushClient();
  lastActivityMs_ = millis();
  Serial.printf("[RP] Client connected: %s\n", client_.remoteIP().toString().c_str());
}

void RpPassthrough::startConfigServer() {
  if (configServerStarted_) {
    return;
  }
  configServer_.on("/", HTTP_GET, [this]() { handleConfigRoot(); });
  configServer_.on("/save", HTTP_POST, [this]() { handleConfigSave(); });
  configServer_.onNotFound([this]() { configServer_.send(404, "text/plain", "Not Found"); });
  configServer_.begin();
  configServerStarted_ = true;
  Serial.printf("[RP] Config portal on http://%s:%u/\n", WiFi.softAPIP().toString().c_str(),
                static_cast<unsigned>(RP_CONFIG_PORT));
}

void RpPassthrough::stopConfigServer() {
  if (!configServerStarted_) {
    return;
  }
  configServer_.stop();
  configServerStarted_ = false;
}

void RpPassthrough::handleConfigClient() {
  if (configServerStarted_) {
    configServer_.handleClient();
  }
}

void RpPassthrough::handleConfigRoot() {
  prefs_.begin(RP_WIFI_NAMESPACE, true);
  String savedSsid = prefs_.getString(RP_WIFI_SSID_KEY, "");
  prefs_.end();

  String html;
  html.reserve(2048);
  html += "<!doctype html><html><head><meta charset=\"utf-8\">";
  html += "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">";
  html += "<title>Robot WiFi</title></head><body>";
  html += "<h1>Robot WiFi Setup</h1>";
  html += "<p>AP SSID: ";
  html += htmlEscape(apSsid_);
  html += "</p><p>AP IP: ";
  html += WiFi.softAPIP().toString();
  html += "</p>";
  if (lastConfigMessage_.length() > 0) {
    html += "<p><b>Status:</b> ";
    html += htmlEscape(lastConfigMessage_);
    html += "</p>";
  }
  html += "<form method=\"post\" action=\"/save\">";
  html += "<label>SSID<br><input name=\"ssid\" value=\"";
  html += htmlEscape(savedSsid);
  html += "\"></label><br>";
  html += "<label>Password<br><input name=\"pass\" type=\"password\"></label><br>";
  html += "<button type=\"submit\">Save</button></form>";

  int count = WiFi.scanNetworks(false, false);
  html += "<h2>Networks</h2>";
  if (count <= 0) {
    html += "<p>No networks found.</p>";
  } else {
    html += "<ul>";
    for (int i = 0; i < count; ++i) {
      String ssid = WiFi.SSID(i);
      int32_t rssi = WiFi.RSSI(i);
      bool open = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN);
      html += "<li>";
      html += htmlEscape(ssid);
      html += " (";
      html += rssi;
      html += " dBm, ";
      html += open ? "open" : "secured";
      html += ")</li>";
    }
    html += "</ul>";
    WiFi.scanDelete();
  }

  html += "</body></html>";
  configServer_.send(200, "text/html", html);
}

void RpPassthrough::handleConfigSave() {
  String ssid = configServer_.arg("ssid");
  String pass = configServer_.arg("pass");
  ssid.trim();

  if (ssid.length() == 0) {
    lastConfigMessage_ = "SSID is required.";
    handleConfigRoot();
    return;
  }

  prefs_.begin(RP_WIFI_NAMESPACE, false);
  prefs_.putString(RP_WIFI_SSID_KEY, ssid);
  prefs_.putString(RP_WIFI_PASS_KEY, pass);
  prefs_.end();

  pendingStaSsid_ = ssid;
  pendingStaPass_ = pass;
  pendingStaConnect_ = true;
  lastConfigMessage_ = "Saved. Attempting to connect...";
  handleConfigRoot();
}

bool RpPassthrough::attemptStaConnect(const String& ssid, const String& pass) {
  if (ssid.length() == 0) {
    return false;
  }

  WiFi.mode(WIFI_AP_STA);
  WiFi.disconnect(false, true);
  Serial.printf("Coonecting to: %s, pass: %s", ssid.c_str(), pass.c_str());
  WiFi.begin(ssid.c_str(), pass.c_str());

  const uint32_t start = millis();
  while (millis() - start < RP_WIFI_STA_TIMEOUT_MS) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("[RP] WiFi STA connected: %s\n", WiFi.localIP().toString().c_str());
      WiFi.softAPdisconnect(true);
      WiFi.mode(WIFI_STA);
      apMode_ = false;
      stopConfigServer();
      return true;
    }
    delay(100);
  }

  WiFi.disconnect(false, true);
  apMode_ = true;
  Serial.println("[RP] WiFi STA connect failed");
  return false;
}

void RpPassthrough::bridge() {
  bool activity = false;

  const size_t rxCount = readAvailable(client_, rxBuf_, sizeof(rxBuf_));
  if (rxCount > 0U) {
    size_t written = link_.write(rxBuf_, rxCount);
    if (written < rxCount) {
      rxDropCount_ += (rxCount - written);
    }
    activity = true;
  }

  const size_t txCount = readAvailable(link_, txBuf_, sizeof(txBuf_));
  if (txCount > 0U) {
    size_t written = client_.write(txBuf_, txCount);
    if (written < txCount) {
      txDropCount_ += (txCount - written);
    }
    activity = true;
  }

  if (activity) {
    lastActivityMs_ = millis();
  }
}

void RpPassthrough::flushUart() {
  while (link_.available()) {
    link_.read();
  }
}

void RpPassthrough::flushClient() {
  if (!client_.connected()) {
    return;
  }
  while (client_.available()) {
    client_.read();
  }
}

void RpPassthrough::updateIpString() {
  if (!wifiReady_) {
    snprintf(ipStr_, sizeof(ipStr_), "0.0.0.0");
    return;
  }
  IPAddress ip =
      (WiFi.status() == WL_CONNECTED) ? WiFi.localIP() : (apMode_ ? WiFi.softAPIP() : IPAddress());
  if (ip == IPAddress()) {
    snprintf(ipStr_, sizeof(ipStr_), "0.0.0.0");
    return;
  }
  snprintf(ipStr_, sizeof(ipStr_), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
}

size_t RpPassthrough::readAvailable(Stream& stream, uint8_t* buf, size_t max_len) {
  size_t count = 0;
  while (count < max_len && stream.available()) {
    int c = stream.read();
    if (c < 0) {
      break;
    }
    buf[count++] = static_cast<uint8_t>(c);
  }
  return count;
}
