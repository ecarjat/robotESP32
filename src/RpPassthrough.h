#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>

#include "rp_passthrough_config.h"

class RpPassthrough {
 public:
  explicit RpPassthrough(HardwareSerial& link);

  void begin();
  void tick();
  void setActive(bool enable);

  bool isActive() const { return active_; }
  bool hasClient() { return client_.connected(); }
  bool isApMode() const { return apMode_; }
  bool wifiReady() const { return wifiReady_; }
  const char* ipAddress() const { return ipStr_; }
  const char* apSsid() const { return apSsid_; }

 private:
  bool connectSta();
  bool startAp();
  void startServer();
  void stopServer();
  void stopClient(const char* reason);
  bool startWifi();
  void stopWifi();
  void startConfigServer();
  void stopConfigServer();
  void handleConfigClient();
  void handleConfigRoot();
  void handleConfigSave();
  bool attemptStaConnect(const String& ssid, const String& pass);
  void acceptClient();
  void bridge();
  void flushUart();
  void flushClient();
  void updateIpString();
  size_t readAvailable(Stream& stream, uint8_t* buf, size_t max_len);

  HardwareSerial& link_;
  WiFiServer server_;
  WiFiClient client_;
  WebServer configServer_;
  Preferences prefs_;
  bool active_ = false;
  bool wifiReady_ = false;
  bool apMode_ = false;
  bool serverStarted_ = false;
  bool configServerStarted_ = false;
  bool pendingStaConnect_ = false;
  String pendingStaSsid_;
  String pendingStaPass_;
  String lastConfigMessage_;
  uint32_t lastActivityMs_ = 0;
  uint32_t rxDropCount_ = 0;
  uint32_t txDropCount_ = 0;
  char ipStr_[16];
  char apSsid_[32];
  uint8_t rxBuf_[RP_BRIDGE_RX_BUF_SIZE];
  uint8_t txBuf_[RP_BRIDGE_TX_BUF_SIZE];
};
