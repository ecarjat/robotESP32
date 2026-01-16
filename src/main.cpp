#include <Arduino.h>
#include <BLEGamepadClient.h>
#include <U8x8lib.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <menu.h>
#include <menuIO/U8x8Out.h>
#include <menuIO/chainStream.h>
#include <menuIO/keyIn.h>
#include <menuIO/serialIn.h>
#include <menuIO/serialOut.h>

#include <cstring>

#include "GamepadMenuIn.h"
#include "RpPassthrough.h"
#include "Stm32Link.h"
#include "config_pins.h"
#include "rp_passthrough_config.h"
#include "ui/menu_ui.h"

using namespace Menu;

GamepadMenuIn gpIn;
XboxController controller;
U8X8_SSD1306_128X64_NONAME_HW_I2C display(U8X8_PIN_NONE);
static constexpr uint8_t I2C_ADDR_8BIT = 0x78;
bool g_connected = false;
#define U8_Width 128
#define U8_Height 64
HardwareSerial linkSerial(2);
Stm32Link stmLink(linkSerial);
RpPassthrough rpPassthrough(linkSerial);

static uint32_t gBootMs = 0;
static bool gControllerEverConnected = false;
static bool gForceDashboardRedraw = true;
static bool gForcePassthroughRedraw = true;
static TaskHandle_t gStmLinkTaskHandle = nullptr;
static constexpr uint32_t kYToggleDebounceMs = 250U;

#ifndef RP_STMLINK_TASK_STACK_WORDS
#define RP_STMLINK_TASK_STACK_WORDS 4096U
#endif
#ifndef RP_STMLINK_TASK_PRIORITY
#define RP_STMLINK_TASK_PRIORITY 1
#endif
#ifndef RP_STMLINK_TASK_DELAY_MS
#define RP_STMLINK_TASK_DELAY_MS 1U
#endif

// ---- Outputs: serial + OLED ----
#define MAX_DEPTH 3

// MENU_OUTPUTS(out, MAX_DEPTH, U8X8_OUT(display, {0,0,16,8}, )   // 128x64 => 16 cols x 8 rows (8x8
// font cells)
// );

menuIn* inputs[] = {&gpIn};
chainStream<1> in(inputs);

// NAVROOT(nav, MenuUI::mainMenu, MAX_DEPTH, in, out);

const panel default_serial_panels[] MEMMODE = {{0, 0, 40, 10}};
navNode* default_serial_nodes[sizeof(default_serial_panels) / sizeof(panel)];
panelsList default_serial_panel_list(default_serial_panels, default_serial_nodes,
                                     sizeof(default_serial_panels) / sizeof(panel));

// define output device
idx_t serialTops[MAX_DEPTH] = {0};
serialOut outSerial(*(Print*)&Serial, serialTops);

// define outputs controller
idx_t u8x8_tops[MAX_DEPTH];
PANELS(u8x8Panels, {0, 0, U8_Width / 8, U8_Height / 8});
U8x8Out u8x8Out(display, u8x8_tops, u8x8Panels);

menuOut* constMEM outputs[] MEMMODE = {&u8x8Out};  // list of output devices
outputsList out(outputs, 1);                       // outputs list controller

// //define navigation root and aux objects
// navNode nav_cursors[MAX_DEPTH];//aux objects to control each level of navigation
// navRoot nav(MenuUI::mainMenu, nav_cursors, MAX_DEPTH, Serial, out);
NAVROOT(nav, MenuUI::mainMenu, MAX_DEPTH, in, out);

static bool gMenuActive = false;
static bool gPrevMenuButton = false;
static bool gPrevYButton = false;
static bool gTeleopEstopActive = true;
static uint32_t gLastYToggleMs = 0;
static XboxControlsState gPrevState;
static uint32_t gLoopReportMs = 0;
static constexpr float kTeleopDeadzone = 0.12f;
static bool gPrevXButton = false;
static bool gDumpRequested = false;
static uint32_t gLastXPressMs = 0;

static float applyDeadzone(float value, float deadzone) {
  if (value > -deadzone && value < deadzone) {
    return 0.0f;
  }
  return value;
}

static void splash() {
  display.clear();
  display.drawString(0, 0, "Robot UI");
  display.drawString(0, 2, "Initializing...");
  delay(800);
}

static void stmLinkTask(void* arg) {
  (void)arg;
  Serial.println("[LINK] Stm32Link task started");
  for (;;) {
    stmLink.tick();
    if (RP_STMLINK_TASK_DELAY_MS > 0U) {
      vTaskDelay(pdMS_TO_TICKS(RP_STMLINK_TASK_DELAY_MS));
    } else {
      taskYIELD();
    }
  }
}

static void setMenuActive(bool on) {
  if (on == gMenuActive) return;
  gMenuActive = on;
  if (gMenuActive) {
    nav.idleOff();
  } else {
    nav.idleOn();
  }
  display.clear();
  gForceDashboardRedraw = true;
  gForcePassthroughRedraw = true;
}

static void renderDashboard(const Stm32Link& link, const XboxController& ctrl, bool forceRedraw) {
  static bool labelsDrawn = false;
  static String lastVals[5];
  if (forceRedraw) {
    labelsDrawn = false;
    for (auto& v : lastVals) {
      v = "";
    }
  }
  if (!labelsDrawn) {
    display.clear();
    display.drawString(0, 0, "Main");
    display.drawString(0, 2, "IMU:");
    display.drawString(0, 3, "Gamepad:");
    display.drawString(0, 4, "Robot:");
    display.drawString(0, 5, "Bat V:");
    // display.drawString(0, 6, "Dump:");

    labelsDrawn = true;
  }

  auto drawValue = [&](uint8_t row, uint8_t col, uint8_t idx, const String& text) {
    if (idx >= 5) return;
    String padded = text;
    const uint8_t maxLen = (col < 16) ? (16 - col) : 0;
    if (padded.length() > maxLen) padded = padded.substring(0, maxLen);
    while (padded.length() < maxLen) padded += ' ';
    if (lastVals[idx] == padded) return;
    display.drawString(col, row, padded.c_str());
    lastVals[idx] = padded;
  };

  auto status = link.getStatus();
  bool armed = (status.status & ROBOT_STATUS_ARMED);
  bool estop = (status.status & ROBOT_STATUS_ESTOP);
  bool fault = (status.status & ROBOT_STATUS_FAULT) || (status.faults != 0);
  bool dumping = (status.status & ROBOT_STATUS_DUMPING) != 0;
  const bool imuCal = (status.status & ROBOT_STATUS_IMU_CAL) != 0;
  const char* imuState = "--";
  if (status.hasTelem) {
    if (fault) {
      imuState = "FAULT";
    } else if (!imuCal) {
      imuState = "UNCAL";
    } else {
      imuState = "OK";
    }
  }
  const char* robotState = fault ? "FAULT" : (estop ? "ESTOP" : (armed ? "ARMED" : "DISARM"));
  const bool connected = ctrl.isConnected();

  // Format battery voltage
  String voltageStr = "--";
  if (status.hasTelem && status.adcVoltage > 0.01f) {
    voltageStr = String(status.adcVoltage, 2) + "V";
  }
  const String dumpStr = dumping ? "Dumping ..." : "";

  drawValue(2, 5, 0, imuState);
  drawValue(3, 9, 3, connected ? "OK" : "--");
  drawValue(4, 7, 1, robotState);
  drawValue(5, 7, 2, voltageStr);
  drawValue(6, 4, 4, dumpStr);
  
}

static void renderPassthroughScreen(RpPassthrough& pt, bool forceRedraw) {
  static bool labelsDrawn = false;
  static String lastVals[3];
  if (forceRedraw) {
    labelsDrawn = false;
    for (auto& v : lastVals) {
      v = "";
    }
  }
  if (!labelsDrawn) {
    display.clear();
    display.drawString(0, 0, "RP Passthrough");
    display.drawString(0, 2, "WiFi:");
    display.drawString(0, 5, "Client:");
    labelsDrawn = true;
  }

  auto drawValue = [&](uint8_t row, uint8_t col, uint8_t idx, const String& text) {
    if (idx >= 3) return;
    String padded = text;
    const uint8_t maxLen = (col < 16) ? (16 - col) : 0;
    if (padded.length() > maxLen) padded = padded.substring(0, maxLen);
    while (padded.length() < maxLen) padded += ' ';
    if (lastVals[idx] == padded) return;
    display.drawString(col, row, padded.c_str());
    lastVals[idx] = padded;
  };

  const char* wifiMode = pt.isApMode() ? "AP" : "STA";
  const char* ip = pt.ipAddress();
  const char* client = pt.hasClient() ? "OK" : "--";

  drawValue(2, 6, 0, wifiMode);
  drawValue(3, 0, 1, ip ? ip : "");
  drawValue(5, 8, 2, client);
}

static void debugControls(const XboxControlsState& s) {
  Serial.printf(
      "[CTRL] LX=%.2f LY=%.2f RX=%.2f RY=%.2f LT=%.2f RT=%.2f "
      "A=%d B=%d X=%d Y=%d LB=%d RB=%d Menu=%d View=%d Share=%d "
      "Dpad(U%d D%d L%d R%d) LS=%d RS=%d\n",
      s.leftStickX, s.leftStickY, s.rightStickX, s.rightStickY, s.leftTrigger, s.rightTrigger,
      s.buttonA, s.buttonB, s.buttonX, s.buttonY, s.leftBumper, s.rightBumper, s.menuButton,
      s.viewButton, s.shareButton, s.dpadUp, s.dpadDown, s.dpadLeft, s.dpadRight, s.leftStickButton,
      s.rightStickButton);
}

static bool controlsChanged(const XboxControlsState& a, const XboxControlsState& b) {
  return memcmp(&a, &b, sizeof(XboxControlsState)) != 0;
}

static void logLoopStatus(uint32_t now, bool ctrlConnected) {
  if (now - gLoopReportMs < 1000U) {
    return;
  }
  gLoopReportMs = now;

  const uint32_t telemPerSec = stmLink.takeTelemCount();
  const uint32_t lastTelem = stmLink.lastTelemRxMs();
  const uint32_t telemAge = (lastTelem > 0U) ? (now - lastTelem) : 0U;
  const auto status = stmLink.getStatus();

  Serial.printf(
      "[LOOP] ms=%lu ctrl=%u menu=%u pt=%u wifi=%u ap=%u client=%u telem=%lu/s last_telem_ms=%lu "
      "age=%lu link_ok=%u has_telem=%u faults=0x%04x\n",
      static_cast<unsigned long>(now), ctrlConnected ? 1U : 0U, gMenuActive ? 1U : 0U,
      rpPassthrough.isActive() ? 1U : 0U, rpPassthrough.wifiReady() ? 1U : 0U,
      rpPassthrough.isApMode() ? 1U : 0U, rpPassthrough.hasClient() ? 1U : 0U,
      static_cast<unsigned long>(telemPerSec), static_cast<unsigned long>(lastTelem),
      static_cast<unsigned long>(telemAge), status.linkOk ? 1U : 0U, status.hasTelem ? 1U : 0U,
      status.faults);
}

static void enterPassthrough() {
  if (rpPassthrough.isActive()) {
    return;
  }
  rpPassthrough.setActive(true);
  MenuUI::setIpString(rpPassthrough.ipAddress());
  if (!rpPassthrough.isActive()) {
    return;
  }
  stmLink.setPassthrough(true);
  MenuUI::setPassthroughActive(true);
  setMenuActive(false);
  gForcePassthroughRedraw = true;
}

static void exitPassthrough() {
  if (!rpPassthrough.isActive()) {
    return;
  }
  rpPassthrough.setActive(false);
  MenuUI::setIpString(rpPassthrough.ipAddress());
  stmLink.setPassthrough(false);
  MenuUI::setPassthroughActive(false);
  setMenuActive(false);
  gForceDashboardRedraw = true;
}

void setup(void) {
  Serial.begin(115200);
  controller.begin();
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  display.setI2CAddress(I2C_ADDR_8BIT);
  display.begin();
  display.setFont(u8x8_font_chroma48medium8_r);
  stmLink.begin();
  rpPassthrough.begin();
  MenuUI::setIpString(rpPassthrough.ipAddress());
  splash();
  setMenuActive(false);  // start in dashboard/idle mode
  gBootMs = millis();

  if (gStmLinkTaskHandle == nullptr) {
    const BaseType_t ok =
        xTaskCreatePinnedToCore(stmLinkTask, "Stm32Link", RP_STMLINK_TASK_STACK_WORDS, nullptr,
                                RP_STMLINK_TASK_PRIORITY, &gStmLinkTaskHandle, 1);
    if (ok != pdPASS) {
      Serial.println("[LINK] Failed to start Stm32Link task");
    }
  }

#if RP_PASSTHROUGH_FORCE_BOOT
  rpPassthrough.setActive(true);
  MenuUI::setIpString(rpPassthrough.ipAddress());
  if (rpPassthrough.isActive()) {
    stmLink.setPassthrough(true);
    MenuUI::setPassthroughActive(true);
  }
#endif
}

void loop() {
  const uint32_t now = millis();
  const bool ctrlConnected = controller.isConnected();
  XboxControlsState s{};
  bool haveState = false;

  if (ctrlConnected) {
    if (!g_connected) {
      g_connected = true;
      display.clear();
      display.drawString(0, 2, "Xbox Ctrlr");
      display.drawString(0, 4, "Connected");
      delay(1000);
      gForceDashboardRedraw = true;
      gForcePassthroughRedraw = true;
    }
    controller.read(&s);

    haveState = true;
    gControllerEverConnected = true;
    if (controlsChanged(s, gPrevState)) {
      debugControls(s);
      gPrevState = s;
    }

    const bool menuPressed = s.menuButton && !gPrevMenuButton;
    gPrevMenuButton = s.menuButton;
    if (menuPressed) {
      setMenuActive(!gMenuActive);
    }
    const bool yPressed = s.buttonY;
    if (yPressed && !gPrevYButton) {
      if (now - gLastYToggleMs >= kYToggleDebounceMs) {
        gTeleopEstopActive = !gTeleopEstopActive;
        gLastYToggleMs = now;
      }
    }
    gPrevYButton = yPressed;

    const bool xPressed = s.buttonX;
    if (xPressed && !gPrevXButton) {
      if (now - gLastXPressMs >= kYToggleDebounceMs) {
        gDumpRequested = true;
        gLastXPressMs = now;
        Serial.println("[ESP32] X button pressed - dump requested");
      }
    }
    gPrevXButton = xPressed;
  } else {
    g_connected = false;
    gPrevState = {};
    gPrevMenuButton = false;
    gPrevYButton = false;
    gPrevXButton = false;
    gDumpRequested = false;
    gTeleopEstopActive = true;
  }

  if (!rpPassthrough.isActive() && !gControllerEverConnected && !ctrlConnected &&
      RP_PASSTHROUGH_NO_CTRL_TIMEOUT_MS > 0U &&
      (now - gBootMs >= RP_PASSTHROUGH_NO_CTRL_TIMEOUT_MS)) {
    enterPassthrough();
  }

  if (rpPassthrough.isActive()) {
    rpPassthrough.tick();
    if (ctrlConnected && gMenuActive) {
      gpIn.update(s);
      nav.poll();
    } else {
      renderPassthroughScreen(rpPassthrough, gForcePassthroughRedraw);
      gForcePassthroughRedraw = false;
    }

    MenuUI::PassthroughRequest req = MenuUI::takePassthroughRequest();
    if (req == MenuUI::PassthroughRequest::Exit) {
      exitPassthrough();
    }

    logLoopStatus(now, ctrlConnected);
    return;
  }

  uint8_t teleopFlags =
      gTeleopEstopActive ? ROBOT_TELEOP_FLAG_ESTOP : ROBOT_TELEOP_FLAG_ARM;
  const bool dumpPending = gDumpRequested;
  if (dumpPending) {
    teleopFlags |= ROBOT_TELEOP_FLAG_DUMP;
  }
  if (ctrlConnected) {
    if (gMenuActive) {
      gpIn.update(s);
      nav.poll();
    } else {
      XboxControlsState filtered = s;
      filtered.leftStickY = applyDeadzone(filtered.leftStickY, kTeleopDeadzone);
      filtered.leftStickX = applyDeadzone(filtered.leftStickX, kTeleopDeadzone);
      stmLink.sendTeleop(filtered, teleopFlags);
      if (dumpPending) {
        gDumpRequested = false;
        Serial.printf("[ESP32] Adding DUMP flag to teleop, flags=0x%02x\n", teleopFlags);
      }

      renderDashboard(stmLink, controller, gForceDashboardRedraw);
      gForceDashboardRedraw = false;
    }
  } else {
    renderDashboard(stmLink, controller, gForceDashboardRedraw);
    gForceDashboardRedraw = false;
  }

  MenuUI::PassthroughRequest req = MenuUI::takePassthroughRequest();
  if (req == MenuUI::PassthroughRequest::Enter) {
    enterPassthrough();
  }

  rpPassthrough.tick();
  logLoopStatus(now, ctrlConnected);
}
