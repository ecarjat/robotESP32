#include "menu_ui.h"
#include <Arduino.h>

using namespace Menu;

namespace MenuUI {

// ----- Local state (menu-owned) -----
static int8_t orientSignX = +1, orientSignY = +1, orientSignZ = +1;
static int8_t dirLeft = +1, dirRight = +1;
static bool motorTestRunning = false;
static uint8_t motorConfigSide = 0;   // 0=Left,1=Right
static bool motorPassthroughActive = false;
static bool rpPassthroughActive = false;
static PassthroughRequest pendingRpRequest = PassthroughRequest::None;
static const char* ipStr = "0.0.0.0";

// IMU wizard state (simple)
struct ImuCalibWizard { bool active=false; uint8_t step=0; } imuWiz;
static const char* kCalibFaces[6] = {"Standing","Inverted","Front","Rear","Left","Right"};

// ----- Replace these with your real display hooks if you want -----
static void status2(const char* l1, const char* l2) {
  // Keep it non-blocking: just print to Serial here.
  // If you want OLED feedback, call your display::showXxx() from here instead.
  Serial.println(l1);
  if (l2) Serial.println(l2);
}

// ----- Handlers (drop your existing calls into TODOs) -----
static result startImuWizard() {
  imuWiz.active = true;
  imuWiz.step = 0;
  status2("IMU Calib", kCalibFaces[imuWiz.step]);
  // TODO: mode=CALIBRATING, motors off
  return proceed;
}

static result captureImuFace() {
  if (!imuWiz.active) startImuWizard();

  // TODO: imu::captureCalibrationFace(kCalibFaces[imuWiz.step], ...)
  bool ok = true;

  if (!ok) { status2("Capture failed", "Back to exit"); return proceed; }

  if (imuWiz.step < 5) {
    imuWiz.step++;
    status2("IMU Calib", kCalibFaces[imuWiz.step]);
    return proceed;
  }

  // TODO: ok = imu::computeCalibrationFromFaces(...)
  ok = true;

  imuWiz.active = false;
  status2(ok ? "Calib OK" : "Calib ERROR", "Back to exit");
  // TODO: mode=STAND_BY
  return proceed;
}

static result runMotorDriverCalib() {
  status2("Motor Calib", "Running...");
  // TODO: bool ok = motors_binaryIO::calibrateDrivers();
  bool ok = true;
  status2(ok ? "Motor Calib OK" : "Motor Calib ERR", "Back to exit");
  return proceed;
}

static result doUprightZero() {
  status2("Upright Zero", "Capturing...");
  // TODO: bool ok = imu::captureUprightOffset();
  bool ok = true;
  status2(ok ? "Upright OK" : "Upright ERR", "Back to exit");
  return proceed;
}

static result saveOrientationSigns() {
  status2("Orient Signs", "Saving...");
  // TODO: imu::setCalibration(...persist=true) using orientSignX/Y/Z
  status2("Orient Signs", "Saved");
  return proceed;
}

static result motorTestStartStop() {
  motorTestRunning = !motorTestRunning;
  status2("Motor Test", motorTestRunning ? "RUNNING" : "STOPPED");

  if (motorTestRunning) {
    // TODO: motors_binaryIO::enable(true);
    // TODO: motors_binaryIO::setDirectionSigns(dirLeft, dirRight, false);
  } else {
    // TODO: motors_binaryIO::setWheelVelocities(0,0,...); motors_binaryIO::enable(false);
  }
  return proceed;
}

static result motorTestSave() {
  status2("Motor Test", "Saving...");
  // TODO: motors_binaryIO::setDirectionSigns(dirLeft, dirRight, true);
  status2("Motor Test", "Saved");
  return proceed;
}

static result motorConfigStart() {
  motorPassthroughActive = true;
  status2("Passthrough", motorConfigSide==0 ? "LEFT" : "RIGHT");
  // TODO: muteSerialForMotorMenu(); startMotorPassthrough(side)
  return proceed;
}

static result motorConfigStop() {
  motorPassthroughActive = false;
  status2("Passthrough", "Stopping...");
  // TODO: requestStopMotorPassthrough(); restoreSerialAfterMotorMenu() when done
  status2("Passthrough", "Stopped");
  return proceed;
}

static result showIp() {
  status2("IP Address", ipStr);
  return proceed;
}

static result rpConfirmEnter() {
  pendingRpRequest = PassthroughRequest::Enter;
  status2("RP Passthrough", "Enter requested");
  return proceed;
}

static result rpConfirmExit() {
  pendingRpRequest = PassthroughRequest::Exit;
  status2("RP Passthrough", "Exit requested");
  return proceed;
}

// ----- Submenus -----
MENU(imuCalibMenu, "IMU Calibration", doNothing, noEvent, wrapStyle,
  OP("Capture / Next", captureImuFace, enterEvent),
  EXIT("<Back")
);

TOGGLE(orientSignX, orientXMenu, "Axis X: ", doNothing, noEvent, wrapStyle,
  VALUE("+", +1, doNothing, enterEvent),
  VALUE("-", -1, doNothing, enterEvent)
);
TOGGLE(orientSignY, orientYMenu, "Axis Y: ", doNothing, noEvent, wrapStyle,
  VALUE("+", +1, doNothing, enterEvent),
  VALUE("-", -1, doNothing, enterEvent)
);
TOGGLE(orientSignZ, orientZMenu, "Axis Z: ", doNothing, noEvent, wrapStyle,
  VALUE("+", +1, doNothing, enterEvent),
  VALUE("-", -1, doNothing, enterEvent)
);

MENU(orientMenu, "Orientation Signs", doNothing, noEvent, wrapStyle,
  SUBMENU(orientXMenu),
  SUBMENU(orientYMenu),
  SUBMENU(orientZMenu),
  OP("Save", saveOrientationSigns, enterEvent),
  EXIT("<Back")
);

MENU(motorCalibMenu, "Motor Calibration", doNothing, noEvent, wrapStyle,
  OP("Run calibration", runMotorDriverCalib, enterEvent),
  EXIT("<Back")
);

TOGGLE(dirLeft, dirLeftMenu, "Left Dir: ", doNothing, noEvent, wrapStyle,
  VALUE("+", +1, doNothing, enterEvent),
  VALUE("-", -1, doNothing, enterEvent)
);
TOGGLE(dirRight, dirRightMenu, "Right Dir: ", doNothing, noEvent, wrapStyle,
  VALUE("+", +1, doNothing, enterEvent),
  VALUE("-", -1, doNothing, enterEvent)
);

MENU(motorTestMenu, "Motor Dir Test", doNothing, noEvent, wrapStyle,
  SUBMENU(dirLeftMenu),
  SUBMENU(dirRightMenu),
  OP("Start/Stop", motorTestStartStop, enterEvent),
  OP("Save", motorTestSave, enterEvent),
  EXIT("<Back")
);

SELECT(motorConfigSide, sideMenu, "Side: ", doNothing, noEvent, wrapStyle,
  VALUE("Left",  0, doNothing, noEvent),
  VALUE("Right", 1, doNothing, noEvent)
);

MENU(motorConfigMenu, "Motor Config", doNothing, noEvent, wrapStyle,
  SUBMENU(sideMenu),
  OP("Start passthrough", motorConfigStart, enterEvent),
  OP("Stop passthrough",  motorConfigStop,  enterEvent),
  EXIT("<Back")
);

MENU(rpMenu, "RP Passthrough", doNothing, noEvent, wrapStyle,
  OP("Enter RP (OK)", rpConfirmEnter, enterEvent),
  OP("Exit RP (OK)", rpConfirmExit, enterEvent),
  OP("Show IP", showIp, enterEvent),
  EXIT("<Back")
);

MENU(toolsMenu, "Tools", doNothing, noEvent, wrapStyle,
  SUBMENU(rpMenu),
  EXIT("<Back")
);

// ----- Main menu (7 items) -----
MENU(mainMenu, "Main", doNothing, noEvent, wrapStyle,
  SUBMENU(imuCalibMenu),                    // 0) IMU Calibration Wizard
  SUBMENU(orientMenu),                      // 1) Orientation Sign Menu
  SUBMENU(motorCalibMenu),                  // 2) Motor Calibration Wizard
  SUBMENU(motorTestMenu),                   // 3) Motor Direction Test
  OP("Upright Zero", doUprightZero, enterEvent), // 4) Upright Zero
  SUBMENU(motorConfigMenu),                 // 5) Motor Config Menu
  SUBMENU(toolsMenu)                        // 6) Tools
);

// ----- API -----
void setOrientationSigns(int8_t x, int8_t y, int8_t z) { orientSignX=x; orientSignY=y; orientSignZ=z; }
void setMotorDirectionSigns(int8_t left, int8_t right) { dirLeft=left; dirRight=right; }
void setIpString(const char* ip) { ipStr = ip ? ip : "0.0.0.0"; }
void setPassthroughActive(bool active) { rpPassthroughActive = active; }

PassthroughRequest takePassthroughRequest() {
  PassthroughRequest req = pendingRpRequest;
  pendingRpRequest = PassthroughRequest::None;
  return req;
}

bool motorTestIsRunning() { return motorTestRunning; }

void motorTestServiceTick() {
  if (!motorTestRunning) return;
  // TODO: motors_binaryIO::setWheelVelocities(2.0f, 2.0f, PARAM_MAX_WHEEL_VELOCITY);
}

} // namespace MenuUI
