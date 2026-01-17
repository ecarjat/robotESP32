#include "TeleopInput.h"

TeleopInput::TeleopInput(uint32_t debounceMs) : debounceMs_(debounceMs) {}

bool TeleopInput::update(const XboxControlsState& s, uint32_t nowMs) {
  bool dumpPressed = false;

  const bool yPressed = s.buttonY;
  if (yPressed && !prevY_) {
    if (nowMs - lastYToggleMs_ >= debounceMs_) {
      estopActive_ = !estopActive_;
      lastYToggleMs_ = nowMs;
    }
  }
  prevY_ = yPressed;

  const bool lbPressed = s.leftBumper;
  if (lbPressed && !prevLb_) {
    if (nowMs - lastLbToggleMs_ >= debounceMs_) {
      lqrMode_ = !lqrMode_;
      lastLbToggleMs_ = nowMs;
    }
  }
  prevLb_ = lbPressed;

  const bool xPressed = s.buttonX;
  if (xPressed && !prevX_) {
    if (nowMs - lastXPressMs_ >= debounceMs_) {
      dumpRequested_ = true;
      dumpPressed = true;
      lastXPressMs_ = nowMs;
    }
  }
  prevX_ = xPressed;

  return dumpPressed;
}

void TeleopInput::onDisconnect() {
  prevY_ = false;
  prevLb_ = false;
  prevX_ = false;
  dumpRequested_ = false;
  estopActive_ = true;
  lqrMode_ = false;
}

uint8_t TeleopInput::buildFlags() const {
  uint8_t flags = estopActive_ ? ROBOT_TELEOP_FLAG_ESTOP : ROBOT_TELEOP_FLAG_ARM;
  if (lqrMode_) {
    flags |= ROBOT_TELEOP_FLAG_LQR_MODE;
  }
  if (dumpRequested_) {
    flags |= ROBOT_TELEOP_FLAG_DUMP;
  }
  return flags;
}

bool TeleopInput::onTeleopSent(uint8_t sentFlags) {
  if ((sentFlags & ROBOT_TELEOP_FLAG_DUMP) != 0U && dumpRequested_) {
    dumpRequested_ = false;
    return true;
  }
  return false;
}

bool TeleopInput::isLqrMode() const {
  return lqrMode_;
}
