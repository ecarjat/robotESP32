#pragma once

#include <BLEGamepadClient.h>
#include <stdint.h>

#include "../common/shared_protocol/robot_protocol.h"

class TeleopInput {
 public:
  explicit TeleopInput(uint32_t debounceMs = 250U);

  // Updates internal toggle state. Returns true if a dump request edge was detected.
  bool update(const XboxControlsState& s, uint32_t nowMs);

  void onDisconnect();

  uint8_t buildFlags() const;
  bool onTeleopSent(uint8_t sentFlags);
  bool isLqrMode() const;

 private:
  uint32_t debounceMs_;
  bool prevY_ = false;
  bool prevLb_ = false;
  bool prevX_ = false;
  bool estopActive_ = true;
  bool lqrMode_ = false;
  bool dumpRequested_ = false;
  uint32_t lastYToggleMs_ = 0;
  uint32_t lastLbToggleMs_ = 0;
  uint32_t lastXPressMs_ = 0;
};
