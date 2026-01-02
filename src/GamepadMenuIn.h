#pragma once
#include <BLEGamepadClient.h>
#include <menu.h>

class GamepadMenuIn : public menuIn {
 public:
  GamepadMenuIn();
  void update(const XboxControlsState& s);
  int available() override;
  int read() override;
  int peek() override;
  void flush() override;
  size_t write(uint8_t) override;

 private:
  static constexpr uint8_t kBufSize = 16;
  uint8_t buf_[kBufSize];
  volatile uint8_t head_, tail_;
  // state for edge/repeat
  bool upPrev_ = false, downPrev_ = false, leftPrev_ = false, rightPrev_ = false;
  uint32_t upNextRepeat_ = 0, downNextRepeat_ = 0, leftNextRepeat_ = 0, rightNextRepeat_ = 0;

  // optional buttons
  bool aPrev_ = false, bPrev_ = false;
  void push(uint8_t c);
  void emitEdgeOrRepeat(bool nowPressed, bool& prevPressed, uint32_t& nextRepeatAt, uint32_t now,
                        uint8_t key);
};
