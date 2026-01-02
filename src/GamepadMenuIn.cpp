#include "GamepadMenuIn.h"

static constexpr float STICK_DEAD = 0.55f;  // threshold for “pressed”
static constexpr uint32_t REPEAT_MS = 300;  // optional auto-repeat

GamepadMenuIn::GamepadMenuIn() : head_(0), tail_(0) {}

// Call this every loop after reading controller state
void GamepadMenuIn::update(const XboxControlsState& s) {
  const uint32_t now = millis();

  // --- DIGITALIZE STICKS (edge detect + optional repeat) ---
  const bool downNow = (s.leftStickY > STICK_DEAD);
  const bool upNow = (s.leftStickY < -STICK_DEAD);
  const bool rightNow = (s.leftStickX > STICK_DEAD);
  const bool leftNow = (s.leftStickX < -STICK_DEAD);

  emitEdgeOrRepeat(upNow, upPrev_, upNextRepeat_, now, defaultNavCodes[upCmd].ch);
  emitEdgeOrRepeat(downNow, downPrev_, downNextRepeat_, now, defaultNavCodes[downCmd].ch);
  emitEdgeOrRepeat(leftNow, leftPrev_, leftNextRepeat_, now, defaultNavCodes[leftCmd].ch);
  emitEdgeOrRepeat(rightNow, rightPrev_, rightNextRepeat_, now, defaultNavCodes[rightCmd].ch);

  // --- OPTIONAL: MAP BUTTONS TO ENTER / ESC ---
  // You will probably need to rename these fields to whatever XboxControlsState exposes in your
  // version. Typical guesses are: s.a / s.b OR s.buttonA / s.buttonB.
  //
  bool aNow = s.buttonA;   // <-- adjust if it doesn't compile
  bool bNow = s.buttonB;   // <-- adjust if it doesn't compile
 
  //
  if (aNow && !aPrev_) push(defaultNavCodes[enterCmd].ch);
  if (bNow && !bPrev_) push(defaultNavCodes[escCmd].ch);
  aPrev_ = aNow;
  bPrev_ = bNow;
}

// --- menuIn interface ---
int GamepadMenuIn::available() { return head_ != tail_; }
int GamepadMenuIn::read() {
  if (!available()) return -1;
  const uint8_t c = buf_[tail_];
  tail_ = (tail_ + 1) % kBufSize;
  return c;
}
int GamepadMenuIn::peek() { return available() ? buf_[tail_] : -1; }
void GamepadMenuIn::flush() { head_ = tail_ = 0; }
size_t GamepadMenuIn::write(uint8_t) { return 1; }  // not used

void GamepadMenuIn::push(uint8_t c) {
  const uint8_t next = (head_ + 1) % kBufSize;
  if (next == tail_) return;  // full, drop
  buf_[head_] = c;
  head_ = next;
}

void GamepadMenuIn::emitEdgeOrRepeat(bool nowPressed, bool& prevPressed, uint32_t& nextRepeatAt,
                                     uint32_t now, uint8_t key) {
  if (nowPressed && !prevPressed) {
    push(key);
    nextRepeatAt = now + REPEAT_MS;
  } else if (nowPressed && prevPressed) {
    if ((int32_t)(now - nextRepeatAt) >= 0) {
      push(key);
      nextRepeatAt = now + REPEAT_MS;
    }
  }
  prevPressed = nowPressed;
}
