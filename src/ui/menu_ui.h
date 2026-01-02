#pragma once
#include <stdint.h>
#include <menu.h>

namespace MenuUI {

// Root menu exported for NAVROOT(...)
extern Menu::menu mainMenu;

enum class PassthroughRequest : uint8_t {
  None = 0,
  Enter,
  Exit
};

// Optional: keep external state in sync
void setOrientationSigns(int8_t x, int8_t y, int8_t z);
void setMotorDirectionSigns(int8_t left, int8_t right);
void setIpString(const char* ip);
void setPassthroughActive(bool active);
PassthroughRequest takePassthroughRequest();

// Optional: let main loop service continuous actions (motor test)
bool motorTestIsRunning();
void motorTestServiceTick(); // call from loop() at ~20ms if running

} // namespace MenuUI
