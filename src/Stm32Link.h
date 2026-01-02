#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

#include "GamepadMenuIn.h"
#include "../common/shared_protocol/robot_protocol.h"
#include "../common/mux_channels.h"

class Stm32Link {
 public:
  explicit Stm32Link(HardwareSerial& port);

  void begin();
  void tick();

  void sendHeartbeat();
  void sendTeleop(const XboxControlsState& state, uint8_t flags = 0);

  void setPassthrough(bool enable);
  void trySendTeleop();
  uint32_t takeTelemCount();
  uint32_t lastTelemRxMs() const;

  struct LinkStatus {
    bool hasTelem = false;
    uint8_t status = 0;
    uint16_t faults = 0;
    bool linkOk = false;
  };
  LinkStatus getStatus() const;

 private:
  struct PendingAck {
    bool inUse = false;
    uint8_t type = 0;
    uint16_t seq = 0;
    uint8_t retries = 0;
    uint32_t nextRetryMs = 0;
    size_t encodedLen = 0;
    uint8_t encoded[ROBOT_FRAME_MAX_ENCODED];
  };

  void handleIncoming();
  void processFrame(const robot_frame_t& frame);
  void handleAck(const robot_frame_t& frame);
  void sendAck(uint16_t seq);
  bool sendFrame(uint8_t type, const uint8_t* payload, uint16_t len, bool requestAck, bool nonBlocking = false);
  void pumpRetries(uint32_t now);

  HardwareSerial& port_;
  robot_mux_t mux_;
  mutable portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;
  PendingAck pending_;
  robot_cmd_teleop_t pendingTeleop_{};
  uint8_t pendingTeleopFlags_ = 0;
  bool hasPendingTeleop_ = false;
  uint16_t seqCounter_ = 0;
  uint8_t rxBuf_[ROBOT_FRAME_MAX_ENCODED];
  size_t rxLen_ = 0;
  uint32_t lastHeartbeatMs_ = 0;
  uint32_t lastHeartbeatWarnMs_ = 0;
  uint32_t missedHeartbeatCount_ = 0;
  uint32_t lastTeleopMs_ = 0;
  uint32_t telemCount_ = 0;
  uint32_t lastTelemRxMs_ = 0;
  volatile bool passthrough_ = false;
  LinkStatus status_;
};
