#include "Stm32Link.h"

#include <cstring>

#include "config_pins.h"
#include "rp_passthrough_config.h"
static constexpr uint32_t LINK_BAUD = 921600;

// Timings (ms)
static constexpr uint32_t HEARTBEAT_PERIOD_MS = 100;  // 10 Hz
static constexpr uint32_t TELEOP_PERIOD_MS = 20;      // 50 Hz
static constexpr uint32_t ACK_TIMEOUT_MS = 50;
static constexpr uint8_t MAX_RETRIES = 3;

Stm32Link::Stm32Link(HardwareSerial& port) : port_(port) {
  robot_mux_init(&mux_);
  robot_mux_register(
      &mux_, ROBOT_CHANNEL_TELEM,
      [](uint8_t, const uint8_t* payload, size_t len, void* ctx) {
        if (len < sizeof(robot_telem_v2_t)) return;
        const robot_telem_v2_t* telem = reinterpret_cast<const robot_telem_v2_t*>(payload);
        Stm32Link* self = static_cast<Stm32Link*>(ctx);
        const uint32_t now = millis();
        portENTER_CRITICAL(&self->lock_);
        self->telemCount_++;
        self->lastTelemRxMs_ = now;
        self->status_.hasTelem = true;
        self->status_.status = telem->status;
        self->status_.faults = telem->faults;
        self->status_.linkOk = true;
        portEXIT_CRITICAL(&self->lock_);
        Serial.printf(
            "[TELEM] v%u status=0x%02x faults=0x%04x ts=%lu theta=%.3f dtheta=%.3f wl=%.3f wr=%.3f iqL=%.3f iqR=%.3f\n",
            telem->version,
            telem->status,
            telem->faults,
            static_cast<unsigned long>(telem->timestamp_ms),
            telem->theta_rad,
            telem->theta_dot,
            telem->wheel_left_rps,
            telem->wheel_right_rps,
            telem->iq_left,
            telem->iq_right);
      },
      this);
  robot_mux_register(
      &mux_, ROBOT_CHANNEL_RPC,
      [](uint8_t type, const uint8_t* payload, size_t len, void* ctx) {
        (void)payload;
        (void)len;
        Serial.printf("[RPC] type=0x%02x len=%u\n", type, static_cast<unsigned int>(len));
      },
      this);
  robot_mux_register(
      &mux_, ROBOT_CHANNEL_FILE,
      [](uint8_t type, const uint8_t* payload, size_t len, void* ctx) {
        (void)payload;
        (void)ctx;
        Serial.printf("[FILE] type=0x%02x len=%u\n", type, static_cast<unsigned int>(len));
      },
      this);
}

void Stm32Link::begin() {
  port_.setPins(PIN_LINK_RX, PIN_LINK_TX, PIN_LINK_CTS, PIN_LINK_RTS);
  port_.setRxBufferSize(RP_STMLINK_RX_BUFFER_SIZE);
  port_.setTxBufferSize(RP_STMLINK_TX_BUFFER_SIZE);
  port_.begin(LINK_BAUD, SERIAL_8N1);
  port_.setHwFlowCtrlMode(UART_HW_FLOWCTRL_CTS_RTS, RP_STMLINK_FLOWCTRL_THRESHOLD);
  const uint32_t now = millis();
  lastTxMs_ = now;
  lastHeartbeatMs_ = now;
}

void Stm32Link::tick() {
  if (passthrough_) {
    return;
  }

  trySendTeleop();

  const uint32_t now = millis();

  // Only send heartbeat when idle (no other frames recently sent).
  if (now - lastTxMs_ >= HEARTBEAT_PERIOD_MS) {
    if (sendFrame(ROBOT_MSG_CMD_HEARTBEAT, nullptr, 0, false, true)) {
      lastHeartbeatMs_ = now;
      missedHeartbeatCount_ = 0;
    } else {
      missedHeartbeatCount_++;
      if (now - lastHeartbeatWarnMs_ >= 250) {
        Serial.printf("[LINK] heartbeat missed (%lu ms since last, missed=%lu, tx_avail=%d)\n",
                      static_cast<unsigned long>(now - lastHeartbeatMs_),
                      static_cast<unsigned long>(missedHeartbeatCount_), port_.availableForWrite());
        lastHeartbeatWarnMs_ = now;
      }
    }
  }

  pumpRetries(now);

  handleIncoming();
}

void Stm32Link::sendTeleop(const XboxControlsState& state, uint8_t flags) {
  if (passthrough_) {
    return;
  }

  robot_cmd_teleop_t cmd{};
  cmd.vx_mps = state.leftStickY;
  cmd.wz_radps = -state.leftStickX;
  cmd.flags = flags;

  portENTER_CRITICAL(&lock_);
  pendingTeleop_ = cmd;
  pendingTeleopFlags_ = flags;
  hasPendingTeleop_ = true;
  portEXIT_CRITICAL(&lock_);
}

void Stm32Link::setPassthrough(bool enable) {
  passthrough_ = enable;
  if (passthrough_) {
    rxLen_ = 0;
    pending_.inUse = false;
    hasPendingTeleop_ = false;
    telemCount_ = 0;
    lastTelemRxMs_ = 0;
  }
}

uint32_t Stm32Link::takeTelemCount() {
  portENTER_CRITICAL(&lock_);
  const uint32_t count = telemCount_;
  telemCount_ = 0;
  portEXIT_CRITICAL(&lock_);
  return count;
}

uint32_t Stm32Link::lastTelemRxMs() const {
  portENTER_CRITICAL(&lock_);
  const uint32_t val = lastTelemRxMs_;
  portEXIT_CRITICAL(&lock_);
  return val;
}

Stm32Link::LinkStatus Stm32Link::getStatus() const {
  portENTER_CRITICAL(&lock_);
  const LinkStatus snapshot = status_;
  portEXIT_CRITICAL(&lock_);
  return snapshot;
}

void Stm32Link::handleIncoming() {
  uint32_t iterCount = 0;
  while (port_.available()) {
    iterCount++;
    uint8_t b = static_cast<uint8_t>(port_.read());
    if (b == 0x00U) {
      if (rxLen_ == 0U) {
        continue;
      }
      if (rxLen_ < sizeof(rxBuf_)) {
        rxBuf_[rxLen_] = 0x00U;
      }
      robot_frame_t frame;

      if (robot_frame_decode(rxBuf_, rxLen_ + 1U, &frame)) {
        processFrame(frame);

      } else {
      }
      rxLen_ = 0;
    } else {
      if (rxLen_ < sizeof(rxBuf_)) {
        rxBuf_[rxLen_++] = b;
      } else {
        // overflow: drop current frame
        rxLen_ = 0;
      }
    }
  }
  if (iterCount > 0U) {
    Serial.printf("[TRACE][RX] loop exit iter=%lu avail=%d rxLen=%u\n",
                  static_cast<unsigned long>(iterCount), port_.available(),
                  static_cast<unsigned>(rxLen_));
  }
}

void Stm32Link::processFrame(const robot_frame_t& frame) {
  if (frame.hdr.flags & ROBOT_FLAG_IS_ACK) {
    handleAck(frame);
    return;
  }

  portENTER_CRITICAL(&lock_);
  status_.linkOk = true;
  portEXIT_CRITICAL(&lock_);
  if (frame.hdr.flags & ROBOT_FLAG_ACK_REQ) {
    sendAck(frame.hdr.seq);
  }

  robot_mux_dispatch(&mux_, frame.hdr.type, frame.payload, frame.hdr.len);
}

void Stm32Link::handleAck(const robot_frame_t& frame) {
  if (!pending_.inUse) return;
  if (frame.hdr.seq != pending_.seq) return;
  pending_.inUse = false;
}

void Stm32Link::sendAck(uint16_t seq) {
  robot_frame_t ack;
  if (!robot_frame_init(&ack, ROBOT_MSG_ACK, seq, ROBOT_FLAG_IS_ACK, nullptr, 0)) {
    return;
  }
  uint8_t encoded[ROBOT_FRAME_MAX_ENCODED];
  size_t encodedLen = 0;
  if (robot_frame_encode(&ack, encoded, sizeof(encoded), &encodedLen) &&
      port_.availableForWrite() >= encodedLen) {
    port_.write(encoded, encodedLen);
  }
}

bool Stm32Link::sendFrame(uint8_t type, const uint8_t* payload, uint16_t len, bool requestAck,
                          bool nonBlocking) {
  uint16_t flags = requestAck ? ROBOT_FLAG_ACK_REQ : 0U;
  robot_frame_t frame;
  if (!robot_frame_init(&frame, type, seqCounter_++, flags, payload, len)) {
    return false;
  }

  uint8_t encoded[ROBOT_FRAME_MAX_ENCODED];
  size_t encodedLen = 0;
  if (!robot_frame_encode(&frame, encoded, sizeof(encoded), &encodedLen)) {
    return false;
  }

  if (nonBlocking && port_.availableForWrite() < encodedLen) {
    return false;
  }

  port_.write(encoded, encodedLen);
  lastTxMs_ = millis();

  if (requestAck && robot_msg_needs_ack(type)) {
    pending_.inUse = true;
    pending_.type = type;
    pending_.seq = frame.hdr.seq;
    pending_.retries = 0;
    pending_.nextRetryMs = millis() + ACK_TIMEOUT_MS;
    pending_.encodedLen = encodedLen;
    memcpy(pending_.encoded, encoded, encodedLen);
  }
  return true;
}

void Stm32Link::pumpRetries(uint32_t now) {
  if (!pending_.inUse) return;
  if (now < pending_.nextRetryMs) return;

  if (pending_.retries >= MAX_RETRIES) {
    Serial.printf("[LINK] ACK timeout type=0x%02x seq=%u\n", pending_.type, pending_.seq);
    pending_.inUse = false;
    return;
  }

  if (port_.availableForWrite() >= pending_.encodedLen) {
    port_.write(pending_.encoded, pending_.encodedLen);
    pending_.retries++;
    pending_.nextRetryMs = now + ACK_TIMEOUT_MS;
  }
}

void Stm32Link::trySendTeleop() {
  if (passthrough_) return;
  const uint32_t now = millis();
  if (now - lastTeleopMs_ < TELEOP_PERIOD_MS) {
    return;
  }

  robot_cmd_teleop_t cmd{};
  bool haveTeleop = false;
  portENTER_CRITICAL(&lock_);
  if (hasPendingTeleop_) {
    cmd = pendingTeleop_;
    haveTeleop = true;
  }
  portEXIT_CRITICAL(&lock_);

  if (!haveTeleop) {
    return;
  }
  uint8_t payload[sizeof(robot_cmd_teleop_t)];
  memcpy(payload, &cmd, sizeof(robot_cmd_teleop_t));
  if (sendFrame(ROBOT_MSG_CMD_TELEOP, payload, sizeof(robot_cmd_teleop_t), false, true)) {
    portENTER_CRITICAL(&lock_);
    hasPendingTeleop_ = false;
    lastTeleopMs_ = now;
    portEXIT_CRITICAL(&lock_);
  }
}
