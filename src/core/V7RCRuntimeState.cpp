#include "V7RCRuntimeState.h"

V7RCRuntimeState::V7RCRuntimeState() : lastFrameMs_(0), hasFrame_(false) {
  reset();
}

void V7RCRuntimeState::reset() {
  for (uint8_t i = 0; i < kNumChannels; ++i) {
    channels_[i] = 1500;
  }
  lastFrameMs_ = 0;
  hasFrame_ = false;
}

void V7RCRuntimeState::setChannel(uint8_t index, int16_t value) {
  if (index >= kNumChannels) {
    return;
  }
  channels_[index] = value;
}

int16_t V7RCRuntimeState::getChannel(uint8_t index) const {
  if (index >= kNumChannels) {
    return 1500;
  }
  return channels_[index];
}

void V7RCRuntimeState::markFrameReceived(unsigned long nowMs) {
  lastFrameMs_ = nowMs;
  hasFrame_ = true;
}

unsigned long V7RCRuntimeState::lastFrameMs() const {
  return lastFrameMs_;
}

bool V7RCRuntimeState::hasFrame() const {
  return hasFrame_;
}

bool V7RCRuntimeState::signalValid(unsigned long nowMs, unsigned long timeoutMs) const {
  if (!hasFrame_) {
    return false;
  }
  return (nowMs - lastFrameMs_) <= timeoutMs;
}
