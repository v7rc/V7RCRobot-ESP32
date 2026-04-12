#pragma once

#include <Arduino.h>

class V7RCRuntimeState {
public:
  static const uint8_t kNumChannels = 16;

  V7RCRuntimeState();

  void reset();
  void setChannel(uint8_t index, int16_t value);
  int16_t getChannel(uint8_t index) const;

  void markFrameReceived(unsigned long nowMs);
  unsigned long lastFrameMs() const;
  bool hasFrame() const;
  bool signalValid(unsigned long nowMs, unsigned long timeoutMs) const;

private:
  int16_t channels_[kNumChannels];
  unsigned long lastFrameMs_;
  bool hasFrame_;
};
