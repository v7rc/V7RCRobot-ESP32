#pragma once

#include "V7RCTransport.h"

class V7RCBleTransportEsp32 : public V7RCTransport {
public:
  V7RCBleTransportEsp32();
  ~V7RCBleTransportEsp32() override;

  void begin(const char* deviceName) override;
  void poll() override;
  bool isConnected() const override;
  size_t send(const uint8_t* data, size_t length) override;

private:
  class Impl;
  Impl* impl_;
};
