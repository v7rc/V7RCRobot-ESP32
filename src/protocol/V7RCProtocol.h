#pragma once

#include <stdint.h>
#include <stddef.h>

enum V7RC_ProtocolType : uint8_t {
  V7RC_NONE = 0,
  V7RC_HEX,
  V7RC_DEG,
  V7RC_SRV,
  V7RC_SS8,
  V7RC_LED
};

struct V7RC_Frame {
  V7RC_ProtocolType type;
  int16_t values[16];
  bool channelPresent[16];
  bool valid;
};

class V7RCProtocolDecoder {
public:
  static const int kNumChannels = 16;
  static const int kBinaryFrameSize = 20;

  static V7RC_Frame decode(const char* buffer, size_t length);
};
