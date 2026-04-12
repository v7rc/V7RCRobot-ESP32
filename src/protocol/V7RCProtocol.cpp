#include "V7RCProtocol.h"

#include <stdlib.h>
#include <string.h>

namespace {

int hexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
  if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
  return 0;
}

V7RC_Frame makeEmptyFrame() {
  V7RC_Frame frame;
  frame.type = V7RC_NONE;
  frame.valid = false;

  for (int i = 0; i < V7RCProtocolDecoder::kNumChannels; ++i) {
    frame.values[i] = 1500;
    frame.channelPresent[i] = false;
  }

  return frame;
}

bool hasHeader(const char* buffer, char a, char b, char c) {
  return buffer[0] == a && buffer[1] == b && buffer[2] == c;
}

}  // namespace

V7RC_Frame V7RCProtocolDecoder::decode(const char* buffer, size_t length) {
  V7RC_Frame frame = makeEmptyFrame();

  if (!buffer || length == 0) {
    return frame;
  }

  if (buffer[length - 1] != '#') {
    return frame;
  }

  if (length >= 3 && hasHeader(buffer, 'H', 'E', 'X')) {
    if (length != (size_t)kBinaryFrameSize) {
      return frame;
    }

    frame.type = V7RC_HEX;
    for (int i = 0; i < kNumChannels; ++i) {
      frame.values[i] = (int16_t)((uint8_t)buffer[3 + i] * 10);
      frame.channelPresent[i] = true;
    }
    frame.valid = true;
    return frame;
  }

  if (length >= 3 && hasHeader(buffer, 'D', 'E', 'G')) {
    if (length != (size_t)kBinaryFrameSize) {
      return frame;
    }

    frame.type = V7RC_DEG;
    for (int i = 0; i < kNumChannels; ++i) {
      frame.values[i] = (int16_t)((int8_t)buffer[3 + i] - 127);
      frame.channelPresent[i] = true;
    }
    frame.valid = true;
    return frame;
  }

  if (length >= 3 && (hasHeader(buffer, 'S', 'R', 'V') || hasHeader(buffer, 'S', 'R', 'T'))) {
    if (length != (size_t)kBinaryFrameSize) {
      return frame;
    }

    frame.type = V7RC_SRV;
    for (int i = 0; i < 4; ++i) {
      char tmp[5];
      int offset = 3 + i * 4;
      tmp[0] = buffer[offset + 0];
      tmp[1] = buffer[offset + 1];
      tmp[2] = buffer[offset + 2];
      tmp[3] = buffer[offset + 3];
      tmp[4] = '\0';

      int pwm = atoi(tmp);
      if (pwm < 0) pwm = 0;
      if (pwm > 3000) pwm = 3000;

      frame.values[i] = (int16_t)pwm;
      frame.channelPresent[i] = true;
    }
    frame.valid = true;
    return frame;
  }

  if (length >= 3 && hasHeader(buffer, 'S', 'S', '8')) {
    if (length != (size_t)kBinaryFrameSize) {
      return frame;
    }

    frame.type = V7RC_SS8;
    for (int i = 0; i < 8; ++i) {
      int offset = 3 + i * 2;
      int value = (hexNibble(buffer[offset]) << 4) | hexNibble(buffer[offset + 1]);
      frame.values[i] = (int16_t)(value * 10);
      frame.channelPresent[i] = true;
    }
    frame.valid = true;
    return frame;
  }

  if (length >= 3 && buffer[0] == 'L' && buffer[1] == 'E') {
    frame.type = V7RC_LED;
    frame.valid = true;
    return frame;
  }

  return frame;
}
