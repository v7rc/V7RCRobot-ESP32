#pragma once

#include <Arduino.h>

typedef void (*V7RCTransportByteHandler)(uint8_t byte, void* context);
typedef void (*V7RCTransportConnectionHandler)(bool connected, void* context);

class V7RCTransport {
public:
  virtual ~V7RCTransport() {}

  void setByteHandler(V7RCTransportByteHandler handler, void* context) {
    byteHandler_ = handler;
    byteHandlerContext_ = context;
  }

  void setConnectionHandler(V7RCTransportConnectionHandler handler, void* context) {
    connectionHandler_ = handler;
    connectionHandlerContext_ = context;
  }

  virtual void begin(const char* deviceName) = 0;
  virtual void poll() = 0;
  virtual bool isConnected() const = 0;
  virtual size_t send(const uint8_t* data, size_t length) = 0;

protected:
  void emitByte(uint8_t byte) {
    if (byteHandler_) {
      byteHandler_(byte, byteHandlerContext_);
    }
  }

  void emitConnectionChanged(bool connected) {
    if (connectionHandler_) {
      connectionHandler_(connected, connectionHandlerContext_);
    }
  }

private:
  V7RCTransportByteHandler byteHandler_ = nullptr;
  void* byteHandlerContext_ = nullptr;
  V7RCTransportConnectionHandler connectionHandler_ = nullptr;
  void* connectionHandlerContext_ = nullptr;
};
