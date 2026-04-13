#include "V7RCBleTransportEsp32.h"

#include <NimBLEDevice.h>
#include <string>

namespace {

static const char* kServiceUuid = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char* kRxUuid = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
static const char* kTxUuid = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

}  // namespace

class V7RCBleTransportEsp32::Impl {
public:
  explicit Impl(V7RCBleTransportEsp32& owner) : owner_(owner) {}

  void begin(const char* deviceName) {
    NimBLEDevice::init(deviceName ? deviceName : "V7RC");
    NimBLEDevice::setMTU(247);

    server_ = NimBLEDevice::createServer();
    serverCallbacks_ = new ServerCallbacks(*this);
    server_->setCallbacks(serverCallbacks_);

    NimBLEService* service = server_->createService(NimBLEUUID(kServiceUuid));

    rxCharacteristic_ = service->createCharacteristic(
      NimBLEUUID(kRxUuid),
      NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    characteristicCallbacks_ = new CharacteristicCallbacks(*this);
    rxCharacteristic_->setCallbacks(characteristicCallbacks_);

    txCharacteristic_ = service->createCharacteristic(
      NimBLEUUID(kTxUuid),
      NIMBLE_PROPERTY::NOTIFY
    );
    txCallbacks_ = new TxCharacteristicCallbacks(*this);
    txCharacteristic_->setCallbacks(txCallbacks_);

    service->start();

    NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
    advertising->reset();
    advertising->addServiceUUID(service->getUUID());
    advertising->setName(deviceName ? deviceName : "V7RC");
    NimBLEDevice::startAdvertising();
  }

  size_t send(const uint8_t* data, size_t length) {
    if (!connected_ || !txCharacteristic_ || !data || length == 0 || !txSubscribed_) {
      return 0;
    }

    return txCharacteristic_->notify(data, length) ? length : 0;
  }

  bool isConnected() const {
    return connected_;
  }

private:
  class ServerCallbacks : public NimBLEServerCallbacks {
  public:
    explicit ServerCallbacks(Impl& impl) : impl_(impl) {}

    void onConnect(NimBLEServer* server, NimBLEConnInfo& connInfo) override {
      (void)server;
      (void)connInfo;
      impl_.connected_ = true;
      impl_.owner_.emitConnectionChanged(true);
    }

    void onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo, int reason) override {
      (void)server;
      (void)connInfo;
      (void)reason;
      impl_.connected_ = false;
      impl_.owner_.emitConnectionChanged(false);
      NimBLEDevice::startAdvertising();
    }

  private:
    Impl& impl_;
  };

  class CharacteristicCallbacks : public NimBLECharacteristicCallbacks {
  public:
    explicit CharacteristicCallbacks(Impl& impl) : impl_(impl) {}

    void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo) override {
      (void)connInfo;

      std::string value = characteristic->getValue();
      for (size_t i = 0; i < value.size(); ++i) {
        impl_.owner_.emitByte(static_cast<uint8_t>(value[i]));
      }
    }

  private:
    Impl& impl_;
  };

  class TxCharacteristicCallbacks : public NimBLECharacteristicCallbacks {
  public:
    explicit TxCharacteristicCallbacks(Impl& impl) : impl_(impl) {}

    void onSubscribe(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo, uint16_t subValue) override {
      (void)characteristic;
      (void)connInfo;
      impl_.txSubscribed_ = (subValue != 0);
    }

  private:
    Impl& impl_;
  };

  V7RCBleTransportEsp32& owner_;
  NimBLEServer* server_ = nullptr;
  NimBLECharacteristic* rxCharacteristic_ = nullptr;
  NimBLECharacteristic* txCharacteristic_ = nullptr;
  ServerCallbacks* serverCallbacks_ = nullptr;
  CharacteristicCallbacks* characteristicCallbacks_ = nullptr;
  TxCharacteristicCallbacks* txCallbacks_ = nullptr;
  bool connected_ = false;
  bool txSubscribed_ = false;
};

V7RCBleTransportEsp32::V7RCBleTransportEsp32() : impl_(new Impl(*this)) {}

V7RCBleTransportEsp32::~V7RCBleTransportEsp32() {
  delete impl_;
}

void V7RCBleTransportEsp32::begin(const char* deviceName) {
  impl_->begin(deviceName);
}

void V7RCBleTransportEsp32::poll() {}

bool V7RCBleTransportEsp32::isConnected() const {
  return impl_->isConnected();
}

size_t V7RCBleTransportEsp32::send(const uint8_t* data, size_t length) {
  return impl_->send(data, length);
}
