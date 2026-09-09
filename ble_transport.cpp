#include "ble_transport.h"

#include <NimBLEDevice.h>

static const char* SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
static const char* TX_UUID      = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";
static const char* RX_UUID      = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";

static NimBLECharacteristic* s_tx = nullptr;
static volatile bool s_connected = false;
static uint16_t s_mtu = 0;

// Inbound command lines, filled from the BLE callback and drained by loop().
static String s_rxBuffer;
static String s_rxLines[4];
static volatile uint8_t s_rxHead = 0;
static volatile uint8_t s_rxTail = 0;
static const uint8_t RX_SLOTS = 4;

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server, NimBLEConnInfo& info) override {
    s_connected = true;
    s_mtu = info.getMTU();
    Serial.printf("[BLE ] connected, MTU %u\n", s_mtu);
    if (s_mtu < 64) {
      // Frames are ~300 bytes. They still get through - send() chunks them -
      // but say so, because a stuck 23-byte MTU is worth noticing.
      Serial.println("[BLE ] MTU is small; frames will be split across packets");
    }
  }

  void onDisconnect(NimBLEServer* server, NimBLEConnInfo& info, int reason) override {
    s_connected = false;
    s_mtu = 0;
    Serial.printf("[BLE ] disconnected (reason %d), advertising again\n", reason);
    NimBLEDevice::startAdvertising();
  }

  void onMTUChange(uint16_t mtu, NimBLEConnInfo& info) override {
    s_mtu = mtu;
    Serial.printf("[BLE ] MTU now %u\n", mtu);
  }
};

class RxCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo& info) override {
    s_rxBuffer += characteristic->getValue().c_str();

    // The phone may split a command across writes, so lines are assembled
    // here rather than assuming one write is one command.
    int newline;
    while ((newline = s_rxBuffer.indexOf('\n')) >= 0) {
      String line = s_rxBuffer.substring(0, newline);
      s_rxBuffer.remove(0, newline + 1);
      line.trim();
      if (line.length() == 0) continue;

      uint8_t next = (uint8_t)((s_rxHead + 1) % RX_SLOTS);
      if (next == s_rxTail) continue;  // full: drop rather than block the stack
      s_rxLines[s_rxHead] = line;
      s_rxHead = next;
    }

    if (s_rxBuffer.length() > 256) s_rxBuffer = "";  // runaway write
  }
};

void BleTransport::setDeviceName(const char* deviceId) {
  strncpy(deviceName_, deviceId, sizeof(deviceName_) - 1);
  deviceName_[sizeof(deviceName_) - 1] = '\0';
}

void BleTransport::begin() {
  if (started_) return;

  NimBLEDevice::init(deviceName_);
  // Ask for the largest MTU the stack allows. The central decides, but
  // without asking, Android leaves it at the 23-byte default.
  NimBLEDevice::setMTU(517);

  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks(), true);

  NimBLEService* service = server->createService(SERVICE_UUID);

  s_tx = service->createCharacteristic(TX_UUID, NIMBLE_PROPERTY::NOTIFY);

  NimBLECharacteristic* rx = service->createCharacteristic(
      RX_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  rx->setCallbacks(new RxCallbacks());

  service->start();

  NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setName(deviceName_);
  advertising->enableScanResponse(true);
  advertising->start();

  started_ = true;
  Serial.printf("[BLE ] advertising as %s\n", deviceName_);
}

void BleTransport::send(const char* line) {
  if (!started_ || !s_connected || s_tx == nullptr) return;

  // A notification carries MTU-3 bytes. Telemetry frames are larger than the
  // default MTU, so they are split here and rejoined by the line parser on
  // the phone - the same reassembly the serial path already needs.
  const size_t payload = (s_mtu > 3 ? s_mtu : 23) - 3;
  const size_t length = strlen(line);

  for (size_t offset = 0; offset < length; offset += payload) {
    const size_t chunk = min(payload, length - offset);
    s_tx->setValue((const uint8_t*)(line + offset), chunk);
    s_tx->notify();
  }

  // The frame terminator. Without it the phone holds an incomplete line.
  const uint8_t newline = '\n';
  s_tx->setValue(&newline, 1);
  s_tx->notify();
}

bool BleTransport::receive(String& out) {
  if (s_rxTail == s_rxHead) return false;
  out = s_rxLines[s_rxTail];
  s_rxTail = (uint8_t)((s_rxTail + 1) % RX_SLOTS);
  return true;
}

bool BleTransport::isConnected() {
  return s_connected;
}

uint16_t BleTransport::mtu() const {
  return s_mtu;
}
