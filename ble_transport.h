#pragma once
#include <Arduino.h>
#include "transport.h"

/**
 * BLE GATT link to the phone bridge.
 *
 * Uses the Nordic UART Service UUIDs. They are not a standard in any formal
 * sense, but every BLE library and scanner app already knows them, which
 * makes the phone side and any off-the-shelf debugging tool work without
 * custom configuration.
 *
 *   service  6e400001-b5a3-f393-e0a9-e50e24dcca9e
 *   tx       6e400003-...  notify   device -> phone, JSON frames
 *   rx       6e400002-...  write    phone -> device, JSON commands
 *
 * The wire format is identical to Serial: one JSON document per line. Nothing
 * above the Transport interface knows which link delivered it.
 */
class BleTransport : public Transport {
 public:
  void begin() override;
  void send(const char* line) override;
  bool receive(String& out) override;
  bool isConnected() override;
  const char* name() const override { return "ble"; }

  /** Advertised name; also what the phone shows in its scan list. */
  void setDeviceName(const char* deviceId);

  /** Negotiated MTU, or 0 when nothing is connected. Diagnostics only. */
  uint16_t mtu() const;

 private:
  char deviceName_[24] = "lacs-node";
  bool started_ = false;
};
