#pragma once
#include <Arduino.h>

// One link to the outside world. Everything above this interface deals in
// whole JSON lines, so swapping Serial for BLE or WiFi later touches only
// the concrete class below - no payload or command code changes.
class Transport {
 public:
  virtual ~Transport() {}
  virtual void begin() = 0;
  virtual void send(const char* line) = 0;      // one complete JSON frame
  virtual bool receive(String& out) = 0;        // one inbound line, if any
  virtual bool isConnected() = 0;
  virtual const char* name() const = 0;
};

// USB CDC serial. JSON frames go out as-is; human-readable lines are written
// separately by the sketch and always start with '[', so a host parser can
// keep only the lines that start with '{'.
class SerialTransport : public Transport {
 public:
  void begin() override;
  void send(const char* line) override;
  bool receive(String& out) override;
  bool isConnected() override;
  const char* name() const override { return "serial"; }

 private:
  String rx_;
};

// BLE lives in ble_transport.h - it needs NimBLE headers that do not belong
// in every translation unit that only wants the interface.

// Planned: WiFi client POSTing telemetry batches to the Express API and
// pulling commands, used when the node is in range instead of paired.
class WifiTransport : public Transport {
 public:
  void begin() override {}
  void send(const char*) override {}
  bool receive(String&) override { return false; }
  bool isConnected() override { return false; }
  const char* name() const override { return "wifi"; }
};
