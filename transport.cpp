#include "transport.h"

void SerialTransport::begin() {
  Serial.begin(115200);
  uint32_t start = millis();
  while (!Serial && (millis() - start) < 2000) { /* native USB enumerating */ }
  rx_.reserve(192);
}

void SerialTransport::send(const char* line) {
  Serial.println(line);
}

// Accumulates one line without blocking. Also passes through bare single-key
// shortcuts so the Arduino Serial Monitor stays usable by hand.
bool SerialTransport::receive(String& out) {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (rx_.length() == 0) continue;
      out = rx_;
      rx_ = "";
      out.trim();
      return out.length() > 0;
    }
    if (rx_.length() < 180) rx_ += c;
  }
  return false;
}

bool SerialTransport::isConnected() {
  return true;  // USB CDC: assume the host may be listening
}
