#pragma once
#include <Arduino.h>
#include "config.h"
#include "sensor_hub.h"
#include "haptics.h"

// Every outbound frame fits comfortably in this.
static const size_t PAYLOAD_BUF_SIZE = 512;

enum CommandType : uint8_t {
  CMD_NONE = 0,   // nothing parsed
  CMD_BUZZ,
  CMD_CONFIG,
  CMD_STATUS,
  CMD_CALIBRATE,
  CMD_HUMAN,
  CMD_UNKNOWN,    // well-formed line, unrecognised "cmd"
};

struct Command {
  CommandType   type    = CMD_NONE;
  HapticPattern pattern = HAPTIC_SINGLE;
  uint16_t      ms      = 0;

  bool     hasHz        = false;  uint8_t  hz        = 0;
  bool     hasGsrDelta  = false;  uint16_t gsrDelta  = 0;
  bool     humanOn      = true;
  bool     humanToggle  = false;  // "h" shortcut flips the current setting

  char name[16] = {0};            // echoed back in the ack
  const char* error = nullptr;    // set when the line could not be used
};

// Parses one inbound line. Accepts JSON ({"cmd":"buzz","pattern":"double"})
// and the single-key shortcuts b/s/c/h for hand-typing in the Serial Monitor.
bool parseCommand(const String& line, Command& out);

// Frame builders. Each writes one complete single-line JSON document.
void buildTelemetry(char* buf, size_t len, const char* id, uint64_t seq,
                    const SensorHub& hub, const Haptics& hap,
                    const char* const* flags, uint8_t flagCount);

void buildStatus(char* buf, size_t len, const char* id, uint64_t seq,
                 const SensorHub& hub, const RuntimeConfig& cfg,
                 const char* transportName);

void buildEvent(char* buf, size_t len, const char* id, uint64_t seq,
                const char* kind, float value);

void buildAck(char* buf, size_t len, const char* id, uint64_t seq,
              const char* cmdName, bool ok, const char* detail);
