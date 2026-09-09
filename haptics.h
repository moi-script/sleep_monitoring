#pragma once
#include <Arduino.h>

enum HapticPattern : uint8_t {
  HAPTIC_IDLE = 0,
  HAPTIC_SINGLE,
  HAPTIC_DOUBLE,
  HAPTIC_LONG,
  HAPTIC_SOS,
};

// Non-blocking vibration motor driver: play() returns immediately, update()
// walks the on/off step list from loop().
class Haptics {
 public:
  void begin(uint8_t pin);
  void play(HapticPattern pattern, uint16_t stepMs = 0);
  void stop();
  void update();

  bool isOn() const { return on_; }
  HapticPattern pattern() const { return pattern_; }
  const char* patternName() const;

  static HapticPattern patternFromName(const char* name);

 private:
  void applyStep();

  uint8_t  pin_ = 255;
  bool     on_  = false;
  HapticPattern pattern_ = HAPTIC_IDLE;
  const uint16_t* steps_ = nullptr;  // alternating on,off durations
  uint8_t  stepCount_ = 0;
  uint8_t  stepIdx_   = 0;
  uint16_t stepOverrideMs_ = 0;
  uint32_t stepStartedMs_  = 0;
};
