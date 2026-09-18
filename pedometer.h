#pragma once
#include <Arduino.h>

/**
 * Counts steps from the accelerometer magnitude.
 *
 * Walking shows up as a roughly 1.5 to 2.5 Hz oscillation around 1 g. The
 * counter low-pass filters the signal, tracks a threshold that sits between
 * the recent peak and trough, and counts a step each time the signal falls
 * through that threshold. A dynamic threshold matters because the size of the
 * swing depends on where the board is worn and how fast the person moves; a
 * fixed one either misses slow walking or counts hand waving.
 *
 * The board has no clock, so the count is steps since power-on. The server
 * turns that into a daily figure by taking differences.
 */
class Pedometer {
 public:
  void update(float magnitude, uint32_t nowMs);
  void reset();

  uint32_t count() const { return count_; }
  /** Steps per minute over the last few steps. 0 when not walking. */
  uint16_t cadence() const;

 private:
  static const uint8_t RECENT = 4;

  bool  primed_ = false;
  float filtered_ = 1.0f;
  float peak_ = 1.0f;
  float trough_ = 1.0f;
  bool  aboveThreshold_ = false;

  uint32_t count_ = 0;
  uint32_t lastStepMs_ = 0;
  uint32_t intervals_[RECENT] = {0, 0, 0, 0};
  uint8_t  intervalIdx_ = 0;
  uint8_t  intervalCount_ = 0;
  uint32_t lastDecayMs_ = 0;
};
