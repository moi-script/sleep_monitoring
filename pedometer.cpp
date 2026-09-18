#include "pedometer.h"

// Smoothing. Low enough to kill sensor noise, high enough to keep the shape
// of a footfall at 2 Hz.
static const float FILTER_ALPHA = 0.30f;

// A step must swing at least this far. Below it the signal is somebody
// breathing or the board settling, not walking.
static const float MIN_SWING_G = 0.12f;

// Fastest believable step rate is about 4 per second. Anything closer
// together is one footfall being counted twice.
static const uint32_t MIN_STEP_INTERVAL_MS = 250;

// A step slower than this is not part of a walk, so cadence restarts.
static const uint32_t MAX_STEP_INTERVAL_MS = 2000;

// The peak and trough drift back toward the current signal, so the threshold
// keeps up when someone slows down or the board shifts on the body.
static const uint32_t DECAY_INTERVAL_MS = 200;
static const float DECAY_RATE = 0.10f;

void Pedometer::update(float magnitude, uint32_t nowMs) {
  if (!primed_) {
    filtered_ = magnitude;
    peak_ = magnitude;
    trough_ = magnitude;
    lastDecayMs_ = nowMs;
    primed_ = true;
    return;
  }

  filtered_ += FILTER_ALPHA * (magnitude - filtered_);

  if (filtered_ > peak_) peak_ = filtered_;
  if (filtered_ < trough_) trough_ = filtered_;

  if (nowMs - lastDecayMs_ >= DECAY_INTERVAL_MS) {
    lastDecayMs_ = nowMs;
    peak_ -= (peak_ - filtered_) * DECAY_RATE;
    trough_ += (filtered_ - trough_) * DECAY_RATE;
  }

  const float swing = peak_ - trough_;
  const float threshold = (peak_ + trough_) * 0.5f;

  if (swing < MIN_SWING_G) {
    aboveThreshold_ = filtered_ > threshold;
    return;
  }

  // Count on the falling edge only. Counting both edges doubles every step.
  const bool above = filtered_ > threshold;
  if (aboveThreshold_ && !above) {
    const uint32_t sinceLast = nowMs - lastStepMs_;
    if (lastStepMs_ == 0 || sinceLast >= MIN_STEP_INTERVAL_MS) {
      count_++;
      if (lastStepMs_ != 0 && sinceLast <= MAX_STEP_INTERVAL_MS) {
        intervals_[intervalIdx_] = sinceLast;
        intervalIdx_ = (uint8_t)((intervalIdx_ + 1) % RECENT);
        if (intervalCount_ < RECENT) intervalCount_++;
      } else {
        // A long gap means the previous walk ended; do not average across it.
        intervalCount_ = 0;
        intervalIdx_ = 0;
      }
      lastStepMs_ = nowMs;
    }
  }
  aboveThreshold_ = above;
}

uint16_t Pedometer::cadence() const {
  if (intervalCount_ == 0) return 0;
  if (millis() - lastStepMs_ > MAX_STEP_INTERVAL_MS) return 0;

  uint32_t sum = 0;
  for (uint8_t i = 0; i < intervalCount_; i++) sum += intervals_[i];
  const uint32_t mean = sum / intervalCount_;
  if (mean == 0) return 0;
  return (uint16_t)(60000UL / mean);
}

void Pedometer::reset() {
  count_ = 0;
  lastStepMs_ = 0;
  intervalCount_ = 0;
  intervalIdx_ = 0;
}
