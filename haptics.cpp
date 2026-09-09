#include "haptics.h"

// Alternating on/off durations in ms, starting with an "on" step.
static const uint16_t STEPS_SINGLE[] = {200, 0};
static const uint16_t STEPS_DOUBLE[] = {120, 100, 120, 0};
static const uint16_t STEPS_LONG[]   = {800, 0};
static const uint16_t STEPS_SOS[]    = {120, 100, 120, 100, 120, 250,
                                        350, 150, 350, 150, 350, 250,
                                        120, 100, 120, 100, 120, 0};

void Haptics::begin(uint8_t pin) {
  pin_ = pin;
  pinMode(pin_, OUTPUT);
  digitalWrite(pin_, LOW);
}

void Haptics::play(HapticPattern pattern, uint16_t stepMs) {
  switch (pattern) {
    case HAPTIC_SINGLE: steps_ = STEPS_SINGLE; stepCount_ = 2;  break;
    case HAPTIC_DOUBLE: steps_ = STEPS_DOUBLE; stepCount_ = 4;  break;
    case HAPTIC_LONG:   steps_ = STEPS_LONG;   stepCount_ = 2;  break;
    case HAPTIC_SOS:    steps_ = STEPS_SOS;    stepCount_ = 18; break;
    default: stop(); return;
  }
  pattern_        = pattern;
  stepIdx_        = 0;
  stepOverrideMs_ = stepMs;
  stepStartedMs_  = millis();
  applyStep();
}

void Haptics::stop() {
  if (pin_ != 255) digitalWrite(pin_, LOW);
  on_       = false;
  pattern_  = HAPTIC_IDLE;
  steps_    = nullptr;
  stepCount_ = 0;
  stepIdx_   = 0;
}

// Even step indices are "on", odd are "off". A caller-supplied ms only
// stretches the on-steps, so {"cmd":"buzz","ms":500} keeps the pattern shape.
void Haptics::applyStep() {
  bool wantOn = (stepIdx_ % 2) == 0;
  on_ = wantOn;
  digitalWrite(pin_, wantOn ? HIGH : LOW);
}

void Haptics::update() {
  if (steps_ == nullptr) return;

  uint16_t dur = steps_[stepIdx_];
  if (dur == 0) { stop(); return; }
  if ((stepIdx_ % 2) == 0 && stepOverrideMs_ > 0) dur = stepOverrideMs_;

  if (millis() - stepStartedMs_ < dur) return;

  stepIdx_++;
  stepStartedMs_ = millis();
  if (stepIdx_ >= stepCount_ || steps_[stepIdx_] == 0) { stop(); return; }
  applyStep();
}

const char* Haptics::patternName() const {
  switch (pattern_) {
    case HAPTIC_SINGLE: return "single";
    case HAPTIC_DOUBLE: return "double";
    case HAPTIC_LONG:   return "long";
    case HAPTIC_SOS:    return "sos";
    default:            return "idle";
  }
}

HapticPattern Haptics::patternFromName(const char* name) {
  if (name == nullptr)                return HAPTIC_SINGLE;
  if (strcmp(name, "double") == 0)    return HAPTIC_DOUBLE;
  if (strcmp(name, "long")   == 0)    return HAPTIC_LONG;
  if (strcmp(name, "sos")    == 0)    return HAPTIC_SOS;
  return HAPTIC_SINGLE;
}
