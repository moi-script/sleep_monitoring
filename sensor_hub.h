#pragma once
#include <Arduino.h>
#include "config.h"

struct PpgData {
  bool     ok      = false;   // sensor answers on the bus
  bool     finger  = false;   // IR level says something is on the sensor
  uint32_t ir      = 0;
  uint32_t red     = 0;
  float    bpm     = 0.0f;    // instantaneous, from the last beat interval
  uint8_t  bpmAvg  = 0;       // rolling mean of the last 4 beats
};

struct ImuData {
  bool  ok    = false;
  float ax = 0, ay = 0, az = 0;   // g
  float gx = 0, gy = 0, gz = 0;   // deg/s
  float mag   = 0;                // |accel| in g
  float tempC = 0;
};

struct GsrData {
  bool     ok   = false;
  uint16_t raw  = 0;      // 0..4095
  float    volt = 0.0f;   // calibrated volts
  uint16_t base = 0;      // slow rolling baseline, for spike detection
};

// Owns every sensor: probing, initialisation, sampling and recovery.
class SensorHub {
 public:
  void begin();
  void poll();       // call as often as possible; internally rate-limited
  void watchdog();   // re-probe offline sensors, notice ones that vanished

  const PpgData& ppg() const { return ppg_; }
  const ImuData& imu() const { return imu_; }
  const GsrData& gsr() const { return gsr_; }

  uint8_t onlineCount() const;
  static uint8_t totalCount() { return 3; }

  // True once after any sensor changed online/offline state.
  bool takeStatusChanged();

  void printStatusTable() const;
  void calibrateGsr();

 private:
  bool initPpg();
  bool initImu();
  void pollPpg();
  void pollImu();
  void pollGsr();

  PpgData ppg_;
  ImuData imu_;
  GsrData gsr_;

  bool     statusChanged_ = false;
  uint32_t lastImuMs_     = 0;
  uint32_t lastGsrMs_     = 0;
  uint32_t lastWatchdogMs_ = 0;

  // Beat detection state
  uint32_t lastBeatMs_ = 0;
  uint8_t  rates_[4]   = {0, 0, 0, 0};
  uint8_t  rateIdx_    = 0;
  bool     ratesFilled_ = false;

  float gsrBaseline_ = 0.0f;
  bool  gsrBaselineInit_ = false;
};
