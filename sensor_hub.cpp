#include "sensor_hub.h"

#include <Wire.h>
#include <MAX30105.h>
#include <heartRate.h>
#include <spo2_algorithm.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

static MAX30105        s_ppg;
static Adafruit_MPU6050 s_imu;

static const float ACCEL_G     = 9.80665f;
static const float RAD_TO_DEGF = 57.29578f;

// Slow EMA so the baseline tracks skin drift but not a startle response.
static const float GSR_BASE_ALPHA = 0.002f;

/**
 * SpO2 window.
 *
 * Maxim's algorithm is written for 25 Hz and 100 samples, which is four
 * seconds of signal. The sensor runs at roughly 100 Hz, so every fourth
 * sample is kept. Windows do not overlap: the buffer fills, gets processed,
 * and starts again, which costs a little latency and saves running a heavy
 * routine on every sample.
 */
static const int32_t SPO2_SAMPLES = 100;
static const uint8_t SPO2_DECIMATE = 4;

static uint32_t s_irWindow[SPO2_SAMPLES];
static uint32_t s_redWindow[SPO2_SAMPLES];
static int32_t  s_spo2Index = 0;
static uint8_t  s_spo2Decimator = 0;

static bool i2cPing(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

void SensorHub::begin() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ_HZ);

  analogSetPinAttenuation(PIN_GSR, ADC_11db);  // full 0..~3.3 V swing
  analogReadResolution(12);

  ppg_.ok = initPpg();
  imu_.ok = initImu();

  // The GSR module is passive: a rail-pinned reading means nothing is wired.
  uint16_t probe = analogRead(PIN_GSR);
  gsr_.ok = (probe > 5 && probe < 4090);
  if (gsr_.ok) {
    gsrBaseline_     = probe;
    gsrBaselineInit_ = true;
    gsr_.raw  = probe;
    gsr_.base = probe;
  }
}

bool SensorHub::initPpg() {
  if (!i2cPing(ADDR_MAX30102)) return false;
  if (!s_ppg.begin(Wire, I2C_SPEED_FAST)) return false;
  // ledBrightness, sampleAverage, ledMode(2 = red+IR), sampleRate, pulseWidth, adcRange
  s_ppg.setup(0x1F, 4, 2, 400, 411, 4096);
  s_ppg.setPulseAmplitudeRed(0x0A);
  s_ppg.setPulseAmplitudeGreen(0);
  return true;
}

bool SensorHub::initImu() {
  if (!i2cPing(ADDR_MPU6050)) return false;
  if (!s_imu.begin(ADDR_MPU6050, &Wire)) return false;
  s_imu.setAccelerometerRange(MPU6050_RANGE_8_G);
  s_imu.setGyroRange(MPU6050_RANGE_500_DEG);
  s_imu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  return true;
}

void SensorHub::poll() {
  pollPpg();
  pollImu();
  pollGsr();
}

void SensorHub::pollPpg() {
  if (!ppg_.ok) return;

  // Read every loop: the beat detector needs the full sample stream.
  ppg_.ir  = s_ppg.getIR();
  ppg_.red = s_ppg.getRed();
  ppg_.finger = (ppg_.ir > PPG_FINGER_IR_MIN);

  if (!ppg_.finger) {
    ppg_.bpm    = 0.0f;
    ppg_.bpmAvg = 0;
    ratesFilled_ = false;
    rateIdx_     = 0;
    // A half-filled window taken before the finger lifted would produce a
    // confident-looking number from nothing.
    resetSpo2();
    return;
  }

  updateSpo2(ppg_.ir, ppg_.red);

  if (checkForBeat(ppg_.ir)) {
    uint32_t now   = millis();
    uint32_t delta = now - lastBeatMs_;
    lastBeatMs_    = now;
    if (delta > 0) {
      float bpm = 60000.0f / (float)delta;
      if (bpm > 20.0f && bpm < 255.0f) {
        ppg_.bpm = bpm;
        rates_[rateIdx_++] = (uint8_t)bpm;
        if (rateIdx_ >= 4) { rateIdx_ = 0; ratesFilled_ = true; }
        uint8_t n = ratesFilled_ ? 4 : rateIdx_;
        if (n > 0) {
          uint16_t sum = 0;
          for (uint8_t i = 0; i < n; i++) sum += rates_[i];
          ppg_.bpmAvg = sum / n;
        }
      }
    }
  }
}

void SensorHub::updateSpo2(uint32_t ir, uint32_t red) {
  if (++s_spo2Decimator < SPO2_DECIMATE) return;
  s_spo2Decimator = 0;

  s_irWindow[s_spo2Index] = ir;
  s_redWindow[s_spo2Index] = red;
  if (++s_spo2Index < SPO2_SAMPLES) return;
  s_spo2Index = 0;

  int32_t spo2 = 0;
  int8_t  spo2Valid = 0;
  int32_t heartRate = 0;   // ignored: the beat detector above is better
  int8_t  hrValid = 0;

  maxim_heart_rate_and_oxygen_saturation(
      s_irWindow, SPO2_SAMPLES, s_redWindow, &spo2, &spo2Valid, &heartRate, &hrValid);

  // The algorithm reports -999 alongside an invalid flag. Keep the flag as the
  // single source of truth and never surface the sentinel as a reading.
  ppg_.spo2Valid = (spo2Valid == 1) && spo2 > 0 && spo2 <= 100;
  ppg_.spo2 = ppg_.spo2Valid ? (int16_t)spo2 : -1;
}

void SensorHub::resetSpo2() {
  s_spo2Index = 0;
  s_spo2Decimator = 0;
  ppg_.spo2 = -1;
  ppg_.spo2Valid = false;
}

void SensorHub::pollImu() {
  if (!imu_.ok) return;
  uint32_t now = millis();
  if (now - lastImuMs_ < IMU_SAMPLE_MS) return;
  lastImuMs_ = now;

  sensors_event_t a, g, t;
  if (!s_imu.getEvent(&a, &g, &t)) return;

  imu_.ax = a.acceleration.x / ACCEL_G;
  imu_.ay = a.acceleration.y / ACCEL_G;
  imu_.az = a.acceleration.z / ACCEL_G;
  imu_.gx = g.gyro.x * RAD_TO_DEGF;
  imu_.gy = g.gyro.y * RAD_TO_DEGF;
  imu_.gz = g.gyro.z * RAD_TO_DEGF;
  imu_.mag = sqrtf(imu_.ax * imu_.ax + imu_.ay * imu_.ay + imu_.az * imu_.az);
  imu_.tempC = t.temperature;

  pedometer_.update(imu_.mag, now);
  steps_.count = pedometer_.count();
  steps_.cadence = pedometer_.cadence();
}

void SensorHub::pollGsr() {
  if (!gsr_.ok) return;
  uint32_t now = millis();
  if (now - lastGsrMs_ < GSR_SAMPLE_MS) return;
  lastGsrMs_ = now;

  gsr_.raw  = analogRead(PIN_GSR);
  gsr_.volt = analogReadMilliVolts(PIN_GSR) / 1000.0f;

  if (!gsrBaselineInit_) {
    gsrBaseline_     = gsr_.raw;
    gsrBaselineInit_ = true;
  } else {
    gsrBaseline_ += GSR_BASE_ALPHA * ((float)gsr_.raw - gsrBaseline_);
  }
  gsr_.base = (uint16_t)gsrBaseline_;
}

void SensorHub::watchdog() {
  uint32_t now = millis();
  if (now - lastWatchdogMs_ < SENSOR_WATCHDOG_MS) return;
  lastWatchdogMs_ = now;

  bool ppgSeen = i2cPing(ADDR_MAX30102);
  if (ppgSeen != ppg_.ok) {
    ppg_.ok = ppgSeen ? initPpg() : false;
    if (!ppg_.ok) { ppg_ = PpgData(); }
    resetSpo2();
    statusChanged_ = true;
  }

  bool imuSeen = i2cPing(ADDR_MPU6050);
  if (imuSeen != imu_.ok) {
    imu_.ok = imuSeen ? initImu() : false;
    if (!imu_.ok) { imu_ = ImuData(); }
    statusChanged_ = true;
  }

  uint16_t probe = analogRead(PIN_GSR);
  bool gsrSeen = (probe > 5 && probe < 4090);
  if (gsrSeen != gsr_.ok) {
    gsr_.ok = gsrSeen;
    if (gsrSeen) { gsrBaselineInit_ = false; }
    else         { gsr_ = GsrData(); }
    statusChanged_ = true;
  }
}

uint8_t SensorHub::onlineCount() const {
  return (ppg_.ok ? 1 : 0) + (imu_.ok ? 1 : 0) + (gsr_.ok ? 1 : 0);
}

bool SensorHub::takeStatusChanged() {
  bool c = statusChanged_;
  statusChanged_ = false;
  return c;
}

void SensorHub::calibrateGsr() {
  gsrBaselineInit_ = false;
}

void SensorHub::printStatusTable() const {
  Serial.printf("[I2C ] scan SDA=GPIO%u SCL=GPIO%u @%luk\n",
                PIN_I2C_SDA, PIN_I2C_SCL, (unsigned long)(I2C_FREQ_HZ / 1000));
  Serial.printf("[I2C ] 0x%02X MAX30102 .... %s\n", ADDR_MAX30102, ppg_.ok ? "ONLINE" : "OFFLINE");
  Serial.printf("[I2C ] 0x%02X MPU6050  .... %s\n", ADDR_MPU6050,  imu_.ok ? "ONLINE" : "OFFLINE");
  if (gsr_.ok) Serial.printf("[ADC ] GSR GPIO%u ....... ONLINE (baseline %u)\n", PIN_GSR, gsr_.base);
  else         Serial.printf("[ADC ] GSR GPIO%u ....... OFFLINE\n", PIN_GSR);
  Serial.printf("[STAT] %u/%u sensors online\n", onlineCount(), totalCount());
}
