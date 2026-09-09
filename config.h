#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------------
// LACS sensor node - XIAO ESP32-C3
// ---------------------------------------------------------------------------
#define LACS_FW_VERSION       "0.1.0"
#define LACS_PAYLOAD_VERSION  1

// Silkscreen -> GPIO map for the Seeed XIAO ESP32-C3.
// D4 = GPIO6, D5 = GPIO7, D0 = GPIO2, D1 = GPIO3.
static const uint8_t PIN_I2C_SDA = 6;   // D4 -> MAX30102 SDA + MPU6050 SDA
static const uint8_t PIN_I2C_SCL = 7;   // D5 -> MAX30102 SCL + MPU6050 SCL
static const uint8_t PIN_GSR     = 2;   // D0 -> Grove GSR SIG (ADC1_CH2, strapping pin)
static const uint8_t PIN_MOTOR   = 3;   // D1 -> Grove vibration motor SIG

static const uint32_t I2C_FREQ_HZ  = 400000;
static const uint8_t  ADDR_MAX30102 = 0x57;
static const uint8_t  ADDR_MPU6050  = 0x68;

// Scheduler intervals (ms)
static const uint32_t IMU_SAMPLE_MS      = 20;    // 50 Hz
static const uint32_t GSR_SAMPLE_MS      = 20;    // 50 Hz
static const uint32_t HUMAN_LINE_MS      = 1000;  // readable [DATA] line
static const uint32_t SENSOR_WATCHDOG_MS = 5000;  // re-probe missing sensors

// Finger detection on the MAX30102 IR channel
static const uint32_t PPG_FINGER_IR_MIN = 50000;

// Runtime-tunable settings, changed by the {"cmd":"config"} frame.
struct RuntimeConfig {
  uint8_t  telemetryHz   = 5;     // JSON telemetry frames per second (1..20)
  uint16_t gsrSpikeDelta = 250;   // raw counts above rolling baseline = spike
  float    fallHighG     = 2.5f;  // |accel| above this = impact
  float    freeFallG     = 0.30f; // |accel| below this = free fall
  uint32_t fingerLostMs  = 3000;  // finger absent this long after being present
  bool     humanStream   = true;  // print the [..] human-readable lines
};
