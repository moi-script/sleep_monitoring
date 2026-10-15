// ---------------------------------------------------------------------------
// LACS sensor node - Seeed Studio XIAO ESP32-C3
//
//   MAX30102          SCL -> D5 (GPIO7)  SDA -> D4 (GPIO6)
//   MPU6050           SCL -> D5 (GPIO7)  SDA -> D4 (GPIO6)
//   Grove GSR         SIG -> D0 (GPIO2)
//   Grove vibration   SIG -> D1 (GPIO3)
//   Grove buzzer      SIG -> D2 (GPIO4)
//   Grove RGB LED     SIG -> D3 (GPIO5)
//
// Serial @115200 carries two interleaved streams on purpose:
//   lines starting with '[' are human-readable status/output
//   lines starting with '{' are the JSON payload for the fullstack app
// A host parser keeps the '{' lines and drops the rest. See README.md.
// ---------------------------------------------------------------------------
#include <Preferences.h>
#include <Adafruit_NeoPixel.h>
#include "config.h"
#include "sensor_hub.h"
#include "haptics.h"
#include "payload.h"
#include "transport.h"
#include "ble_transport.h"

static SensorHub       hub;
static Haptics         haptics;
static SerialTransport serialLink;
static BleTransport    bleLink;
static RuntimeConfig   cfg;
static Adafruit_NeoPixel statusLed(LED_COUNT, PIN_LED, NEO_GRB + NEO_KHZ800);

// Every frame goes out on both links. Serial stays live for bench debugging
// while the phone is connected over BLE, and a frame is never lost because
// the "wrong" transport was selected.
static Transport* const LINKS[] = { &serialLink, &bleLink };
static const uint8_t LINK_COUNT = sizeof(LINKS) / sizeof(LINKS[0]);

static void broadcast(const char* line) {
  for (uint8_t i = 0; i < LINK_COUNT; i++) LINKS[i]->send(line);
}

static char     deviceId[20] = {0};
// Must never repeat for this device, even across reboots: the server treats a
// {deviceId, seq} it already has as a duplicate and drops the frame. Starting
// from 0 on every boot silently lost everything after a restart until the
// counter passed the old high point. See initSeq().
static uint64_t seq = 0;
static char     frame[PAYLOAD_BUF_SIZE];

// Rule state
static bool     fingerWasPresent = false;
static uint32_t fingerLostAtMs   = 0;
static uint32_t lastFallMs       = 0;
static uint32_t lastSpikeMs      = 0;
static uint32_t lastNoFingerMs   = 0;
static const uint32_t RULE_COOLDOWN_MS = 5000;

// Scheduler
static uint32_t lastTelemetryMs = 0;
static uint32_t lastHumanMs     = 0;

// Set by an app buzz command, cleared when that pattern finishes.
static bool appAlarm = false;

// Active conditions, rebuilt every loop and attached to each telemetry frame.
static const char* activeFlags[3];
static uint8_t     activeFlagCount = 0;

// ---------------------------------------------------------------------------

static void makeDeviceId() {
  uint64_t mac = ESP.getEfuseMac();
  snprintf(deviceId, sizeof(deviceId), "lacs-%02x%02x%02x",
           (uint8_t)(mac >> 24), (uint8_t)(mac >> 32), (uint8_t)(mac >> 40));
}

static void sendStatusFrame() {
  buildStatus(frame, sizeof(frame), deviceId, ++seq, hub, cfg,
              bleLink.isConnected() ? "ble" : "serial");
  broadcast(frame);
}

static void sendEventFrame(const char* kind, float value) {
  buildEvent(frame, sizeof(frame), deviceId, ++seq, kind, value);
  broadcast(frame);
}

static void sendAckFrame(const char* cmdName, bool ok, const char* detail) {
  buildAck(frame, sizeof(frame), deviceId, ++seq, cmdName, ok, detail);
  broadcast(frame);
}

// ---------------------------------------------------------------------------

/**
 * Each boot gets its own block of a billion sequence numbers, from a boot
 * counter kept in flash. At the maximum 20 Hz that is over a year and a half
 * of uptime per block, and the largest possible value stays well inside the
 * range a JSON number (a JS double) holds exactly.
 */
static void initSeq() {
  Preferences prefs;
  prefs.begin("lacs", false);
  uint32_t boot = prefs.getUInt("boot", 0) + 1;
  prefs.putUInt("boot", boot);
  prefs.end();
  seq = (uint64_t)boot * 1000000000ULL;
}

void setup() {
  serialLink.begin();
  makeDeviceId();
  initSeq();

  Serial.println();
  Serial.printf("[BOOT] LACS node fw%s  id=%s\n", LACS_FW_VERSION, deviceId);

  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  statusLed.begin();
  statusLed.show();   // all off
  haptics.begin(PIN_MOTOR);
  hub.begin();
  hub.printStatusTable();

  // Confirms the motor is wired: one short pulse you can feel and see echoed
  // in the motor field of the first telemetry frames.
  haptics.play(HAPTIC_SINGLE);
  Serial.printf("[MOT ] VIB GPIO%u ....... test pulse\n", PIN_MOTOR);
  Serial.println("[HELP] type b=buzz s=status c=calibrate h=toggle text, or a JSON cmd");

  // Advertise under the device id so the phone's picker shows the same name
  // the Device tab asks for.
  bleLink.setDeviceName(deviceId);
  bleLink.begin();

  sendStatusFrame();
}

// ---------------------------------------------------------------------------

static void handleCommands() {
  String line;
  bool got = false;
  for (uint8_t i = 0; i < LINK_COUNT && !got; i++) got = LINKS[i]->receive(line);
  if (!got) return;

  Command c;
  if (!parseCommand(line, c)) return;

  switch (c.type) {
    case CMD_BUZZ:
      haptics.play(c.pattern, c.ms);
      appAlarm = haptics.pattern() != HAPTIC_IDLE;
      sendAckFrame(c.name, true, haptics.patternName());
      break;

    case CMD_CONFIG: {
      if (c.hasHz && c.hz >= 1 && c.hz <= 20) cfg.telemetryHz = c.hz;
      if (c.hasGsrDelta)                      cfg.gsrSpikeDelta = c.gsrDelta;
      sendAckFrame(c.name, true, nullptr);
      sendStatusFrame();
      break;
    }

    case CMD_STATUS:
      hub.printStatusTable();
      sendStatusFrame();
      break;

    case CMD_CALIBRATE:
      hub.calibrateGsr();
      sendAckFrame(c.name, true, "gsr baseline reset");
      break;

    case CMD_HUMAN:
      cfg.humanStream = c.humanToggle ? !cfg.humanStream : c.humanOn;
      sendAckFrame(c.name, true, cfg.humanStream ? "on" : "off");
      break;

    case CMD_UNKNOWN:
      sendAckFrame(c.name[0] ? c.name : "?", false, c.error);
      break;

    default:
      break;
  }
}

// Rebuilds activeFlags and fires an event + haptic pattern on each rising
// edge. The cooldown stops a single fall from producing a burst of frames.
static void evaluateRules() {
  uint32_t now = millis();
  activeFlagCount = 0;

  const ImuData& i = hub.imu();
  if (i.ok && (i.mag > cfg.fallHighG || i.mag < cfg.freeFallG)) {
    activeFlags[activeFlagCount++] = "fall";
    if (now - lastFallMs > RULE_COOLDOWN_MS) {
      lastFallMs = now;
      sendEventFrame("fall", i.mag);
      haptics.play(HAPTIC_SOS);
    }
  }

  const GsrData& g = hub.gsr();
  if (g.ok && g.raw > g.base && (g.raw - g.base) > cfg.gsrSpikeDelta) {
    activeFlags[activeFlagCount++] = "gsr_spike";
    if (now - lastSpikeMs > RULE_COOLDOWN_MS) {
      lastSpikeMs = now;
      sendEventFrame("gsr_spike", (float)(g.raw - g.base));
      haptics.play(HAPTIC_DOUBLE);
    }
  }

  const PpgData& p = hub.ppg();
  if (p.ok) {
    if (p.finger) {
      fingerWasPresent = true;
      fingerLostAtMs   = 0;
    } else if (fingerWasPresent) {
      if (fingerLostAtMs == 0) {
        fingerLostAtMs = now;
      } else if (now - fingerLostAtMs > cfg.fingerLostMs) {
        activeFlags[activeFlagCount++] = "no_finger";
        if (now - lastNoFingerMs > RULE_COOLDOWN_MS) {
          lastNoFingerMs = now;
          sendEventFrame("no_finger", (float)(now - fingerLostAtMs));
          haptics.play(HAPTIC_SINGLE);
          fingerWasPresent = false;   // announce once until it returns
        }
      }
    }
  }
}

// Priority: app alarm (blink red) > sensor fault (red) > app link (green,
// steady when connected, blinking when not). The buzzer follows the motor.
static void updateIndicators() {
  digitalWrite(PIN_BUZZER, haptics.isOn() ? HIGH : LOW);

  if (haptics.pattern() == HAPTIC_IDLE) appAlarm = false;
  bool blinkOn = (millis() / LED_BLINK_MS) % 2 == 0;
  bool allOk   = hub.ppg().ok && hub.imu().ok && hub.gsr().ok;

  uint8_t r = 0, g = 0;
  if (appAlarm)                    r = blinkOn ? LED_LEVEL : 0;
  else if (!allOk)                 r = LED_LEVEL;
  else if (bleLink.isConnected())  g = LED_LEVEL;
  else                             g = blinkOn ? LED_LEVEL : 0;

  static int32_t lastColor = -1;   // only write on change, each WS2812 write blocks briefly
  int32_t color = (r << 8) | g;
  if (color == lastColor) return;
  lastColor = color;
  statusLed.fill(statusLed.Color(r, g, 0));
  statusLed.show();
}

static void emitTelemetry() {
  uint32_t period = 1000UL / (cfg.telemetryHz ? cfg.telemetryHz : 1);
  uint32_t now = millis();
  if (now - lastTelemetryMs < period) return;
  lastTelemetryMs = now;

  buildTelemetry(frame, sizeof(frame), deviceId, ++seq, hub, haptics,
                 activeFlags, activeFlagCount);
  broadcast(frame);
}

static void emitHumanLine() {
  if (!cfg.humanStream) return;
  uint32_t now = millis();
  if (now - lastHumanMs < HUMAN_LINE_MS) return;
  lastHumanMs = now;

  const PpgData& p = hub.ppg();
  const ImuData& i = hub.imu();
  const GsrData& g = hub.gsr();

  Serial.print("[DATA] ppg ");
  if (p.ok) {
    Serial.printf("ir=%lu bpm=%u finger=%c", (unsigned long)p.ir,
                  p.bpmAvg, p.finger ? 'Y' : 'N');
    if (p.spo2Valid) Serial.printf(" spo2=%d%%", p.spo2);
  } else {
    Serial.print("--offline--");
  }

  Serial.print(" | imu ");
  if (i.ok) Serial.printf("a=%.2fg ax=%.2f ay=%.2f az=%.2f %.1fC",
                          i.mag, i.ax, i.ay, i.az, i.tempC);
  else      Serial.print("--offline--");

  Serial.print(" | gsr ");
  if (g.ok) Serial.printf("%u base=%u %.2fV", g.raw, g.base, g.volt);
  else      Serial.print("--offline--");

  Serial.printf(" | steps %lu", (unsigned long)hub.steps().count);
  if (hub.steps().cadence > 0) Serial.printf(" @%u/min", hub.steps().cadence);

  Serial.printf(" | mot %s", haptics.patternName());

  if (activeFlagCount > 0) {
    Serial.print(" | !");
    for (uint8_t k = 0; k < activeFlagCount; k++) {
      Serial.print(activeFlags[k]);
      if (k + 1 < activeFlagCount) Serial.print(',');
    }
  }
  Serial.println();
}

void loop() {
  hub.poll();          // PPG every pass, IMU and GSR rate-limited inside
  haptics.update();
  handleCommands();
  evaluateRules();

  hub.watchdog();
  if (hub.takeStatusChanged()) {
    Serial.println("[STAT] sensor set changed");
    hub.printStatusTable();
    sendStatusFrame();
  }

  updateIndicators();
  emitTelemetry();
  emitHumanLine();
}
