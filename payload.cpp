#include "payload.h"
#include <ArduinoJson.h>

// Keep frames small: the app does not need 9 significant digits of noise.
static float r2(float v) { return roundf(v * 100.0f) / 100.0f; }
static float r3(float v) { return roundf(v * 1000.0f) / 1000.0f; }

static void addEnvelope(JsonDocument& doc, const char* type,
                        const char* id, uint32_t seq) {
  doc["v"]   = LACS_PAYLOAD_VERSION;
  doc["t"]   = type;
  doc["id"]  = id;
  doc["seq"] = seq;
  doc["ms"]  = millis();
}

void buildTelemetry(char* buf, size_t len, const char* id, uint32_t seq,
                    const SensorHub& hub, const Haptics& hap,
                    const char* const* flags, uint8_t flagCount) {
  JsonDocument doc;
  addEnvelope(doc, "telemetry", id, seq);

  const PpgData& p = hub.ppg();
  JsonObject ppg = doc["ppg"].to<JsonObject>();
  ppg["ok"]     = p.ok;
  ppg["finger"] = p.finger;
  ppg["ir"]     = p.ir;
  ppg["red"]    = p.red;
  ppg["bpm"]    = r2(p.bpm);
  ppg["bpmAvg"] = p.bpmAvg;

  const ImuData& i = hub.imu();
  JsonObject imu = doc["imu"].to<JsonObject>();
  imu["ok"]    = i.ok;
  imu["ax"]    = r3(i.ax);
  imu["ay"]    = r3(i.ay);
  imu["az"]    = r3(i.az);
  imu["gx"]    = r2(i.gx);
  imu["gy"]    = r2(i.gy);
  imu["gz"]    = r2(i.gz);
  imu["mag"]   = r3(i.mag);
  imu["tempC"] = r2(i.tempC);

  const GsrData& g = hub.gsr();
  JsonObject gsr = doc["gsr"].to<JsonObject>();
  gsr["ok"]   = g.ok;
  gsr["raw"]  = g.raw;
  gsr["volt"] = r3(g.volt);
  gsr["base"] = g.base;

  JsonObject motor = doc["motor"].to<JsonObject>();
  motor["on"]      = hap.isOn();
  motor["pattern"] = hap.patternName();

  JsonArray arr = doc["flags"].to<JsonArray>();
  for (uint8_t k = 0; k < flagCount; k++) arr.add(flags[k]);

  serializeJson(doc, buf, len);
}

void buildStatus(char* buf, size_t len, const char* id, uint32_t seq,
                 const SensorHub& hub, const RuntimeConfig& cfg,
                 const char* transportName) {
  JsonDocument doc;
  addEnvelope(doc, "status", id, seq);
  doc["fw"]        = LACS_FW_VERSION;
  doc["transport"] = transportName;
  doc["online"]    = hub.onlineCount();
  doc["total"]     = SensorHub::totalCount();

  JsonObject s = doc["sensors"].to<JsonObject>();
  s["max30102"] = hub.ppg().ok;
  s["mpu6050"]  = hub.imu().ok;
  s["gsr"]      = hub.gsr().ok;

  JsonObject c = doc["config"].to<JsonObject>();
  c["hz"]           = cfg.telemetryHz;
  c["gsrDelta"]     = cfg.gsrSpikeDelta;
  c["fallHighG"]    = r2(cfg.fallHighG);
  c["freeFallG"]    = r2(cfg.freeFallG);
  c["fingerLostMs"] = cfg.fingerLostMs;
  c["human"]        = cfg.humanStream;

  serializeJson(doc, buf, len);
}

void buildEvent(char* buf, size_t len, const char* id, uint32_t seq,
                const char* kind, float value) {
  JsonDocument doc;
  addEnvelope(doc, "event", id, seq);
  doc["kind"]  = kind;
  doc["value"] = r2(value);
  serializeJson(doc, buf, len);
}

void buildAck(char* buf, size_t len, const char* id, uint32_t seq,
              const char* cmdName, bool ok, const char* detail) {
  JsonDocument doc;
  addEnvelope(doc, "ack", id, seq);
  doc["cmd"] = cmdName;
  doc["ok"]  = ok;
  if (detail != nullptr) doc["detail"] = detail;
  serializeJson(doc, buf, len);
}

static void setName(Command& c, const char* n) {
  strncpy(c.name, n, sizeof(c.name) - 1);
  c.name[sizeof(c.name) - 1] = '\0';
}

bool parseCommand(const String& line, Command& out) {
  out = Command();
  if (line.length() == 0) return false;

  // Single-key shortcuts for typing straight into the Serial Monitor.
  if (line.length() == 1) {
    switch (line[0]) {
      case 'b': out.type = CMD_BUZZ;      setName(out, "buzz");      return true;
      case 's': out.type = CMD_STATUS;    setName(out, "status");    return true;
      case 'c': out.type = CMD_CALIBRATE; setName(out, "calibrate"); return true;
      case 'h': out.type = CMD_HUMAN;     out.humanToggle = true;
                setName(out, "human");     return true;
      default: break;
    }
  }

  if (line[0] != '{') { out.type = CMD_UNKNOWN; out.error = "not json"; return true; }

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, line);
  if (err) { out.type = CMD_UNKNOWN; out.error = "bad json"; return true; }

  const char* cmd = doc["cmd"] | "";
  setName(out, cmd);

  if (strcmp(cmd, "buzz") == 0) {
    out.type    = CMD_BUZZ;
    out.pattern = Haptics::patternFromName(doc["pattern"] | (const char*)nullptr);
    out.ms      = doc["ms"] | 0;
    return true;
  }
  if (strcmp(cmd, "config") == 0) {
    out.type = CMD_CONFIG;
    if (doc["hz"].is<uint8_t>() || doc["hz"].is<int>()) {
      out.hasHz = true;  out.hz = doc["hz"].as<uint8_t>();
    }
    if (doc["gsrThresh"].is<int>()) {
      out.hasGsrDelta = true;  out.gsrDelta = doc["gsrThresh"].as<uint16_t>();
    }
    return true;
  }
  if (strcmp(cmd, "status") == 0)    { out.type = CMD_STATUS;    return true; }
  if (strcmp(cmd, "calibrate") == 0) { out.type = CMD_CALIBRATE; return true; }
  if (strcmp(cmd, "human") == 0) {
    out.type    = CMD_HUMAN;
    out.humanOn = doc["on"] | true;
    return true;
  }

  out.type  = CMD_UNKNOWN;
  out.error = "unknown cmd";
  return true;
}
