# Dependencies — LACS node firmware

Everything this sketch needs, what installed it, and where it landed.
Recorded 2026-09-09 against a verified build for `esp32:esp32:XIAO_ESP32C3`
(683645 bytes flash / 52%, 24824 bytes RAM / 7%, with BLE enabled).

## Already on the machine — not installed by me

| Component | Version | Path |
|---|---|---|
| Arduino IDE 2 (bundles `arduino-cli`) | — | `%LOCALAPPDATA%\Programs\Arduino IDE` |
| esp32 board core | 3.3.7 | `%LOCALAPPDATA%\Arduino15\packages\esp32\hardware\esp32\3.3.7` |

`arduino-cli` is not on your PATH. It ships inside the IDE at:

```
C:\Users\moises\AppData\Local\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe
```

`Wire` and `SPI` come from the esp32 core, so they need no install.

## Installed into your sketchbook

All of these went to `C:\Users\moises\Documents\Arduino\libraries`.

### Requested directly

| Library | Version | Used for |
|---|---|---|
| SparkFun MAX3010x Pulse and Proximity Sensor Library | 1.1.2 | MAX30102 driver + `checkForBeat()` |
| Adafruit MPU6050 | 2.2.9 | MPU6050 accel/gyro/temp |
| ArduinoJson | 7.4.3 | building and parsing every JSON frame |
| NimBLE-Arduino | 2.5.1 | BLE GATT server for the phone bridge |

### Pulled in as dependencies

| Library | Version | Why | In the build? |
|---|---|---|---|
| Adafruit Unified Sensor | 1.1.15 | `sensors_event_t` type used by MPU6050 | yes |
| Adafruit BusIO | 1.17.4 | I2C/SPI abstraction under MPU6050 | yes |
| Adafruit GFX Library | 1.12.6 | only for the MPU6050 OLED examples | no |
| Adafruit SSD1306 | 2.5.17 | only for the MPU6050 OLED examples | no |

GFX and SSD1306 are declared dependencies of Adafruit MPU6050 but are not
compiled into this sketch. They are safe to leave in place; deleting them
would only be undone the next time you install an Adafruit library.

## Reproducing this setup

From Git Bash, on any machine that has Arduino IDE 2 and the esp32 core:

```bash
CLI="/c/Users/$USER/AppData/Local/Programs/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe"
export ARDUINO_DIRECTORIES_USER="$HOME/Documents/Arduino"
export ARDUINO_DIRECTORIES_DATA="$LOCALAPPDATA/Arduino15"

"$CLI" lib install \
  "SparkFun MAX3010x Pulse and Proximity Sensor Library@1.1.2" \
  "Adafruit MPU6050@2.2.9" \
  "ArduinoJson@7.4.3"
```

If the esp32 core is missing too, add the board manager URL and install it:

```bash
"$CLI" config init
"$CLI" config add board_manager.additional_urls \
  https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
"$CLI" core update-index
"$CLI" core install esp32:esp32@3.3.7
```

## Verify

```bash
"$CLI" compile --fqbn esp32:esp32:XIAO_ESP32C3 "C:\Users\moises\Documents\Arduino\lacs_node"
```

To flash, find the port first (`"$CLI" board list`) and add
`--upload --port COMx`.

## Version notes worth keeping

- **ArduinoJson 7.x** replaced `StaticJsonDocument`/`DynamicJsonDocument`
  with a single `JsonDocument`, and containers are created with
  `doc["k"].to<JsonObject>()`. `payload.cpp` uses the v7 API — it will not
  compile against 6.x without edits.
- **esp32 core 3.x** is the Arduino-ESP32 v3 line (IDF 5.x). `analogRead`,
  `analogReadMilliVolts` and `analogSetPinAttenuation` behave as used here;
  core 2.x had a different attenuation API.
- **SparkFun MAX3010x** targets the MAX30105 but is register-compatible with
  the MAX30102 — the `MAX30105` class is the correct one to instantiate.
