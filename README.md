# LACS node — XIAO ESP32-C3 firmware

Sensor node firmware. Reads MAX30102 (PPG), MPU6050 (IMU) and a Grove GSR
sensor, drives a Grove vibration motor, and emits a versioned JSON payload
that the fullstack app consumes.

## Wiring

| Peripheral            | Silkscreen | GPIO   |
|-----------------------|------------|--------|
| MAX30102 / MPU6050 SDA| D4         | GPIO6  |
| MAX30102 / MPU6050 SCL| D5         | GPIO7  |
| Grove GSR SIG         | D0         | GPIO2  |
| Grove vibration SIG   | D1         | GPIO3  |

Both I2C devices share the bus (0x57 and 0x68), 400 kHz.

> **GPIO2 is a boot strapping pin on the ESP32-C3.** It must not be held low
> at power-up. The Grove GSR output idles mid-rail so this works in practice,
> but if the board ever refuses to boot with the GSR attached, move it to
> D2 (GPIO4) and change `PIN_GSR` in `config.h`.

## Build

Board: **XIAO_ESP32C3** (esp32 core 3.x). Libraries, all installed via
Library Manager:

- SparkFun MAX3010x Pulse and Proximity Sensor Library
- Adafruit MPU6050 (pulls Adafruit Unified Sensor + BusIO)
- ArduinoJson 7.x
- NimBLE-Arduino 2.x

See `DEPENDENCIES.md` for exact versions and a one-line install command.

## Serial protocol

One port, 115200 baud, two interleaved streams:

- lines starting with `[` — human-readable status and live output
- lines starting with `{` — JSON frames, one per line

**A host parser keeps only lines that start with `{` and drops the rest.**
Send `h` (or `{"cmd":"human","on":false}`) to silence the human stream once
the app is driving the port.

Boot output:

```
[BOOT] LACS node fw0.1.0  id=lacs-7a3f21
[I2C ] scan SDA=GPIO6 SCL=GPIO7 @400k
[I2C ] 0x57 MAX30102 .... ONLINE
[I2C ] 0x68 MPU6050  .... ONLINE
[ADC ] GSR GPIO2 ....... ONLINE (baseline 1820)
[STAT] 3/3 sensors online
[MOT ] VIB GPIO3 ....... test pulse
[BLE ] advertising as lacs-7a3f21
[DATA] ppg ir=98421 bpm=72 finger=Y | imu a=1.01g ax=0.01 ... | gsr 1820 base=1800 1.47V | mot idle
```

Sensors are re-probed every 5 s. Unplugging a Grove cable prints `OFFLINE`,
sets that sensor's `ok` to `false`, and emits a fresh `status` frame;
replugging re-initialises it automatically.

## Payload contract (`v: 1`)

Every frame carries the same envelope: `v`, `t`, `id`, `seq`, `ms`.
`id` is derived from the eFuse MAC and is stable per board. `seq` increments
across all frame types, so the app can detect drops. `ms` is uptime, not wall
clock — the server timestamps on arrival.

### `telemetry` — default 5 Hz

```json
{"v":1,"t":"telemetry","id":"lacs-7a3f21","seq":142,"ms":28450,
 "ppg":{"ok":true,"finger":true,"ir":98421,"red":87233,"bpm":72.4,"bpmAvg":71},
 "imu":{"ok":true,"ax":0.01,"ay":-0.02,"az":1.0,"gx":0.4,"gy":-1.2,"gz":0.1,"mag":1.01,"tempC":31.2},
 "gsr":{"ok":true,"raw":1820,"volt":1.47,"base":1800},
 "motor":{"on":false,"pattern":"idle"},
 "flags":[]}
```

| Field | Meaning |
|---|---|
| `ppg.ok` / `imu.ok` / `gsr.ok` | sensor is present on the bus right now |
| `ppg.finger` | IR level above 50000, i.e. skin on the sensor |
| `ppg.bpm` / `bpmAvg` | instantaneous beat interval / mean of last 4 beats |
| `imu.ax..az` | g · `gx..gz` deg/s · `mag` = ‖accel‖ in g |
| `gsr.raw` | 0–4095 · `base` slow rolling baseline for spike detection |
| `motor.pattern` | `idle` `single` `double` `long` `sos` |
| `flags` | conditions active this instant: `fall`, `gsr_spike`, `no_finger` |

SpO2 is deliberately not computed on-device — derive it server-side from
`ir`/`red` where you have real compute.

### `status` — on boot, on sensor change, on request

```json
{"v":1,"t":"status","id":"lacs-7a3f21","seq":1,"ms":812,"fw":"0.1.0",
 "transport":"serial","online":3,"total":3,
 "sensors":{"max30102":true,"mpu6050":true,"gsr":true},
 "config":{"hz":5,"gsrDelta":250,"fallHighG":2.5,"freeFallG":0.3,"fingerLostMs":3000,"human":true}}
```

### `event` — rising edge of a rule, 5 s cooldown each

```json
{"v":1,"t":"event","id":"lacs-7a3f21","seq":143,"ms":28460,"kind":"fall","value":3.12}
```

`kind` is `fall` (‖a‖ > 2.5 g or < 0.3 g), `gsr_spike` (raw exceeds baseline
by `gsrDelta`) or `no_finger` (finger absent 3 s after being present). Each
also fires a haptic pattern locally, so the node is useful with no app
attached.

### `ack` — reply to every inbound command

```json
{"v":1,"t":"ack","id":"lacs-7a3f21","seq":144,"ms":30100,"cmd":"buzz","ok":true,"detail":"double"}
```

## Commands (device ← app)

One JSON object per line. Same parser regardless of transport.

| Command | Effect |
|---|---|
| `{"cmd":"buzz","pattern":"double","ms":300}` | play a pattern; `ms` stretches the on-steps, `pattern` ∈ single/double/long/sos |
| `{"cmd":"config","hz":10,"gsrThresh":300}` | telemetry rate 1–20 Hz, GSR spike delta |
| `{"cmd":"status"}` | force a `status` frame |
| `{"cmd":"calibrate"}` | reset the GSR baseline to the current reading |
| `{"cmd":"human","on":false}` | mute/unmute the `[..]` text stream |

Single-key shortcuts for the Serial Monitor: `b` buzz, `s` status,
`c` calibrate, `h` toggle text.

## Integration path

`transport.h` defines a `Transport` interface — `begin/send/receive/
isConnected/name`. `SerialTransport` is implemented; `BleTransport` and
`WifiTransport` are compiling stubs that report themselves disconnected.
Nothing above that interface knows which link it is talking to, so adding
BLE means implementing one class and pointing `activeLink` at it — the frame
builders and command parser stay untouched.

## Files

| File | Purpose |
|---|---|
| `lacs_node.ino` | scheduler, rule evaluation, command dispatch |
| `config.h` | pins, intervals, thresholds, `RuntimeConfig` |
| `sensor_hub.h/.cpp` | probe, init, sample, watchdog recovery |
| `payload.h/.cpp` | frame builders and command parser |
| `transport.h/.cpp` | link abstraction, Serial implementation |
| `ble_transport.h/.cpp` | NimBLE GATT server, chunked frame notifications |
| `haptics.h/.cpp` | non-blocking motor patterns |

There is no `delay()` anywhere; everything is a `millis()` state machine so
the PPG beat detector keeps its sample stream.
# sleep_monitoring
