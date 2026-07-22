# baja-can

ESP32 **TWAI** (built-in CAN 2.0) driver wrapper for NU Baja SAE, shared across
all boards. Wraps the ESP-IDF `twai` peripheral with send/recv helpers, typed
messages, and — the main upgrade over the original `baja_can` library —
**self-healing bus recovery** plus health diagnostics.

> Hardware note: this uses the ESP32's on-chip TWAI controller (needs only a CAN
> *transceiver* like the SN65HVD230), **not** an external MCP2515 SPI controller.

## What's improved vs. the original

| Area | Original `baja_can` | This library |
|---|---|---|
| Bus-off recovery | Manual: poll `readAlerts()`, call `recover()` yourself | `poll()` auto-recovers as a non-blocking state machine |
| Diagnostics | `getState()` only | `getHealth()` → `BusHealth` (error counters, bus-off/recovery totals, queue depths) |
| Config | Hardcoded 500 kbit, rx queue 50 | `CanConfig`: bitrate, TX/RX queue depths, auto-recover toggle, settle delay |
| TX while bus-off | Frame silently queued | `writeFrame()` returns `ESP_ERR_INVALID_STATE` while recovering |

The TWAI peripheral does **not** auto-recover from bus-off on its own — firmware
must run the recovery sequence. `poll()` does that for you.

## Quick start

```cpp
#include "BajaCan.h"

BajaCan can(GPIO_NUM_5, GPIO_NUM_4);   // (tx, rx)

void setup() {
    CanConfig cfg;                      // defaults: 500 kbit, auto-recover on
    can.begin(cfg);
}

void loop() {
    can.poll();                         // <-- call every loop: auto-recovers bus-off

    CanMessage msg;
    if (can.readMessage(msg, 5) == ESP_OK && msg.getDataType() == FLOAT) {
        float rpm = msg.getFloat();
        (void)rpm;
    }

    CanMessage out(0x441, 3600.0f);     // ENGINE_RPM
    can.writeMessage(out, 5);
}
```

## Checking bus health

```cpp
BusHealth h;
if (can.getHealth(h) == ESP_OK) {
    // h.state, h.txErrCounter, h.busOffEvents, h.recoveries, ...
}
if (can.isBusOff()) { /* degraded — poll() is working on it */ }
```

## API

### `CanConfig`
- `bitrate` — `CanBitrate::{BR_125K,BR_250K,BR_500K,BR_1M}` (default `BR_500K`)
- `rxQueueLen` / `txQueueLen` — driver queue depths (default 50 / 20)
- `autoRecover` — `poll()` auto-recovers from bus-off (default `true`)
- `recoverDelayMs` — settle time before restarting after bus-off (default 100)

### `BajaCan`
- `begin(const CanConfig& = {}, uint32_t acceptanceCode = 0)` / `end()`
- `poll(uint32_t timeoutMs = 0)` — **call periodically**; drains alerts + drives recovery
- `readMessage(CanMessage&, timeoutMs)` / `writeMessage(const CanMessage&, timeoutMs)`
- `readFrame(twai_message_t*, timeoutMs)` / `writeFrame(const twai_message_t*, timeoutMs)`
- `getHealth(BusHealth&)` / `isBusOff()` / `getState(twai_state_t&)`
- `recover(timeoutMs = 1000)` — blocking manual recovery (for `autoRecover = false`)
- `readAlerts(uint32_t&, timeoutMs)` / `getRXQueueLength(int&)`

### `CanMessage`
Typed constructors (`float`/`int32_t`/`uint8_t`/`bool`/raw bytes) and matching
getters. Payloads are little-endian (ESP32 native); check `getDataType()` before
decoding. Message IDs and expected types live in `CanDatabase.h`.

### Multi-field sensor messages (GPS / IMU)

A CAN frame is capped at 8 bytes, so multi-field readings use packed structs:

| Struct | Fields | Bytes | Datatype tag |
|---|---|---|---|
| `GpsPosition` | `latitude`, `longitude` (deg, float) | 8 | `GPS_POSITION` |
| `GpsVelocity` | `speed` (m/s), `heading` (deg, float) | 8 | `GPS_VELOCITY` |
| `ImuVec3` (accel) | `x,y,z` int16, **milli-g** | 6 | `IMU_ACCEL` |
| `ImuVec3` (gyro) | `x,y,z` int16, **0.1 deg/s** | 6 | `IMU_GYRO` |

IMU axes use `int16` (not `float`) so all three fit in a single frame; the
sender applies the scaling above. Default DAQ IDs: `GPS_POSITION_MSG` `0x223`,
`GPS_VELOCITY_MSG` `0x224`, `IMU_ACCEL_MSG` `0x225`, `IMU_GYRO_MSG` `0x226`.

```cpp
// send
GpsPosition pos{42.3601f, -71.0589f};
can.writeMessage(CanMessage(CanDatabase::GPS_POSITION_MSG.id, pos), 5);

ImuVec3 accel{10, -20, 1000};                       // 1000 milli-g = 1 g on z
can.writeMessage(CanMessage(CanDatabase::IMU_ACCEL_MSG.id, accel, /*isGyro=*/false), 5);

// receive
if (can.readMessage(msg, 5) == ESP_OK && msg.getDataType() == GPS_POSITION) {
    GpsPosition p = msg.getGpsPosition();
    (void)p.latitude;
}
```

Common returns: `ESP_OK`, `ESP_ERR_TIMEOUT`, `ESP_ERR_INVALID_STATE`,
`ESP_ERR_INVALID_RESPONSE`.
