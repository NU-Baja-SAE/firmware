#pragma once
#ifndef BAJA_CAN_H
#define BAJA_CAN_H

// BajaCan — ESP32 TWAI (built-in CAN 2.0) driver wrapper for NU Baja SAE.
//
// Improvements over the original baja_can library:
//   * Self-healing bus: poll() watches alerts and auto-recovers from bus-off
//     with backoff — callers no longer have to hand-roll a recover() loop.
//   * Health/diagnostics: bus-off count, recovery count, TX/RX error counters
//     and controller state exposed via a single BusHealth snapshot.
//   * Configurable: bitrate, queue depths and auto-recovery are set through a
//     Config struct instead of being hardcoded.
//   * TX resilience: transmit failures trip the same recovery path so a wedged
//     controller can't silently stop talking.
//
// All boards are ESP32 / Arduino framework, but this only depends on the
// ESP-IDF twai driver, so it is framework-agnostic.

#include "driver/twai.h"
#include <stdint.h>
#include <stddef.h>

// --- Data typing -----------------------------------------------------------

typedef enum {
    FLOAT,
    INT32,
    UINT8,
    BOOL,
    BYTES,
    GPS_POSITION,   // packed GpsPosition (lat/lon), 8 bytes
    GPS_VELOCITY,   // packed GpsVelocity (speed/heading), 8 bytes
    IMU_ACCEL,      // packed ImuVec3 (accel, milli-g), 6 bytes
    IMU_GYRO,       // packed ImuVec3 (gyro, 0.1 deg/s), 6 bytes
    NONE
} CanDataType;

// --- Multi-field payloads --------------------------------------------------
//
// A TWAI frame carries at most 8 bytes, so multi-field sensor readings are
// packed into fixed little-endian layouts. IMU axes use int16 (not float) so
// all three fit one frame; the sender chooses the scaling documented below.

// GPS position: latitude + longitude in decimal degrees. 8 bytes.
struct GpsPosition {
    float latitude;    // degrees, +N / -S
    float longitude;   // degrees, +E / -W
};

// GPS velocity: ground speed + heading. 8 bytes.
struct GpsVelocity {
    float speed;       // m/s over ground
    float heading;     // degrees, 0..360, clockwise from true north
};

// Generic 3-axis IMU reading. 6 bytes. Units depend on message ID:
//   IMU_ACCEL -> milli-g       (e.g. 1000 == 1 g)
//   IMU_GYRO  -> 0.1 deg/s     (e.g. 100  == 10 deg/s)
struct ImuVec3 {
    int16_t x;
    int16_t y;
    int16_t z;
};

// A decoded/encodable CAN frame. twai_message_t is the single source of truth;
// the typed constructors and getters just encode/decode its payload bytes.
//
// Payloads are stored little-endian (ESP32 native). Every node in this project
// is little-endian ESP32, so no byte-swapping is performed. If a big-endian
// peer is ever added, encode/decode explicitly instead of relying on these.
class CanMessage {
public:
    CanMessage(uint32_t id, float value);
    CanMessage(uint32_t id, int32_t value);
    CanMessage(uint32_t id, uint8_t value);
    CanMessage(uint32_t id, bool value);
    CanMessage(uint32_t id, const uint8_t* data, uint8_t len);
    CanMessage(uint32_t id, const GpsPosition& pos);
    CanMessage(uint32_t id, const GpsVelocity& vel);
    // isGyro selects the datatype tag (IMU_GYRO vs IMU_ACCEL); the packed
    // layout is identical for both.
    CanMessage(uint32_t id, const ImuVec3& vec, bool isGyro = false);
    CanMessage();

    const twai_message_t& getFrame() const { return frame; }
    CanDataType getDataType() const { return dataType; }
    uint32_t getId() const { return frame.identifier; }
    uint8_t getDataLength() const { return frame.data_length_code; }
    const uint8_t* getBytes() const { return frame.data; }

    // Typed getters decode from frame bytes. Check getDataType() first; they
    // return a zero value if the frame is too short for the requested type.
    float getFloat() const;
    int32_t getInt32() const;
    uint8_t getUInt8() const;
    bool getBool() const;

    // Multi-field decoders. Return a zeroed struct if the frame is too short.
    GpsPosition getGpsPosition() const;
    GpsVelocity getGpsVelocity() const;
    ImuVec3 getImuVec3() const;

    void setDataType(CanDataType type) { dataType = type; }
    void setFrame(const twai_message_t& f) { frame = f; }

private:
    CanDataType dataType;
    twai_message_t frame;
};

// --- Bus health snapshot ---------------------------------------------------

// A point-in-time view of controller health. Fetch with BajaCan::getHealth().
struct BusHealth {
    twai_state_t state;        // RUNNING / BUS_OFF / STOPPED / RECOVERING
    uint32_t txErrCounter;     // TWAI TX error counter (>=128 => error-passive)
    uint32_t rxErrCounter;     // TWAI RX error counter
    uint32_t msgsToTx;         // frames queued for transmission
    uint32_t msgsToRx;         // frames waiting to be received
    uint32_t txFailedCount;    // frames that failed to transmit (driver counter)
    uint32_t rxMissedCount;    // frames dropped because RX queue was full
    uint32_t rxOverrunCount;   // frames lost to a hardware RX FIFO overrun

    // Library-maintained counters (survive across recovery cycles):
    uint32_t busOffEvents;     // times the bus has gone bus-off since begin()
    uint32_t recoveries;       // times auto/manual recovery completed
};

// --- Configuration ---------------------------------------------------------

enum class CanBitrate {
    BR_125K,
    BR_250K,
    BR_500K,   // NU Baja default
    BR_1M
};

struct CanConfig {
    CanBitrate bitrate      = CanBitrate::BR_500K;
    uint32_t rxQueueLen     = 50;
    uint32_t txQueueLen     = 20;
    bool autoRecover        = true;   // poll() auto-recovers from bus-off
    uint32_t recoverDelayMs = 100;    // settle time before restarting after bus-off
};

// --- Driver ----------------------------------------------------------------

class BajaCan {
public:
    BajaCan(gpio_num_t tx, gpio_num_t rx);

    // Install + start the driver. acceptanceCode/mask enable hardware ID
    // filtering (0 => accept all). Safe to call twice (returns ESP_OK).
    esp_err_t begin(const CanConfig& config = CanConfig{}, uint32_t acceptanceCode = 0);
    esp_err_t end();

    // Raw frame I/O.
    esp_err_t readFrame(twai_message_t* frame, uint32_t timeoutMs);
    esp_err_t writeFrame(const twai_message_t* frame, uint32_t timeoutMs);

    // Typed message I/O. readMessage() tags the CanDataType from CanDatabase.
    esp_err_t writeMessage(const CanMessage& message, uint32_t timeoutMs);
    esp_err_t readMessage(CanMessage& message, uint32_t timeoutMs);

    // Call this periodically (e.g. every loop iteration) with a small/zero
    // timeout. It drains pending alerts, updates health counters, and — when
    // autoRecover is enabled — drives the bus-off -> recovery -> running cycle
    // without blocking. Returns ESP_OK normally, or the error from a failed
    // recovery attempt.
    esp_err_t poll(uint32_t timeoutMs = 0);

    // Snapshot of current bus health (see BusHealth).
    esp_err_t getHealth(BusHealth& health);

    // True if the controller is bus-off or recovering (i.e. not RUNNING).
    bool isBusOff();

    // Manually run the full recovery sequence. poll() calls this for you when
    // autoRecover is on; exposed for callers that manage recovery themselves.
    esp_err_t recover(uint32_t timeoutMs = 1000);

    // Low-level status passthroughs (kept for API compatibility).
    esp_err_t getRXQueueLength(int& length);
    esp_err_t getState(twai_state_t& state);
    esp_err_t readAlerts(uint32_t& alerts, uint32_t timeoutMs);

private:
    esp_err_t startRecovery();          // bus-off -> initiate recovery
    esp_err_t finishRecovery();         // STOPPED -> start (after settle delay)

    bool initialized;
    gpio_num_t txPin;
    gpio_num_t rxPin;

    CanConfig cfg;

    // Auto-recovery state machine (non-blocking, advanced by poll()).
    bool recovering;
    uint32_t recoverStartedMs;

    // Persistent counters (BusHealth).
    uint32_t busOffEvents;
    uint32_t recoveries;
};

#endif // BAJA_CAN_H
