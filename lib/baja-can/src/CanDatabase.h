#pragma once
#include "BajaCan.h"
#include <stddef.h>

typedef struct
{
    uint32_t id;
    CanDataType type;
    uint8_t dlc; // minimum payload length required for this ID
} CanMessageDef;

namespace CanDatabase
{
    // CAN ID scheme: first three bits = sender (100=ECVT, 010=DAQ, 001=HUD),
    // next three bits = receiver(s), last five bits = message id (32 options).
    constexpr uint32_t MASK      = 0b11100011111;
    constexpr uint32_t ECVT_CODE = 0b00010000000;
    constexpr uint32_t DAQ_CODE  = 0b00001000000;
    constexpr uint32_t HUD_CODE  = 0b00000100000;

    constexpr CanMessageDef ENGINE_RPM     = {0x441, FLOAT, sizeof(float)};
    constexpr CanMessageDef SECONDARY_RPM  = {0x442, FLOAT, sizeof(float)};
    constexpr CanMessageDef MODE           = {0x443, UINT8, sizeof(uint8_t)};
    constexpr CanMessageDef STATUS         = {0x444, UINT8, sizeof(uint8_t)};
    constexpr CanMessageDef TEMPERATURE    = {0x445, FLOAT, sizeof(float)};
    constexpr CanMessageDef MOTOR_SETPOINT = {0x446, INT32, sizeof(int32_t)};
    constexpr CanMessageDef MOTOR_POSITION = {0x447, INT32, sizeof(int32_t)};

    constexpr CanMessageDef FUEL_LEVEL   = {0x222, FLOAT, sizeof(float)};
    constexpr CanMessageDef LINEAR_SPEED = {0x221, FLOAT, sizeof(float)};

    // DAQ sensors: GPS + IMU. Multi-field payloads packed to <= 8 bytes.
    constexpr CanMessageDef GPS_POSITION_MSG = {0x223, GPS_POSITION, sizeof(float) * 2}; // lat, lon
    constexpr CanMessageDef GPS_VELOCITY_MSG = {0x224, GPS_VELOCITY, sizeof(float) * 2}; // speed, heading
    constexpr CanMessageDef IMU_ACCEL_MSG    = {0x225, IMU_ACCEL, sizeof(int16_t) * 3};  // ax, ay, az (milli-g)
    constexpr CanMessageDef IMU_GYRO_MSG     = {0x226, IMU_GYRO, sizeof(int16_t) * 3};   // gx, gy, gz (0.1 deg/s)

    constexpr CanMessageDef BRAKE_POT               = {0x141, FLOAT, sizeof(float)};
    constexpr CanMessageDef FRONT_LEFT_WHEEL_SPEED  = {0x142, FLOAT, sizeof(float)};
    constexpr CanMessageDef FRONT_RIGHT_WHEEL_SPEED = {0x143, FLOAT, sizeof(float)};

    constexpr CanMessageDef canDatabase[] = {
        ENGINE_RPM,
        SECONDARY_RPM,
        MODE,
        STATUS,
        TEMPERATURE,
        FUEL_LEVEL,
        MOTOR_SETPOINT,
        MOTOR_POSITION,
        LINEAR_SPEED,
        GPS_POSITION_MSG,
        GPS_VELOCITY_MSG,
        IMU_ACCEL_MSG,
        IMU_GYRO_MSG,
        BRAKE_POT,
        FRONT_LEFT_WHEEL_SPEED,
        FRONT_RIGHT_WHEEL_SPEED};

    constexpr size_t canDatabaseSize = sizeof(canDatabase) / sizeof(canDatabase[0]);
}
