// basic_usage.cpp — minimal example of the baja-can library.
//
// This is illustrative only (not built by any board env). Copy the relevant
// parts into your board's boards/<name>/src/main.cpp.
//
// Wiring: connect the ESP32 TX/RX pins below to a CAN transceiver
// (e.g. SN65HVD230), then the transceiver's CANH/CANL to the bus.

#include "BajaCan.h"
#include "CanDatabase.h"

// (tx, rx) — pick pins that suit your board.
BajaCan can(GPIO_NUM_5, GPIO_NUM_4);

void setup() {
    CanConfig cfg;                       // defaults: 500 kbit, auto-recover on
    // cfg.bitrate = CanBitrate::BR_250K; // override if needed
    can.begin(cfg);
}

void loop() {
    // Call every loop: drains alerts and auto-recovers from a bus-off fault.
    can.poll();

    // --- Sending ---------------------------------------------------------

    // Scalar message: engine RPM as a float.
    can.writeMessage(CanMessage(CanDatabase::ENGINE_RPM.id, 3600.0f), 5);

    // GPS position (lat/lon packed into one 8-byte frame).
    GpsPosition pos{42.3601f, -71.0589f};
    can.writeMessage(CanMessage(CanDatabase::GPS_POSITION_MSG.id, pos), 5);

    // IMU acceleration (x/y/z in milli-g; 1000 == 1 g).
    ImuVec3 accel{0, 0, 1000};
    can.writeMessage(CanMessage(CanDatabase::IMU_ACCEL_MSG.id, accel, /*isGyro=*/false), 5);

    // --- Receiving -------------------------------------------------------

    CanMessage msg;
    if (can.readMessage(msg, 5) == ESP_OK) {
        switch (msg.getDataType()) {
            case FLOAT: {
                float rpm = msg.getFloat();
                (void)rpm;
                break;
            }
            case GPS_POSITION: {
                GpsPosition p = msg.getGpsPosition();
                (void)p.latitude;
                (void)p.longitude;
                break;
            }
            case IMU_ACCEL:
            case IMU_GYRO: {
                ImuVec3 v = msg.getImuVec3();
                (void)v.x;
                break;
            }
            default:
                break;
        }
    }

    // --- Health (optional) ----------------------------------------------

    BusHealth h;
    if (can.getHealth(h) == ESP_OK && h.busOffEvents > 0) {
        // Report bus faults over serial, an LED, etc.
    }
}
