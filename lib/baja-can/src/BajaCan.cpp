#include "BajaCan.h"
#include "CanDatabase.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

using namespace CanDatabase;

// pdMS_TO_TICKS overflows on portMAX_DELAY, so pass it through untouched to
// preserve the documented "block indefinitely" behavior.
static TickType_t toTicks(uint32_t timeoutMs)
{
    return (timeoutMs == portMAX_DELAY) ? portMAX_DELAY : pdMS_TO_TICKS(timeoutMs);
}

static twai_timing_config_t timingFor(CanBitrate br)
{
    switch (br)
    {
    case CanBitrate::BR_125K: return TWAI_TIMING_CONFIG_125KBITS();
    case CanBitrate::BR_250K: return TWAI_TIMING_CONFIG_250KBITS();
    case CanBitrate::BR_1M:   return TWAI_TIMING_CONFIG_1MBITS();
    case CanBitrate::BR_500K:
    default:                  return TWAI_TIMING_CONFIG_500KBITS();
    }
}

static uint32_t nowMs()
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

// ---------------------------------------------------------------------------
// BajaCan lifecycle
// ---------------------------------------------------------------------------

BajaCan::BajaCan(gpio_num_t tx, gpio_num_t rx)
    : initialized(false), txPin(tx), rxPin(rx),
      recovering(false), recoverStartedMs(0),
      busOffEvents(0), recoveries(0)
{
}

esp_err_t BajaCan::begin(const CanConfig& config, uint32_t acceptanceCode)
{
    if (initialized)
    {
        return ESP_OK;
    }

    cfg = config;

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        txPin, rxPin, TWAI_MODE_NORMAL);
    g_config.rx_queue_len = cfg.rxQueueLen;
    g_config.tx_queue_len = cfg.txQueueLen;
    g_config.alerts_enabled = TWAI_ALERT_BUS_OFF
                            | TWAI_ALERT_BUS_RECOVERED
                            | TWAI_ALERT_ERR_PASS
                            | TWAI_ALERT_TX_FAILED
                            | TWAI_ALERT_RX_QUEUE_FULL;

    twai_timing_config_t t_config = timingFor(cfg.bitrate);
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (acceptanceCode != 0)
    {
        f_config.acceptance_code = acceptanceCode;
        f_config.acceptance_mask = MASK; // only match bits set in the mask
        f_config.single_filter = true;
    }

    esp_err_t ret = twai_driver_install(&g_config, &t_config, &f_config);
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = twai_start();
    if (ret == ESP_OK)
    {
        initialized = true;
        recovering = false;
        busOffEvents = 0;
        recoveries = 0;
    }
    return ret;
}

esp_err_t BajaCan::end()
{
    if (!initialized)
    {
        return ESP_OK;
    }
    esp_err_t ret = twai_stop();
    if (ret != ESP_OK)
    {
        return ret;
    }
    ret = twai_driver_uninstall();
    if (ret == ESP_OK)
    {
        initialized = false;
        recovering = false;
    }
    return ret;
}

// ---------------------------------------------------------------------------
// Raw frame I/O
// ---------------------------------------------------------------------------

esp_err_t BajaCan::readFrame(twai_message_t* frame, uint32_t timeoutMs)
{
    if (!initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }
    return twai_receive(frame, toTicks(timeoutMs));
}

esp_err_t BajaCan::writeFrame(const twai_message_t* frame, uint32_t timeoutMs)
{
    if (!initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }
    // Don't feed the TX queue while bus-off/recovering — the frame would just
    // rot in the queue. Report the condition so the caller can back off.
    if (recovering)
    {
        return ESP_ERR_INVALID_STATE;
    }
    return twai_transmit(frame, toTicks(timeoutMs));
}

// ---------------------------------------------------------------------------
// Typed message I/O
// ---------------------------------------------------------------------------

esp_err_t BajaCan::writeMessage(const CanMessage& message, uint32_t timeoutMs)
{
    return writeFrame(&message.getFrame(), timeoutMs);
}

esp_err_t BajaCan::readMessage(CanMessage& message, uint32_t timeoutMs)
{
    twai_message_t frame;
    esp_err_t ret = readFrame(&frame, timeoutMs);
    if (ret == ESP_OK)
    {
        message.setFrame(frame);
        message.setDataType(BYTES); // safe default for unknown IDs

        for (size_t i = 0; i < canDatabaseSize; ++i)
        {
            const CanMessageDef& def = canDatabase[i];
            if (def.id == frame.identifier)
            {
                if (frame.data_length_code < def.dlc)
                {
                    return ESP_ERR_INVALID_RESPONSE; // truncated payload
                }
                message.setDataType(def.type);
                break;
            }
        }
    }
    return ret;
}

// ---------------------------------------------------------------------------
// Health / status
// ---------------------------------------------------------------------------

esp_err_t BajaCan::getRXQueueLength(int& length)
{
    if (!initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }
    twai_status_info_t status;
    esp_err_t ret = twai_get_status_info(&status);
    if (ret == ESP_OK)
    {
        length = status.msgs_to_rx;
    }
    return ret;
}

esp_err_t BajaCan::getState(twai_state_t& state)
{
    if (!initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }
    twai_status_info_t status;
    esp_err_t ret = twai_get_status_info(&status);
    if (ret == ESP_OK)
    {
        state = status.state;
    }
    return ret;
}

esp_err_t BajaCan::getHealth(BusHealth& health)
{
    if (!initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }
    twai_status_info_t status;
    esp_err_t ret = twai_get_status_info(&status);
    if (ret != ESP_OK)
    {
        return ret;
    }
    health.state          = status.state;
    health.txErrCounter   = status.tx_error_counter;
    health.rxErrCounter   = status.rx_error_counter;
    health.msgsToTx       = status.msgs_to_tx;
    health.msgsToRx       = status.msgs_to_rx;
    health.txFailedCount  = status.tx_failed_count;
    health.rxMissedCount  = status.rx_missed_count;
    health.rxOverrunCount = status.rx_overrun_count;
    health.busOffEvents   = busOffEvents;
    health.recoveries     = recoveries;
    return ESP_OK;
}

bool BajaCan::isBusOff()
{
    if (!initialized)
    {
        return false;
    }
    if (recovering)
    {
        return true;
    }
    twai_status_info_t status;
    if (twai_get_status_info(&status) != ESP_OK)
    {
        return false;
    }
    return status.state == TWAI_STATE_BUS_OFF;
}

esp_err_t BajaCan::readAlerts(uint32_t& alerts, uint32_t timeoutMs)
{
    if (!initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }
    return twai_read_alerts(&alerts, toTicks(timeoutMs));
}

// ---------------------------------------------------------------------------
// Self-healing bus recovery
// ---------------------------------------------------------------------------
//
// The TWAI peripheral does NOT auto-recover from a bus-off condition: after
// the TX error counter passes 255 it stops driving the bus and stays there
// until firmware runs the recovery sequence (initiate recovery -> wait for the
// controller to reach STOPPED -> start again).
//
// poll() drives this as a non-blocking state machine so it is safe to call
// every loop iteration with timeoutMs = 0. The one unavoidable blocking spot
// is the settle delay after the controller reaches STOPPED, which is bounded
// by cfg.recoverDelayMs.

esp_err_t BajaCan::startRecovery()
{
    esp_err_t ret = twai_initiate_recovery();
    if (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE)
    {
        // INVALID_STATE means recovery is already underway — treat as started.
        recovering = true;
        recoverStartedMs = nowMs();
        return ESP_OK;
    }
    return ret;
}

esp_err_t BajaCan::finishRecovery()
{
    twai_status_info_t status;
    esp_err_t ret = twai_get_status_info(&status);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Wait until the controller has finished recovering and reached STOPPED.
    if (status.state != TWAI_STATE_STOPPED)
    {
        return ESP_ERR_TIMEOUT; // not ready yet; poll() will retry
    }

    // Give the bus a moment to settle before we start driving it again.
    if (cfg.recoverDelayMs > 0)
    {
        uint32_t elapsed = nowMs() - recoverStartedMs;
        if (elapsed < cfg.recoverDelayMs)
        {
            vTaskDelay(pdMS_TO_TICKS(cfg.recoverDelayMs - elapsed));
        }
    }

    ret = twai_start();
    if (ret == ESP_OK)
    {
        recovering = false;
        recoveries++;
    }
    return ret;
}

esp_err_t BajaCan::poll(uint32_t timeoutMs)
{
    if (!initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    // Drain pending alerts (non-blocking by default) so we notice bus-off even
    // if the caller never reads alerts directly.
    uint32_t alerts = 0;
    esp_err_t alertRet = twai_read_alerts(&alerts, toTicks(timeoutMs));
    if (alertRet == ESP_OK && (alerts & TWAI_ALERT_BUS_OFF))
    {
        busOffEvents++;
    }

    if (!cfg.autoRecover)
    {
        return ESP_OK; // caller drives recovery via recover()
    }

    // Advance the recovery state machine.
    if (recovering)
    {
        esp_err_t ret = finishRecovery();
        // ESP_ERR_TIMEOUT just means "still recovering"; not an error to caller.
        return (ret == ESP_ERR_TIMEOUT) ? ESP_OK : ret;
    }

    // Not currently recovering — check whether we need to start.
    twai_status_info_t status;
    if (twai_get_status_info(&status) == ESP_OK
        && status.state == TWAI_STATE_BUS_OFF)
    {
        return startRecovery();
    }

    return ESP_OK;
}

esp_err_t BajaCan::recover(uint32_t timeoutMs)
{
    if (!initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    twai_status_info_t status;
    esp_err_t ret = twai_get_status_info(&status);
    if (ret != ESP_OK)
    {
        return ret;
    }

    if (status.state != TWAI_STATE_BUS_OFF
        && status.state != TWAI_STATE_STOPPED
        && !recovering)
    {
        return ESP_OK; // already running, nothing to do
    }

    if (status.state == TWAI_STATE_BUS_OFF && !recovering)
    {
        ret = startRecovery();
        if (ret != ESP_OK)
        {
            return ret;
        }
    }

    // Block (bounded) until the controller reaches STOPPED, then restart.
    const uint32_t pollMs = 10;
    uint32_t waited = 0;
    while (waited < timeoutMs)
    {
        if (twai_get_status_info(&status) == ESP_OK
            && status.state == TWAI_STATE_STOPPED)
        {
            return finishRecovery();
        }
        vTaskDelay(pdMS_TO_TICKS(pollMs));
        waited += pollMs;
    }
    return ESP_ERR_TIMEOUT;
}

// ---------------------------------------------------------------------------
// CanMessage — encode
// ---------------------------------------------------------------------------

CanMessage::CanMessage(uint32_t id, float value)
{
    memset(&frame, 0, sizeof(frame));
    dataType = FLOAT;
    frame.identifier = id;
    frame.data_length_code = sizeof(float);
    memcpy(frame.data, &value, sizeof(float));
}

CanMessage::CanMessage(uint32_t id, int32_t value)
{
    memset(&frame, 0, sizeof(frame));
    dataType = INT32;
    frame.identifier = id;
    frame.data_length_code = sizeof(int32_t);
    memcpy(frame.data, &value, sizeof(int32_t));
}

CanMessage::CanMessage(uint32_t id, uint8_t value)
{
    memset(&frame, 0, sizeof(frame));
    dataType = UINT8;
    frame.identifier = id;
    frame.data_length_code = sizeof(uint8_t);
    frame.data[0] = value;
}

CanMessage::CanMessage(uint32_t id, bool value)
{
    memset(&frame, 0, sizeof(frame));
    dataType = BOOL;
    frame.identifier = id;
    frame.data_length_code = 1U;
    frame.data[0] = value ? 1U : 0U;
}

CanMessage::CanMessage(uint32_t id, const uint8_t* data, uint8_t len)
{
    memset(&frame, 0, sizeof(frame));
    dataType = BYTES;
    frame.identifier = id;

    const uint8_t clampedLen = (len > sizeof(frame.data)) ? sizeof(frame.data) : len;
    frame.data_length_code = clampedLen;
    if ((data != nullptr) && (clampedLen > 0U))
    {
        memcpy(frame.data, data, clampedLen);
    }
}

CanMessage::CanMessage(uint32_t id, const GpsPosition& pos)
{
    memset(&frame, 0, sizeof(frame));
    dataType = GPS_POSITION;
    frame.identifier = id;
    frame.data_length_code = sizeof(GpsPosition); // 8 bytes
    memcpy(frame.data, &pos, sizeof(GpsPosition));
}

CanMessage::CanMessage(uint32_t id, const GpsVelocity& vel)
{
    memset(&frame, 0, sizeof(frame));
    dataType = GPS_VELOCITY;
    frame.identifier = id;
    frame.data_length_code = sizeof(GpsVelocity); // 8 bytes
    memcpy(frame.data, &vel, sizeof(GpsVelocity));
}

CanMessage::CanMessage(uint32_t id, const ImuVec3& vec, bool isGyro)
{
    memset(&frame, 0, sizeof(frame));
    dataType = isGyro ? IMU_GYRO : IMU_ACCEL;
    frame.identifier = id;
    frame.data_length_code = 3U * sizeof(int16_t); // 6 bytes, layout-independent
    // Pack x,y,z explicitly to avoid depending on struct padding.
    memcpy(&frame.data[0], &vec.x, sizeof(int16_t));
    memcpy(&frame.data[2], &vec.y, sizeof(int16_t));
    memcpy(&frame.data[4], &vec.z, sizeof(int16_t));
}

CanMessage::CanMessage() : dataType(NONE)
{
    memset(&frame, 0, sizeof(frame));
}

// ---------------------------------------------------------------------------
// CanMessage — decode
// ---------------------------------------------------------------------------

float CanMessage::getFloat() const
{
    float out = 0.0F;
    if (frame.data_length_code >= sizeof(float))
    {
        memcpy(&out, frame.data, sizeof(float));
    }
    return out;
}

int32_t CanMessage::getInt32() const
{
    int32_t out = 0;
    if (frame.data_length_code >= sizeof(int32_t))
    {
        memcpy(&out, frame.data, sizeof(int32_t));
    }
    return out;
}

uint8_t CanMessage::getUInt8() const
{
    if (frame.data_length_code >= 1U)
    {
        return frame.data[0];
    }
    return 0U;
}

bool CanMessage::getBool() const
{
    return getUInt8() != 0U;
}

GpsPosition CanMessage::getGpsPosition() const
{
    GpsPosition out = {0.0F, 0.0F};
    if (frame.data_length_code >= sizeof(GpsPosition))
    {
        memcpy(&out, frame.data, sizeof(GpsPosition));
    }
    return out;
}

GpsVelocity CanMessage::getGpsVelocity() const
{
    GpsVelocity out = {0.0F, 0.0F};
    if (frame.data_length_code >= sizeof(GpsVelocity))
    {
        memcpy(&out, frame.data, sizeof(GpsVelocity));
    }
    return out;
}

ImuVec3 CanMessage::getImuVec3() const
{
    ImuVec3 out = {0, 0, 0};
    if (frame.data_length_code >= 3U * sizeof(int16_t))
    {
        memcpy(&out.x, &frame.data[0], sizeof(int16_t));
        memcpy(&out.y, &frame.data[2], sizeof(int16_t));
        memcpy(&out.z, &frame.data[4], sizeof(int16_t));
    }
    return out;
}
