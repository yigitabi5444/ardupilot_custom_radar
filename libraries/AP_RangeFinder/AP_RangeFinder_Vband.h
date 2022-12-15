#pragma once

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

class AP_RangeFinder_VBand : public AP_RangeFinder_Backend_Serial
{
public:

    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return new AP_RangeFinder_VBand(_state, _params);
    }

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_RADAR;
    }

    // baudrate used during object construction:
    uint32_t initial_baudrate(uint8_t serial_instance) const override {
        return 115200;
    }

    uint16_t rx_bufsize() const override { return 128; }
    uint16_t tx_bufsize() const override { return 128; }

private:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    // get a reading
    bool get_reading(float &reading_m) override;

    char _stringBuffer[16];
    uint8_t  _stringBuffer_index, comma_count;
    int16_t BIN, I, Q, J, K;
};
