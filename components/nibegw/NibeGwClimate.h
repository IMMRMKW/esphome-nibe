#pragma once

#include <cstddef>
#include <map>
#include <vector>

#include "esphome/components/climate/climate.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"

namespace esphome {
namespace nibegw {

    class NibeGwComponent;

    class NibeGwClimate : public climate::Climate, public Component {
    public:
        void setup() override;
        void dump_config();
        void set_sensor(sensor::Sensor* sensor)
        {
            this->sensor_ = sensor;
        }
        void set_gw(NibeGwComponent* gw)
        {
            this->gw_ = gw;
        }
        void set_system(int system)
        {
            this->index_ = system - 1;
            this->address_ = 0x19 + this->index_;
        }

    protected:
        /// Override control to change settings of the climate device.
        void control(const climate::ClimateCall& call) override;
        void publish_current_temperature();
        void publish_current_temperature(float value);
        void publish_set_point(float value);
        void restart_timeout_on_data();
        void restart_timeout_on_sensor();

        typedef std::map<int, std::vector<uint8_t>> data_index_map_t;
        using request_data_type = std::vector<uint8_t>;

        // Utility functions for request/response data
        static request_data_type build_request_data(uint8_t token, const request_data_type& payload);
        static request_data_type set_u16_index(int index, int value);
        static request_data_type set_u16(int value);
        static uint16_t get_u16(const uint8_t data[2]);
        static float get_s16_decimal(uint16_t value, float scale, int offset);
        static float get_s16_decimal(const uint8_t data[2], float scale, int offset);
        static request_data_type set_s16_decimal(float value, float scale, int offset);
        static request_data_type set_u8_decimal(float value, float scale, int offset);
        static float get_u8_decimal(const uint8_t data[1], float scale, int offset);

        static constexpr int int16_invalid = -0x8000;
        static constexpr int uint8_invalid = 0xFF;
        static constexpr int RMU_WRITE_INDEX_TEMPERATURE = 6;
        static constexpr int RMU_WRITE_INDEX_SETPOINT_S1 = 9;
        static constexpr int RMU_WRITE_INDEX_SETPOINT_SX(int index) { return RMU_WRITE_INDEX_SETPOINT_S1 + (index) * 2; }
        static constexpr const char* TAG = "nibegw";

        data_index_map_t::iterator next_data(); /* return next data index */

        /// Return the traits of this controller.
        climate::ClimateTraits traits() override;

        NibeGwComponent* gw_ { nullptr };
        sensor::Sensor* sensor_ { nullptr };
        int address_;
        int index_;
        int data_index_; /* last transmitted data index */
        data_index_map_t data_;
    };

} // namespace nibegw
} // namespace esphome
