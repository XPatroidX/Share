#ifndef PAYLOAD_DS3231_HPP
#define PAYLOAD_DS3231_HPP

#include <cstdint>
#include <zephyr/device.h>

namespace drivers::ds3231 {
    class DS3231Sensor {
        public:
            struct time_h_m_s;
            explicit DS3231Sensor(const device *_device);
            ~DS3231Sensor();
            int8_t start();
            void start(uint32_t& t);
            int8_t get_time_elapsed(time_h_m_s& t);
            int8_t get_second_elapsed();
        
        private:
            int8_t get_counter(uint32_t& c);
            uint32_t start_time = 0;
            const device *dev;
            bool ready = false;
    };
}

#endif // PAYLOAD_DS3231_HPP
