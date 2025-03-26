#ifndef PAYLOAD_FC33_HPP
#define PAYLOAD_FC33_HPP

#include <zephyr/drivers/gpio.h>
#include <sys/types.h>

namespace drivers::FC33 {
    class FC33Sensor {
        public:
            explicit FC33Sensor(const gpio_dt_spec *_gpio, const uint16_t _sampling_period, const uint8_t _resolution);
            ~FC33Sensor();
            uint16_t getRotationVelocity();
            uint8_t startReading();
        
        private:
            uint16_t rotation_velocity;
            const uint8_t resolution;
            const uint16_t sampling_period;
            const gpio_dt_spec *gpio;
            bool ready = false;
            bool started = false;
    };
}

#endif // PAYLOAD_FC33_HPP