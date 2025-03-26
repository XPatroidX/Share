#ifndef PAYLOAD_BME280_HPP
#define PAYLOAD_BME280_HPP

#include <zephyr/device.h>

namespace drivers::bme280 {
    class BME280Sensor {
        public:
            explicit BME280Sensor(const device *_device);
            ~BME280Sensor();
            double getActualPressure();
            double getActualTemperature();
        
            double reference_altitude;
        private:
            const device *dev;
            bool ready = false;
    };
}

#endif // PAYLOAD_BME280_HPP