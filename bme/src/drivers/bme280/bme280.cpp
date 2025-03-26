/**
 * @file bme280.cpp
 * @author Lorenzo Thomas Contessa <lorenzocontessa.dev@gmail.com>
 * @date 24/02/2023
 * @copyright 2023 Copyright (c) | Sapienza Space Team, all rights reserved.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include "bme280.hpp"

LOG_MODULE_REGISTER(BME280Sensor, LOG_LEVEL_DBG);
namespace drivers::bme280 {
    BME280Sensor::BME280Sensor(const device *_device) : dev {_device} {
        if (!device_is_ready(dev)) {
            LOG_ERR("\nError: Device \"%s\" is not ready; "
                "check the driver initialization logs for errors.\n",
                dev->name);
            this->dev = NULL;
        } else {
            LOG_INF("Found device \"%s\", getting sensor data\n", dev->name);
            this->ready = true;
        }
    }

    BME280Sensor::~BME280Sensor() = default;

    double BME280Sensor::getActualPressure() {
        if(this->ready) {
            struct sensor_value press;

            // Fetch data from the sensor
            if (sensor_sample_fetch(dev) < 0) {
                LOG_ERR("Could not fetch data from \"%s\"\n", dev->name);
                return -1;
            }

            // Get data channel value
            sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press);

            // Cast to double value
            double pressure = static_cast<double>(press.val1) + (static_cast<double>(press.val2) / 1e6);

            return pressure;
        }

        // Error return
        LOG_ERR("Can't read pressure from \"%s\" due to device unavailability", dev->name);
        return -1;
    }

    double BME280Sensor::getActualTemperature() {
        if(this->ready) {
            struct sensor_value temp;

            // Fetch data from the sensor
            if (sensor_sample_fetch(dev) < 0) {
                LOG_ERR("Could not fetch data from \"%s\"\n", dev->name);
                return -1;
            }

            // Get data channel value
            sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);

            // Cast to double value
            double temperature = static_cast<double>(temp.val1) + (static_cast<double>(temp.val2) / 1e6);

            return temperature;
        }

        // Error return
        LOG_ERR("Can't read temperature from \"%s\" due to device unavailability", dev->name);
        return -274;
    }
}