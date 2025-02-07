/** Sapienza Space Team
 * @author Niccolò Gatti 
 * @author Vittorio De Lucia
 * @date 08/06/2024
 * @copyright 2024 Copyright (c) | Sapienza Space Team, all rights reserved.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include "mpu6050.hpp"

LOG_MODULE_REGISTER(MPU6050Sensor, LOG_LEVEL_DBG);
namespace drivers {
    MPU6050Sensor::MPU6050Sensor(const device *_device): dev {_device} {
        // Check for device availability
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

    MPU6050Sensor::~MPU6050Sensor() = default;

    int MPU6050Sensor::getAccel() {
        if(this->ready) {
            if (sensor_sample_fetch(dev) < 0) {
                LOG_ERR("Could not fetch data from \"%s\"\n", dev->name);
                return -1;
            }

            int rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);

            return rc;
        }
        return -1;
    }

    int MPU6050Sensor::getGyro() {
        if(this->ready) {
            if (sensor_sample_fetch(dev) < 0) {
                LOG_ERR("Could not fetch data from \"%s\"\n", dev->name);
                return -1;
            }

            int rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);

            return rc;
        }
        return -1;
    }
}