/** Sapienza Space Team
 * @author Gabriele Di Pietro
 * @author Francesco Baldassarre
 * @author Marco Niutta
 * @date 08/06/2024
 * @copyright 2024 Copyright (c) | Sapienza Space Team, all rights reserved.
 */

#include <cstdint>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/logging/log.h>
#include "ds3231.hpp"

LOG_MODULE_REGISTER(DS3231Sensor, LOG_LEVEL_DBG);

namespace drivers::ds3231 {
    struct DS3231Sensor::time_h_m_s
    {
        int8_t hours;
        int8_t minutes;
        int8_t seconds;
    };

    DS3231Sensor::DS3231Sensor(const device *_device): dev {_device} {
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

    DS3231Sensor::~DS3231Sensor() = default;


    /*
     * Wrapper method to set the "start_time" variable in normal initizalization
     *
     * Returns:
     *    ->      0 if reading is successful
     *    ->     -1 the data is not retrived
     *    ->     -2 device not ready
     */
    int8_t DS3231Sensor::start()
    {
        if(ready)
        return get_counter(this->start_time);
    }



    /*
     * Wrapper method to set the "start_time" variable after a system crash
     */
    void DS3231Sensor::start(uint32_t& t)
    {
        this->start_time = t; 
    }



    /*
     * Reads the counter value
     *
     * Returns:
     *    ->      0 if reading is successful
     *    ->     -1 the data is not retrived
     *    ->     -2 device not ready
     */
    
    int8_t DS3231Sensor::get_counter(uint32_t& c) {
        if(ready)
        {
            if(counter_get_value(dev, &c) < 0) {
                LOG_ERR("Could not fetch data from \"%s\"\n", dev->name);
                return -1;
            }
            return 0;
        }
        return -2;
    }


    /*
     * Gives the the hours, minutes and seconds elapsed from mission start
     *
     * Returns:
     *    ->      0 if reading is successful
     *    ->     -1 the data is not retrived
     *    ->     -2 device not ready
     */

    int8_t DS3231Sensor::get_time_elapsed(time_h_m_s& time)
    {
        uint32_t now;
        int8_t er = DS3231Sensor::get_counter(now);
        if(!er)
        {
            uint32_t elapsed = now - this->start_time;
            time.hours = elapsed / 3600;
            time.minutes = (elapsed % 3600) / 60;
            time.seconds = elapsed % 60;
            return 0;
        }
        return er;
    }
}
