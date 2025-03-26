/** Sapienza Space Team
 * @author Gabriele Di Pietro
 * @author Francesco Baldassarre
 * @author Marco Niutta
 * @date 08/06/2024
 * @copyright 2024 Copyright (c) | Sapienza Space Team, all rights reserved.
 */

 #include <zephyr/kernel.h>
 #include <zephyr/drivers/gpio.h>
 #include <zephyr/logging/log.h>
 #include "fc33.hpp"
 
 LOG_MODULE_REGISTER(FC33Sensor, LOG_LEVEL_DBG);
 namespace drivers::FC33 {
     FC33Sensor::FC33Sensor(const gpio_dt_spec *_gpio, const uint16_t _sampling_period, const uint8_t _resolution): resolution {_resolution}, sampling_period {_sampling_period}, gpio {_gpio}  {
         // Check for device availability
         if (!gpio_is_ready_dt(gpio)) {
             LOG_ERR("\nError: Device \"%hhu\" is not ready; "
                 "check the driver initialization logs for errors.\n",
                 gpio->pin);
         } else {
             LOG_INF("Found device \"%hhu\", getting sensor data\n", gpio->pin);
             this->ready = true;
         }
     }
 
     FC33Sensor::~FC33Sensor() = default;

        /*
      * Start the sensor, it will now measure the steps for a time range specified by sampling_period
      * and calculate an angular velocity in deg°/s
      *
      * Returns:
      *    ->     -1 if the device is not ready or if the data is not retrived
      */
 
     uint8_t FC33Sensor::startReading()
     {
        if(this->ready)
        {
            uint32_t begin = k_uptime_get();
            uint32_t steps = 0;
            this->started = true;
            while (1)
            {
                bool passed;
                if((k_uptime_get() - begin) > this->sampling_period)
                {
                    begin = k_uptime_get();
                    this->rotation_velocity = (steps / resolution) * (1000/sampling_period) * (360/resolution);
                    steps = 0;
                }
   
                if((gpio_pin_get(gpio->port, gpio->pin) == 1) && passed) {
                    steps++;
                    passed = 0;
                }
                if((gpio_pin_get(gpio->port, gpio->pin) == 0))
                    passed = 1;
                else 
                {
                    LOG_ERR("Could not fetch data from \"%hhu\"\n", gpio->pin);
                    return -1;
                }
            }
        }
        return -1;
     }

        /*
      * Access to the rotation velocity
      *
      * Returns:
      *    ->     rotation_velocity
      *    ->     -1 if the device is not ready, or isn't started
      */
     uint16_t FC33Sensor::getRotationVelocity() {
        if(this->ready && this->started) 
            return this->rotation_velocity;
        return -1;
     }
 }