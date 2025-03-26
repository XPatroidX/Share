/** Sapienza Space Team
 * @author Niccolò Gatti 
 * @author Vittorio De Lucia
 * @date 08/06/2024
 * @copyright 2024 Copyright (c) | Sapienza Space Team, all rights reserved.
 */

#include "led.hpp"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(led, LOG_LEVEL_DBG);

namespace payload::subsys {
    Led::Led(const gpio_dt_spec& dev): drivers::gpio::Gpio(dev) {
        auto ret = configure(GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to set %s to GPIO_OUTPUT_ACTIVE", dev_.port->name);
        }
    }

    void Led::toggleLed(){
        toggle_pin();
    }

    void Led::blink(int sleepTimeMs){
        toggleLed();
        k_msleep(sleepTimeMs);   
        toggleLed();
    }
}