/** Sapienza Space Team
 * @author Niccolò Gatti 
 * @author Vittorio De Lucia
 * @date 08/06/2024
 * @copyright 2024 Copyright (c) | Sapienza Space Team, all rights reserved.
 */

#include "drivers/gpio/gpio.hpp"

#ifndef CANSAT_LED
#define CANSAT_LED

namespace payload::subsys {
    class Led : public drivers::gpio::Gpio {
        public:
            explicit Led(const gpio_dt_spec &dev);
            void toggleLed();
            void blink(int sleepTimeMs);
    };
}

#endif