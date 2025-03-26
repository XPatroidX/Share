#include <zephyr/drivers/gpio.h>

#ifndef CANSAT_GPIO
#define CANSAT_GPIO

namespace drivers::gpio {
    class Gpio {
        public:
            explicit Gpio(const gpio_dt_spec &dev): dev_ {dev} {}
        protected:
            int configure(gpio_flags_t flags) {
                return gpio_pin_configure_dt(&dev_, flags);
            }
            int toggle_pin() {
                return gpio_pin_toggle_dt(&dev_);
            }
            const gpio_dt_spec &dev_;
    };
}

#endif