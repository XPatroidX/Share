#include "zephyr/dt-bindings/gpio/gpio.h"
#include "zephyr/dt-bindings/input/input-event-codes.h"
#include <stdint.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <cerrno>
#include <cstdint>
#include <string>
#include <float.h>

using namespace std;



LOG_MODULE_REGISTER(logging_blog, LOG_LEVEL_DBG);


#define PIN_A               DT_NODELABEL(pina)


int main() 
{
    //const struct device *pina = DEVICE_DT_GET(PIN_A);
    static const struct gpio_dt_spec pin_a = GPIO_DT_SPEC_GET(PIN_A, gpios);

    gpio_pin_configure_dt(&pin_a, GPIO_INPUT);

    uint16_t passes = 0;
    bool passed = 1;
    while (1) {
        if((gpio_pin_get(pin_a.port, 10) == 1) && passed) {
            passes++;
            printk("passes: %d\n", passes);
            passed = 0;
        }
        if((gpio_pin_get(pin_a.port, 10) == 0))
            passed = 1;

    }
    return 0;
}