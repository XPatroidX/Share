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

LOG_MODULE_REGISTER(logging_blog, LOG_LEVEL_DBG);


#define PIN_A               DT_NODELABEL(pina)
#define PIN_B               DT_NODELABEL(pinb)
#define PIN_C               DT_NODELABEL(pinc)
#define PIN_D               DT_NODELABEL(pind)
#define NUM_CONTROL_PINS    4
#define MAX_MICRO_STEP_RES  2

static const uint8_t
	half_step_lookup_table[NUM_CONTROL_PINS * MAX_MICRO_STEP_RES][NUM_CONTROL_PINS] = {
		{1u, 1u, 0u, 0u}, {0u, 1u, 0u, 0u}, {0u, 1u, 1u, 0u}, {0u, 0u, 1u, 0u},
		{0u, 0u, 1u, 1u}, {0u, 0u, 0u, 1u}, {1u, 0u, 0u, 1u}, {1u, 0u, 0u, 0u}};

int main() 
{
    //const struct device *pina = DEVICE_DT_GET(PIN_A);
    static const struct gpio_dt_spec pin_a = GPIO_DT_SPEC_GET(PIN_A, gpios);
    static const struct gpio_dt_spec pin_b = GPIO_DT_SPEC_GET(PIN_B, gpios);
    static const struct gpio_dt_spec pin_c = GPIO_DT_SPEC_GET(PIN_C, gpios);
    static const struct gpio_dt_spec pin_d = GPIO_DT_SPEC_GET(PIN_D, gpios);

    gpio_pin_configure_dt(&pin_a, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&pin_b, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&pin_c, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&pin_d, GPIO_OUTPUT_ACTIVE);

    //gpio_pin_set_raw(pin_a.port, 10, 1);
/*     k_msleep(5000);
    int val = gpio_pin_set(pin_a.port, 10, 1);
    printk("pin: %p\n", &pin_a);
    printk("config: %d\n", val1);
    printk("toggle: %d", val);
 */
    
    uint32_t steps = 0;
    uint8_t i = 0;
    int64_t start = k_uptime_get();
    while(1) {
        int64_t now = k_uptime_get();
        k_msleep(1);
        if ((i < 7) && ((now - start) < 60000)) {

            //printk("\n\nValore di i: %d\n", i);
            if(half_step_lookup_table[i][0] == 1) {
                gpio_pin_set(pin_a.port, 10, 1);
                //printk("%d-", 1);
            } else {
                gpio_pin_set(pin_a.port, 10, 0);
                //printk("%d-", 0);
            }

            if(half_step_lookup_table[i][1] == 1) {
                gpio_pin_set(pin_b.port, 11, 1);
                //printk("-%d-", 1);
            } else {
                gpio_pin_set(pin_b.port, 11, 0);
                //printk("-%d-", 0);
            }

            if(half_step_lookup_table[i][2] == 1) {
                gpio_pin_set(pin_c.port, 12, 1);
                //printk("-%d-", 1);
            } else {
                gpio_pin_set(pin_c.port, 12, 0);
                //printk("-%d-", 0);
            }

            if(half_step_lookup_table[i][3] == 1) {
                gpio_pin_set(pin_d.port, 13, 1);
                //printk("-%d", 1);
            } else {
                gpio_pin_set(pin_d.port, 13, 0);
                //printk("-%d", 0);
            }
            i++;
        } else {
            i = 0;
        }
        steps++;
        //int64_t time = (now - start) / 1000;
        printk("Steps: %u", steps);
        //LOG_INF("Steps: %u", steps);
    }
    return 0;
}