#include "zephyr/dt-bindings/input/input-event-codes.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <cerrno>
#include <cstdint>

using namespace std;

#define LED0_NODE       DT_ALIAS(led0)

#define BUTTON          DT_NODELABEL(button0)
#define BUTTON_DEV      DT_PHANDLE(BUTTON, gpios)
#define BUTTON_PIN      DT_PHA(BUTTON, gpios, pin)
#define BUTTON_FLAGS    DT_PHA(BUTTON, gpios, flags)
#define BUTTON_PRS      K_EVENT_DEFINE(my_event);

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

struct gpio_callback button_cb_data;

static void button_event_handler(const struct device *dev, struct gpio_callback *p_cb, uint32_t pins) {}

int main() 
{
    const struct device *button_dev = DEVICE_DT_GET(BUTTON_DEV);

    if(button_dev == NULL || !device_is_ready(button_dev))
    {
        //LOG_ERR("Could not get gpio device.");
        return -EIO;
    }

    int err = gpio_pin_configure(button_dev, BUTTON_PIN, BUTTON_FLAGS);
    if(err != 0)
    {
        //LOG_ERR("Could not configure secondary gpio");
        return err;
    }

    gpio_pin_interrupt_configure(button_dev, BUTTON_PIN, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&button_cb_data, button_event_handler, BIT(BUTTON_PIN));

    gpio_add_callback(button_dev, &button_cb_data);

    int ret;

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_HIGH);
	if (ret < 0) {
		return 0;
	}

    while(1)
    {
		/* If we have an LED, match its state to the button's. */
		int val = gpio_pin_get(button_dev, BUTTON_PIN);

		if (val > 0) {
			gpio_pin_toggle_dt(&led);
            k_msleep(200); 
		} 
    }
    return 0;
}