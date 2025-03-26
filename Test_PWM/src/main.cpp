#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include "drivers/lcd1602.hpp"
#include <cstdint>
#include <cstdio>
#include <string>

#define LED		DT_NODELABEL(ledpwm)
#define LCD     DT_NODELABEL(i2c0)

const struct pwm_dt_spec led = PWM_DT_SPEC_GET(LED);
//const struct device *lcd_dev = DEVICE_DT_GET(LCD);


#define NUM_STEPS	50U
#define SLEEP_MSEC	50U

int main()
{
	uint32_t pulse_width = 0U;
	uint32_t step = led.period / NUM_STEPS;
	uint8_t dir = 1U;
	int ret;

	/* lcdInit(lcd_dev);
    lcdClear(lcd_dev);
	char chn[3];
	snprintf(chn, 3, "%d", led.channel);
	lcdPutCur(lcd_dev, 0, 0);
	lcdSendString(lcd_dev, chn); */

	printk("PWM-based LED fade\n");

	if (!pwm_is_ready_dt(&led)) {
		printk("Error: PWM device %s is not ready\n",
		       led.dev->name);
		return 0;
	}

	while (1) {
/* 		ret = pwm_set_pulse_dt(&led, pulse_width);
		if (ret) {
			printk("Error %d: failed to set pulse width\n", ret);
			return 0;
		}

		if (dir) {
			pulse_width += step;
			if (pulse_width >= led.period) {
				pulse_width = led.period - step;
				dir = 0U;
			}
		} else {
			if (pulse_width >= step) {
				pulse_width -= step;
			} else {
				pulse_width = step;
				dir = 1U;
			}
		}

		k_sleep(K_MSEC(SLEEP_MSEC)); */
		pwm_set_pulse_dt(&led, pulse_width);
	}
	return 0;
}
