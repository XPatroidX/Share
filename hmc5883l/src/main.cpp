#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>
#include "drivers/hmc5883l.c"


LOG_MODULE_REGISTER(logging_blog, LOG_LEVEL_DBG);

#define HMC                DT_NODELABEL(hmc5883l)

int main() 
{
    const struct device* dev = DEVICE_DT_GET(HMC);
    struct sensor_value magn[3];
    //gpio_pin_set_raw(pin_a.port, 10, 1);
/*     k_msleep(5000);
    int val = gpio_pin_set(pin_a.port, 10, 1);
    printk("pin: %p\n", &pin_a);
    printk("config: %d\n", val1);
    printk("toggle: %d", val);
 */
    
/*     uint32_t steps = 1;
    int64_t start = k_uptime_get();

    while(steps < 4096) {
        uint8_t j = 0;
        while( j < 8) {
            gpio_pin_set(pin_a.port, 10, half_step_lookup_table[j][0]);
            gpio_pin_set(pin_b.port, 11, half_step_lookup_table[j][1]);
            gpio_pin_set(pin_c.port, 12, half_step_lookup_table[j][2]);
            gpio_pin_set(pin_d.port, 13, half_step_lookup_table[j][3]);
            //printk("j: %d, steps: %d\n", j, steps);
            j++;
            steps++;
            k_usleep(950);
            }
        }

    int64_t now = k_uptime_get();
    int64_t time = now - start;
    double vel = (360.0 / time) * 1000;
    string str = to_string(vel);
    printk("velocità: %d; ", (int) vel);
    //printk ("steps: %u\n", steps - 1); */
    k_msleep(5000);
    while (1) {
/*         printk("%d\n", device_is_ready(dev));
        printk("%d\n", sensor_sample_fetch(dev));
        printk( "%d\n", sensor_channel_get(dev, SENSOR_CHAN_MAGN_XYZ, magn));
        printk("acceleration: X:%d,%d; Y:%d,%d; Z:%d,%d\n",magn[0].val1, magn[0].val2, magn[1].val1, magn[1].val2, magn[2].val1, magn[2].val2);
        k_msleep(1000); */
        hmc5883l_sample_fetch(dev, SENSOR_CHAN_MAGN_XYZ);
        hmc5883l_channel_get(dev, SENSOR_CHAN_MAGN_XYZ, magn);
        printk("acceleration: X:%d,%d; Y:%d,%d; Z:%d,%d\n",magn[0].val1, magn[0].val2, magn[1].val1, magn[1].val2, magn[2].val1, magn[2].val2);
        k_msleep(1000);
    }
    return 0;
}