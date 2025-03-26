#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include "zephyr/drivers/i2c.h"
#include "drivers/bme280/bme280.hpp"
#include <zephyr/drivers/sensor.h>
#include <string>


#define BME280     DT_NODELABEL(bme280)

int main()
{
    const struct device *bme280_dev = DEVICE_DT_GET(BME280);
    //drivers::bme280::BME280Sensor bme280(bme280_dev);
    k_msleep(5000);
    if (!device_is_ready(bme280_dev)) {
        printk("\nError: Device \"%s\" is not ready; "
            "check the driver initialization logs for errors.\n",
            bme280_dev->name);
    } else {
        printk("Found device \"%s\", getting sensor data\n", bme280_dev->name);
    }
    struct sensor_value press;
    struct sensor_value temp;
    if (sensor_sample_fetch(bme280_dev) < 0) {
        printk("Could not fetch data from \"%s\"\n", bme280_dev->name);
        return -1;
    }
    while (1) {
        sensor_sample_fetch(bme280_dev);
        sensor_channel_get(bme280_dev, SENSOR_CHAN_PRESS, &press);
        sensor_channel_get(bme280_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
        printk("\nTemperature: %d,%d    Pressure: %d,%d", temp.val1, temp.val2, press.val1, press.val2);
        k_msleep(300);
    }
/*     while(1) {
        printk("test");
    } */
    return 0;    
}