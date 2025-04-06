#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>
#include <cstdint>
#include "zephyr/drivers/i2c.h"
#include "zephyr/usb/usb_device.h"
#include <zephyr/drivers/uart.h>
#include <math.h>


LOG_MODULE_REGISTER(logging_blog, LOG_LEVEL_DBG);

#define HMC                DT_NODELABEL(i2c0)
#define ADDRESS            0x0D

BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart), "Console device is not ACM CDC UART device");


uint16_t heading (double x, double y)
{
    double rad = atan2(y, x);
    int16_t deg = (180*rad) / 3.14;
    if (deg < 0)
        deg += 360;
    return deg;
}

void read_data(const struct device* dev, int16_t* values){
    uint8_t data [6] ={0,0,0,0,0,0};
        uint8_t status = 0;
        i2c_reg_read_byte(dev, ADDRESS, 0x06, &status);
        if(status & 0x01)
        {
            int i = i2c_reg_read_byte(dev, ADDRESS, 0x00, &data[0]);
            i += i2c_reg_read_byte(dev, ADDRESS, 0x01, &data[1]);
            i += i2c_reg_read_byte(dev, ADDRESS, 0x02, &data[2]);
            i += i2c_reg_read_byte(dev, ADDRESS, 0x03, &data[3]);
            i += i2c_reg_read_byte(dev, ADDRESS, 0x04, &data[4]);
            i += i2c_reg_read_byte(dev, ADDRESS, 0x05, &data[5]);
    
            values[0] = ((data[1] << 8) | data[0]);
            values[1] = ((data[3] << 8) | data[2]);
            values[2] = ((data[5] << 8) | data[4]);
        }
        else
        {
            printk("Data not ready");
        }
}

int main() 
{
    	/* Configure to set Console output to USB Serial */ 
	const struct device *usb_device = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

    /* Check if USB can be initialised, bails out if fail is returned */
	if (usb_enable(NULL) != 0) {
		return -1;
	}

    const struct device* dev = DEVICE_DT_GET(HMC);
    i2c_reg_write_byte(dev, ADDRESS, 0x0B, 0x01);
    k_msleep(100);
    i2c_reg_write_byte(dev, ADDRESS, 0x09, 0x1D);
    i2c_reg_write_byte(dev, ADDRESS, 0x0A, 0x00);

    k_msleep(5000);

    int16_t values[3];
    while(1)
    {
        read_data(dev, values);
        printk("%d, %d, %d\n", values[0], values[1], values[2]);
        k_msleep(500);
    }

    return 0;
}