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
#include <math.h>


LOG_MODULE_REGISTER(logging_blog, LOG_LEVEL_DBG);

#define HMC                DT_NODELABEL(i2c0)
#define ADDRESS            0x0D

uint16_t heading (double x, double y)
{
    double rad = atan2(y, x);
    int16_t deg = (180*rad) / 3.14;
    if (deg < 0)
        deg += 360;
    return deg;
}

int main() 
{
    const struct device* dev = DEVICE_DT_GET(HMC);
    device_is_ready(dev);
    //struct sensor_value magn[3];
    i2c_reg_write_byte(dev, ADDRESS, 0x0B, 0x01);
    k_msleep(100);
    i2c_reg_write_byte(dev, ADDRESS, 0x09, 0x1D);
    i2c_reg_write_byte(dev, ADDRESS, 0x0A, 0x00);

    k_msleep(5000);
    while (1) {
        uint8_t data [6] ={0,0,0,0,0,0};
        //uint8_t wdata[2] ={0x1A, 0x00};
        //int i = i2c_burst_read(dev, ADDRESS, 0x00, &data[0], 1);
        uint8_t status = 0;
        i2c_reg_read_byte(dev, ADDRESS, 0x06, &status);
        if(status & 0x01)
        {
            //int i = i2c_read(dev,data, 6, 0x0d);
            int i = i2c_reg_read_byte(dev, ADDRESS, 0x00, &data[0]);
            i += i2c_reg_read_byte(dev, ADDRESS, 0x01, &data[1]);
            i += i2c_reg_read_byte(dev, ADDRESS, 0x02, &data[2]);
            i += i2c_reg_read_byte(dev, ADDRESS, 0x03, &data[3]);
            i += i2c_reg_read_byte(dev, ADDRESS, 0x04, &data[4]);
            i += i2c_reg_read_byte(dev, ADDRESS, 0x05, &data[5]);
    
            int16_t x = ((data[1] << 8) | data[0]);
            int16_t y = ((data[3] << 8) | data[2]);
            int16_t z = ((data[5] << 8) | data[4]);
            if (x > 32767) 
                x -= 65536;
            if (y > 32767) 
                y -= 65536;
            if (z > 32767) 
                z -= 65536;
            uint16_t deg = heading(x, y);
            //printk("Magnetic field: X:%d; Y:%d; Z:%d, heading:%d, success:%d\n", x, y, z, deg, i);
            printk("%d;%d;%d\n", x, y, z);
            k_msleep(500);
        }
        else
        {
            printk("Error: Data not rady\n");
        }
    }
    return 0;
}