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
#include <vector>


LOG_MODULE_REGISTER(logging_blog, LOG_LEVEL_DBG);

#define QMC                DT_NODELABEL(i2c0)
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



void starting_parameters(const struct device* dev, double* p)
{
    printk("Mag Calibration: Rotate device on the xy plane");
    k_msleep(3000);
    int16_t values[3] = {0, 0,0};
    int16_t mag_max[3] = {-32767, -32767, -32767};
    int16_t mag_min[3] = {32767, 32767,32767};
    for(int i = 0; i < 3000; i++)
    {
        read_data(dev, &values[0]);
        printk(".");
        for(int j = 0; j < 3; j++)
        {
            if(values[j] > mag_max[j])
                mag_max[j] = values[j];
            if(values[j] < mag_min[j])
                mag_min[j] = values[j];
        }
        printk("%d,%d\n", values[0], values[1]);
        k_msleep(20);
    }
    k_msleep(2000);

    p[0] = (mag_max[0] + mag_min[0]) / 2.; //x_c: coordianta del centro sulle ascisse
    p[1] = (mag_max[1] + mag_min[1]) / 2.; //y_c: coordinata del centro sulle ordinate

    double d = sqrt(pow((mag_max[0] - p[0]), 2) + pow((mag_max[1] - p [1]), 2));
    p[2] = (mag_max[0] - p[0]) / d; //cos(alpha): coseno dell'angolo tra il semiasse x dell'ellisse e le ascisse
    p[3] = (mag_max[1] - p[1]) / d; //sin(alpha): seno dell'angolo tra il semiasse y dell'ellisse e le ordinate

    p[4] = (mag_max[0] - mag_min[0]) / 2.; //b: semiasse delle x
    p[5] = (mag_max[1] - mag_min[1]) / 2.; //a: semiasse delle y

    k_msleep(2000);
    printk("\n\nInitial polar parameters: %f, %f, %f, %f, %f, %f\n\n", p[0], p[1], p[2], p[3], p[4], p[5]);
    
    printk("End of calibration\n");
}

void calc_new_param(double* p, double* new_p, double cos, double sin){
    new_p[0] = p[0]*pow(cos, 2) + p[2]*pow(sin, 2) + p[1]*sin*cos;
    new_p[1] = 0;
    new_p[2] = p[0]*pow(sin, 2) + p[2]*pow(cos, 2) - p[1]*cos*sin;
    new_p[3] = p[3]*cos + p[4]*sin;
    new_p[4] = -p[3]*sin + p[4]*cos;
    new_p[5] = p[5] - (pow(p[3], 2)/(4*p[0])) - (pow(p[4], 2)/(4*p[2]));
    printk("\nNew cartesian parameters: %f, %f, %f, %f, %f, %f\n", new_p[0], new_p[1], new_p[2], new_p[3], new_p[4], new_p[5]);
}

void polar_to_cartesian(double* p, double* c)
{
    c[0] = pow(p[4]*p[3], 2) + pow(p[5]*p[2], 2);
    c[1] = 2*(pow(p[5], 2) - pow(p[4], 2))*p[2]*p[3];
    c[2] = pow(p[4]*p[2], 2) + pow(p[5]*p[3], 2);
    c[3] = -2*c[0]*p[0] - c[1]*p[1];
    c[4] = -c[1]*p[0] -2*c[2]*p[1];
    c[5] = c[0]*pow(p[0], 2) + c[1]*p[0]*p[1] + c[2]*pow(p[1], 2) - pow(p[4]*p[5], 2);
}

void data_circularization(int16_t* data, int16_t* c_data, double* new_p, double cos, double sin)
{
    double rotation[2][2] = {{cos, sin}, {-sin, cos}};
    double offset[2] = {new_p[3]/(2*new_p[0]), new_p[4]/(2*new_p[2])};
    double scaling_factor = sqrt(new_p[0]/new_p[2]);

    c_data[0] = rotation[0][1]*data[0] + rotation[0][1]*data[1] + offset[0];
    c_data[1] = scaling_factor*(rotation[1][1]*data[0] + rotation[1][1]*data[1] + offset[1]);
}

int main() 
{
    	// Configure to set Console output to USB Serial
	const struct device *usb_device = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

    // Check if USB can be initialised, bails out if fail is returned
	if (usb_enable(NULL) != 0) {
		return -1;
	}

    const struct device* dev = DEVICE_DT_GET(QMC);
    k_msleep(4000);
    if(device_is_ready(dev))
        printk("Funziona\n");
    else
        printk("Non funziona\n");
    //struct sensor_value magn[3];
    i2c_reg_write_byte(dev, ADDRESS, 0x0B, 0x01);
    k_msleep(100);
    i2c_reg_write_byte(dev, ADDRESS, 0x09, 0x19);
    i2c_reg_write_byte(dev, ADDRESS, 0x0A, 0x00);



    int16_t values[3] = {0,0,0};
    int16_t cal_values[3]= {0,0,0};

    k_msleep(5000);

    double i_poalr_param[6] = {1,1,1,1,1,1};
    double i_cart_param[6] = {0,0,0,0,0,0};
    double n_cart_param[6] = {0,0,0,0,0,0};
    starting_parameters(dev, i_poalr_param);
    polar_to_cartesian(i_poalr_param, i_cart_param);
    calc_new_param(i_cart_param, n_cart_param, i_poalr_param[2], i_poalr_param[3]);

    while (1) {
        read_data(dev, &values[0]);
        for(int i = 0; i < 3; i++) 
        {
            data_circularization(values, cal_values, n_cart_param, i_poalr_param[2], i_poalr_param[3]);
        }
        uint16_t deg = heading(cal_values[0], cal_values[1]);
        printk("%d,%d,%d, angle: %d\n", cal_values[0], cal_values[1], cal_values[2], deg);
        k_msleep(500);
    }
    return 0;
}