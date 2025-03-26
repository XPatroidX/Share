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
#include "drivers/mpu6050.hpp"
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>

using namespace std;

LOG_MODULE_REGISTER(logging_blog, LOG_LEVEL_DBG);

#define MPU                 DT_NODELABEL(mpu6050)

#define PIN_A               DT_NODELABEL(pina)
#define PIN_B               DT_NODELABEL(pinb)
#define PIN_C               DT_NODELABEL(pinc)
#define PIN_D               DT_NODELABEL(pind)
#define NUM_CONTROL_PINS    4
#define MAX_MICRO_STEP_RES  2

static const uint8_t
	half_step_lookup_table[NUM_CONTROL_PINS * MAX_MICRO_STEP_RES][NUM_CONTROL_PINS] = {
		{1u, 1u, 0u, 0u}, 
        {0u, 1u, 0u, 0u}, 
        {0u, 1u, 1u, 0u}, 
        {0u, 0u, 1u, 0u},
		{0u, 0u, 1u, 1u}, 
        {0u, 0u, 0u, 1u}, 
        {1u, 0u, 0u, 1u}, 
        {1u, 0u, 0u, 0u}};

static const uint8_t
    full_step_lookup_table[NUM_CONTROL_PINS][NUM_CONTROL_PINS] = {
		{1u, 1u, 0u, 0u}, 
        {0u, 1u, 1u, 0u}, 
		{0u, 0u, 1u, 1u}, 
        {1u, 0u, 0u, 1u}};

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

    const struct device* mpu6050_dev = DEVICE_DT_GET(MPU);
    drivers::MPU6050Sensor mpu6050(mpu6050_dev);

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
    while (1) {
        mpu6050.getAccel();
        mpu6050.getGyro();
        printk("acceleration: X:%d,%d; Y:%d,%d; Z:%d,%d\n", mpu6050.accel[0].val1, mpu6050.accel[0].val2, mpu6050.accel[1].val1, mpu6050.accel[1].val2, mpu6050.accel[2].val1, mpu6050.accel[2].val2);
        printk("gyro acceleration: X:%d,%d; Y:%d,%d; Z:%d,%d\n\n", mpu6050.gyro[0].val1, mpu6050.gyro[0].val2, mpu6050.gyro[1].val1, mpu6050.gyro[1].val2, mpu6050.gyro[2].val1, mpu6050.gyro[2].val2);
        k_msleep(50);
    }
    return 0;
}