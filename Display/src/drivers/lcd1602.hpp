#ifndef LCD_DRIVER_H_INCLUDED
#define LCD_DRIVER_H_INCLUDED

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include "zephyr/drivers/i2c.h"


void lcdClear(const struct device *const i2c_dev);
void lcdPutCur(const struct device *const i2c_dev, int row, int col);
void lcdInit (const struct device *const i2c_dev);
void lcdSendString(const struct device *const i2c_dev, char *str);
void lcdSendData(const struct device *const i2c_dev, char data);
void lcdSendCommand(const struct device *const i2c_dev, char cmd);

#endif