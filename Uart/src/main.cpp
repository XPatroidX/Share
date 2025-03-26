#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include "zephyr/drivers/i2c.h"
#include "drivers/lcd1602.hpp"
#include <string>


#define LCD     DT_NODELABEL(i2c0)

int main()
{
    uint8_t test [] = {0x2c, 0x8c, 0xEc, 0xc, 0x6c, 0x5d, 0x7d};
    const struct device *lcd_dev = DEVICE_DT_GET(LCD);

    if(lcd_dev == NULL || !device_is_ready(lcd_dev))
    {
        return 0;
    }
    
    char str[] = "I PIN 12 e 13 funzionano";
    char* s = str;
    int size = sizeof(str)/sizeof(char);
    

    lcdInit(lcd_dev);
    lcdClear(lcd_dev);
/*     lcdSendString(lcd_dev, str);
    k_msleep(1000);
    lcdClear(lcd_dev);
    k_msleep(1000);
    char str1[] = "PROVA";
    lcdClear(lcd_dev);
    lcdPutCur(lcd_dev, 0, 0);
    lcdSendString(lcd_dev, str1); */
/*     lcdPutCur(lcd_dev, 0, 0);
    lcdSendString(lcd_dev, str); */
    lcdPutCur(lcd_dev, 0, 0);
    lcdSendString(lcd_dev, str);
    while(true)
    {
        lcdSendCommand(lcd_dev, 0x18);
        k_msleep(700);
    }
    
    return 0;    
}