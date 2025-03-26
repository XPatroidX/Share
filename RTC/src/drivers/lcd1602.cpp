#include "lcd1602.hpp"

// LOG_MODULE_REGISTER(i2c_lcd);

#define SLAVE_ADDRESS_LCD 0x27

void lcdSendCommand(const struct device *const i2c_dev, char cmd){
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (cmd & 0xf0);
    data_l = ((cmd << 4) & 0xf0);
    data_t[0] = data_u | 0x0C; // en=1, rs=0
    data_t[1] = 0x08; // en=0
    data_t[2] = data_l | 0x0C; // en=1, rs=0
    data_t[3] = 0x08; // en=0
    int ret;
    ret = i2c_write(i2c_dev, data_t, 4, SLAVE_ADDRESS_LCD);
}

void lcdSendData(const struct device *const i2c_dev, char data){
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (data & 0xf0);
    data_l = ((data << 4) & 0xf0);
    data_t[0] = data_u | 0x0D; // en=1, rs=1
    data_t[1] = 0x08; // en=0
    data_t[2] = data_l | 0x0D; // en=1, rs=1
    data_t[3] = 0x08; // en=0
    i2c_write(i2c_dev, data_t, 4, SLAVE_ADDRESS_LCD);
}

void lcdClear(const struct device *const i2c_dev){
    lcdSendCommand(i2c_dev, 0x80);
    for (int i = 0; i < 70; i++)
    {
        lcdSendData(i2c_dev, ' ');
    }
}

void lcdPutCur(const struct device *const i2c_dev, int row, int col){
    switch (row){
        case 0:
            col |= 0x80;
        break;
        case 1:
            col |= 0xC0;
        break;
    }
    lcdSendCommand(i2c_dev, col);
}

void lcdInit (const struct device *const i2c_dev){
    // 4 bit initialization
    k_msleep(50); // wait for >40ms
    lcdSendCommand(i2c_dev, 0x30);
    k_msleep(5); // wait for >4.1ms
    lcdSendCommand(i2c_dev, 0x30);
    k_usleep(100); // wait for >100us
    lcdSendCommand(i2c_dev, 0x30);
    k_msleep(10);
    lcdSendCommand(i2c_dev, 0x20); // 4bit mode
    k_usleep(10);

    // display initialization
    lcdSendCommand(i2c_dev, 0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
    k_msleep(1);
    lcdSendCommand(i2c_dev, 0x08); // Display on/off control --> D=0,C=0, B=0 ---> display off
    k_msleep(1);
    lcdSendCommand(i2c_dev, 0x01); // clear display
    k_msleep(1);
    k_msleep(1);
    lcdSendCommand(i2c_dev, 0x06); // Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
    k_msleep(1);
    lcdSendCommand(i2c_dev, 0x0C); // Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void lcdSendString(const struct device *const i2c_dev, const char *str){
    while (*str)
        lcdSendData(i2c_dev, *str++);
}