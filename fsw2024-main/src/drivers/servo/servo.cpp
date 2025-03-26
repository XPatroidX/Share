/**
 * @file servo.cpp
 * @author Lorenzo Thomas Contessa <lorenzocontessa.dev@gmail.com>
 * @date 10/03/2023
 * @copyright 2023 Copyright (c) | Sapienza Space Team, all rights reserved.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>

#include "servo.hpp"

#define NSEC_PER_SEC USEC_PER_SEC * 1000
#define PERIOD (NSEC_PER_SEC / 50U) // 20ms (50Hz)

namespace drivers::servo {
    Servo::~Servo() = default;

    void Servo::setServoPosition(uint8_t dir) {
        switch(dir) {
            case 1U:
                pulse_width = _middle_pulse;
                move();
                break;
            case 2U:
                pulse_width = _max_pulse;
                move();
                break;
            default:
                pulse_width = _min_pulse;
                move();
        }
    }

    int Servo::move() {
        int ret = 0;
        if(ret = pwm_set(pwm_dev, 2, PERIOD, pulse_width * 1000, 0)) {
			printk("PWM set failed: %d\n", ret);
			return ret;
		}

        return ret;
    }
}