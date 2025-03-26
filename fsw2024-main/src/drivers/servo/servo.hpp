/**
 * @file servo.hpp
 * @author Lorenzo Thomas Contessa <lorenzocontessa.dev@gmail.com>
 * @date 10/03/2023
 * @copyright 2023 Copyright (c) | Sapienza Space Team, all rights reserved.
 */

#ifndef PAYLOAD_SERVO_HPP
#define PAYLOAD_SERVO_HPP

namespace drivers::servo {
    class Servo {
        public:
            explicit Servo(const struct device* dev, uint16_t min_pulse, uint16_t middle_pulse, uint16_t max_pulse): pwm_dev {dev}, _min_pulse {min_pulse}, _middle_pulse {middle_pulse}, _max_pulse {max_pulse} { pulse_width = _min_pulse; }
            ~Servo();
            void setServoPosition(uint8_t dir);
        private:
            uint16_t _min_pulse;
            uint16_t _middle_pulse;
            uint16_t _max_pulse;
            const struct device* pwm_dev;

            uint32_t pulse_width;
            int move();
    };
}

#endif