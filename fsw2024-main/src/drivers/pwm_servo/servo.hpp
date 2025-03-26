//
// Created by varkrid on 06/06/22.
//

#ifndef CSFW_SERVO_HPP
#define CSFW_SERVO_HPP

#include <zephyr/drivers/pwm.h>

namespace drivers::pwm_servo
{

  struct servo_t
  {
      pwm_dt_spec dev;
      uint32_t    min_pulse;
      uint32_t    max_pulse;
  };

  class Servo
  {
    public:
      explicit Servo(const servo_t& dev): dev_ {dev} {}
      int set_pulse(uint32_t pulse_length)
      {
        return pwm_set_pulse_dt(&(dev_.dev), pulse_length);
      }
      servo_t dev_;
  };

}// namespace drivers::pwm_servo

#endif// CSFW_SERVO_HPP
