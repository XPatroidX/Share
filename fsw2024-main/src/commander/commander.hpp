/** Sapienza Space Team
 * @author Niccolò Gatti 
 * @author Vittorio De Lucia
 * @date 08/06/2024
 * @copyright 2024 Copyright (c) | Sapienza Space Team, all rights reserved.
 */

#ifndef PAYLOAD_COMMANDER_HPP
#define PAYLOAD_COMMANDER_HPP

#include "common/common.hpp"
#include "drivers/pwm_servo/servo.hpp"

#include <zephyr/drivers/gpio.h>
#include "drivers/gpio/gpio.hpp"
#include "subsys/camera/camera.hpp"
#include "subsys/led/led.hpp"

#include <iostream>
#include <string>

namespace payload::commander {
    class Commander {
        public:
            explicit Commander(drivers::pwm_servo::Servo& parachuteDeployServo, drivers::pwm_servo::Servo& heatShieldDeployServo, drivers::pwm_servo::Servo& heatShieldReleaseServo, gpio_dt_spec buzzer_dev, subsys::Camera& camera_first, subsys::Camera& camera_bonus, subsys::Led& led);
            ~Commander();

            void deployParachute();
            void deployHeatShield();
            void releaseHeatShield();

            void resetParachute();
            void resetHeatShieldDeploy();
            void resetHeatShieldRelease();

            void recordVideoFirst();
            void stopRecordVideoFirst();
            void recordVideoBonus();
            void stopRecordVideoBonus();

            void singBuzzer();
            void singBuzzerLanded();
            void shutUpBuzzer();

            void setActualState(state newState);

            static void handleMechanism(void* instance, void *, void *);
            static void blinkLed(void* instance, void *, void *);

            bool parachuteDeploy = false;
            bool heatShieldDeploy = false;
            bool heatShieldRelease = false;

            bool parachuteDeployReset = false;
            bool heatShieldDeployReset = false;
            bool heatShieldReleaseReset = false;

            bool videoRecordFirst = false;
            bool stopVideoRecordFirst = false;
            bool videoRecordBonus = false;
            bool stopVideoRecordBonus = false;
            
            bool goBuzzer = false;
            bool goBuzzerLanded = false;

        private:
            drivers::pwm_servo::Servo& parachuteDeployServo_;
            drivers::pwm_servo::Servo& heatShieldDeployServo_;
            drivers::pwm_servo::Servo& heatShieldReleaseServo_;
            gpio_dt_spec   buzzer_;
            subsys::Camera& camera_first_;
            subsys::Camera& camera_bonus_;
            subsys::Led& led_;

            bool initializeExecuted = false;
            bool launchWaitExecuted = false;
            bool ascentExecuted = false;
            bool cansatDescentExecuted = false;
            bool pcDeployExecuted = false;
            bool landedExecuted = false;

            bool changeState = false;
            state actualState;
            
            struct k_thread commander_thread {};
            struct k_thread led_blink_thread {};
    };
}

#endif