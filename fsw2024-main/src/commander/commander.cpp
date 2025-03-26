/** Sapienza Space Team
 * @author Niccolò Gatti 
 * @author Vittorio De Lucia
 * @date 08/06/2024
 * @copyright 2024 Copyright (c) | Sapienza Space Team, all rights reserved.
 */

#include <zephyr/kernel.h>
#include <zephyr/smf.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>

#include "commander.hpp"
#include "drivers/gpio/gpio.hpp"
#include "subsys/camera/camera.hpp"
#include "subsys/led/led.hpp"

#include <stdio.h>
#include <iostream>
#include <string>

#define PRIORITY 7
#define COMMANDER_STACK_SIZE 1024
#define LED_STACK_SIZE 1024

LOG_MODULE_REGISTER(COMMANDER, LOG_LEVEL_DBG);

K_THREAD_STACK_DEFINE(commander_stack_area, COMMANDER_STACK_SIZE);
K_THREAD_STACK_DEFINE(led_stack_area, LED_STACK_SIZE);

namespace payload::commander {

    Commander::Commander(drivers::pwm_servo::Servo& parachuteDeployServo, drivers::pwm_servo::Servo& heatShieldDeployServo, drivers::pwm_servo::Servo& heatShieldReleaseServo, gpio_dt_spec buzzer_dev, subsys::Camera& camera_first, subsys::Camera& camera_bonus, subsys::Led& led): parachuteDeployServo_ {parachuteDeployServo}, heatShieldDeployServo_ {heatShieldDeployServo}, heatShieldReleaseServo_ {heatShieldReleaseServo}, buzzer_ {buzzer_dev}, camera_first_ {camera_first}, camera_bonus_ {camera_bonus}, led_ {led} {
        k_thread_create(&commander_thread, commander_stack_area, K_THREAD_STACK_SIZEOF(commander_stack_area), handleMechanism, this, NULL, NULL, PRIORITY, 0, K_NO_WAIT);
        k_thread_name_set(&commander_thread, "commander_thread");
        k_thread_start(&commander_thread);

        k_thread_create(&led_blink_thread, led_stack_area, K_THREAD_STACK_SIZEOF(led_stack_area), blinkLed, this, NULL, NULL, PRIORITY, 0, K_NO_WAIT);
        k_thread_name_set(&led_blink_thread, "led_thread");
        k_thread_start(&led_blink_thread);
    };

    void Commander::deployParachute() {
        parachuteDeployServo_.set_pulse(parachuteDeployServo_.dev_.max_pulse);
        k_msleep(1000);
        parachuteDeployServo_.set_pulse(0);
    }

    void Commander::deployHeatShield() {
        heatShieldDeployServo_.set_pulse(heatShieldDeployServo_.dev_.max_pulse);
        k_msleep(1000);
        heatShieldDeployServo_.set_pulse(0);
    }

    void Commander::releaseHeatShield() {
        heatShieldReleaseServo_.set_pulse(heatShieldReleaseServo_.dev_.min_pulse);
        k_msleep(1000);
        heatShieldReleaseServo_.set_pulse(0);
    }

    void Commander::resetParachute() {
        printk("Devo resettare il paracadute\n");
        parachuteDeployServo_.set_pulse(parachuteDeployServo_.dev_.min_pulse);
        k_msleep(1000);
        parachuteDeployServo_.set_pulse(0);
    }

    void Commander::resetHeatShieldDeploy() {
        printk("Devo resettare l'hs deploy\n");
        heatShieldDeployServo_.set_pulse(heatShieldDeployServo_.dev_.min_pulse);
        k_msleep(1000);
        heatShieldDeployServo_.set_pulse(0);
    }

    void Commander::resetHeatShieldRelease() {
        printk("Devo resettare l'hs release\n");
        heatShieldReleaseServo_.set_pulse(heatShieldReleaseServo_.dev_.max_pulse);
        k_msleep(1000);
        heatShieldReleaseServo_.set_pulse(0);
    }

    void Commander::recordVideoFirst() {
        camera_first_.startRecording();
    }

    void Commander::stopRecordVideoFirst() {
        camera_first_.stopRecording();
    }

    void Commander::recordVideoBonus() {
        camera_bonus_.startRecording();
    }

    void Commander::stopRecordVideoBonus() {
        camera_bonus_.stopRecording();
    }

    void Commander::singBuzzer() {
        gpio_pin_configure_dt(&buzzer_, GPIO_OUTPUT_ACTIVE);
    }

    void Commander::singBuzzerLanded() {
        while(1){
            gpio_pin_configure_dt(&buzzer_, GPIO_OUTPUT_ACTIVE);
            k_msleep(1000);
            gpio_pin_configure_dt(&buzzer_, GPIO_OUTPUT_INACTIVE);
            k_msleep(2000);
        }
    }

    void Commander::shutUpBuzzer() {
        gpio_pin_configure_dt(&buzzer_, GPIO_OUTPUT_INACTIVE);
    }

    void Commander::setActualState(state newState){
        actualState = newState;
    }

    void Commander::blinkLed(void* instance, void *, void *){
        auto commander = reinterpret_cast<Commander*>(instance);
        if (!commander) return;
        // TO-DO: Test combinations of fast/slow blink (120/500)
        int fast_blink = 150; /* fast_blink -> 0*/
        int slow_blink = 500; /* slow_blink -> 1*/
        while(1){
            // S1 -> 000
            if(commander->actualState == S1){
                commander->led_.blink(fast_blink);
                k_msleep(fast_blink);
                commander->led_.blink(fast_blink);
                k_msleep(fast_blink);
                commander->led_.blink(fast_blink);
            }
            // S2 -> 001
            else if(commander->actualState == S2){
                commander->led_.blink(fast_blink);
                k_msleep(fast_blink);
                commander->led_.blink(fast_blink);
                k_msleep(fast_blink);
                commander->led_.blink(slow_blink);
            }
            // S3 -> 010
            else if(commander->actualState == S3){
                commander->led_.blink(fast_blink);
                k_msleep(fast_blink);
                commander->led_.blink(slow_blink);
                k_msleep(fast_blink);
                commander->led_.blink(fast_blink);
            }
            // S4 -> 011
            else if(commander->actualState == S4){
                commander->led_.blink(fast_blink);
                k_msleep(fast_blink);
                commander->led_.blink(slow_blink);
                k_msleep(fast_blink);
                commander->led_.blink(slow_blink);
            }
            // S5 -> 100
            else if(commander->actualState == S5){
                commander->led_.blink(slow_blink);
                k_msleep(fast_blink);
                commander->led_.blink(fast_blink);
                k_msleep(fast_blink);
                commander->led_.blink(fast_blink);
            }
            // S6 -> 101
            else if(commander->actualState == S6){
                commander->led_.blink(slow_blink);
                k_msleep(fast_blink);
                commander->led_.blink(fast_blink);
                k_msleep(fast_blink);
                commander->led_.blink(slow_blink);
            }
            // S7 -> 110
            else if(commander->actualState == S7){
                commander->led_.blink(slow_blink);
                k_msleep(fast_blink);
                commander->led_.blink(slow_blink);
                k_msleep(fast_blink);
                commander->led_.blink(fast_blink);
            }
            // S8 -> 111
            else if(commander->actualState == S8){
                commander->led_.blink(slow_blink);
                k_msleep(fast_blink);
                commander->led_.blink(slow_blink);
                k_msleep(fast_blink);
                commander->led_.blink(slow_blink);
            }
            
            k_msleep(1100);
        }
    }

    void Commander::handleMechanism(void* instance, void *, void *) {
        auto commander = reinterpret_cast<Commander*>(instance);
        if (!commander) return;

        while(1) {
            if(commander->parachuteDeploy) {
                printf("Ho deployato parachute\n");
                commander->deployParachute();
                commander->parachuteDeploy = false;
            } 
            else if (commander -> heatShieldDeploy) {
                printf("Ho deployato heat shield\n");
                commander->deployHeatShield();
                commander->heatShieldDeploy = false;
            } 
            else if (commander -> heatShieldRelease) {
                printf("Ho rilasciato heat shield\n");
                commander->releaseHeatShield();
                commander->heatShieldRelease = false;
            } 
            else if (commander -> parachuteDeployReset) {
                printf("Ho resettato parachute\n");
                commander->resetParachute();
                commander->parachuteDeployReset = false;
            } 
            else if (commander -> heatShieldDeployReset) {
                printf("Ho resettato heat shield deploy\n");
                commander->resetHeatShieldDeploy();
                commander->heatShieldDeployReset = false;
            } 
            else if (commander -> heatShieldReleaseReset) {
                printf("Ho resettato heat shield release\n");
                commander->resetHeatShieldRelease();
                commander->heatShieldReleaseReset = false;
            } 
            if (commander -> goBuzzer) {
                commander -> singBuzzer();
            } 
            if (commander -> goBuzzerLanded) {
                commander -> singBuzzerLanded();
            } 
            if (commander -> videoRecordFirst) {
                commander -> recordVideoFirst();
                commander -> videoRecordFirst = false;
            } 
            if (commander -> stopVideoRecordFirst) {
                commander -> stopRecordVideoFirst();
                commander -> stopVideoRecordFirst = false;
            } 
            if (commander -> videoRecordBonus) {
                commander -> recordVideoBonus();
                commander -> videoRecordBonus = false;
            } 
            if (commander -> stopVideoRecordBonus) {
                commander -> stopRecordVideoBonus();
                commander -> stopVideoRecordBonus = false;
            }
            k_msleep(100);
        }
    }
}