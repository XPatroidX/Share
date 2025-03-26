/** Sapienza Space Team
 * @author Niccolò Gatti 
 * @author Vittorio De Lucia
 * @date 08/06/2024
 * @copyright 2024 Copyright (c) | Sapienza Space Team, all rights reserved.
 */

#include <stdio.h>
#include <iostream>
#include <string>

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "config.hpp"
#include "common/common.hpp"

#include "drivers/pwm_servo/servo.hpp"
#include "drivers/gpio/gpio.hpp"
#include "drivers/bme280/bme280.hpp"
#include "drivers/mpu6050/mpu6050.hpp"
#include "drivers/adc/adc.hpp"
#include "subsys/camera/camera.hpp"
#include "subsys/led/led.hpp"
#include "newsm/sm.hpp"
#include "commander/commander.hpp"
#include "controller/controller.hpp"
#include "telemetry/commands.hpp"
#include "telemetry/service.hpp"

#define PARACHUTE_NODE            DT_NODELABEL(parachute_servo)
#define HEATSHIELD_NODE           DT_NODELABEL(heatshield_servo)
#define HEATSHIELDRELEASE_NODE    DT_NODELABEL(heatshieldrelease_servo)

#define XBEE_UART_NODE            DT_CHOSEN(cansat_xbee)
#define NEO_SERIAL                DT_NODELABEL(neom8)
#define BMENODE                   DT_NODELABEL(bme280)
   
#define BUZZER_NODE               DT_NODELABEL(buzzer)
#define CAMERA_FIRST_NODE         DT_NODELABEL(camera_first)
#define CAMERA_BONUS_NODE         DT_NODELABEL(camera_bonus)
#define LED_NODE                  DT_NODELABEL(led)

using namespace payload;

int main() {
   // TO-DO: Remove all print on launch day!!!
   printk("HELLO WORLD!\n");
    
   static const drivers::pwm_servo::servo_t parachute_servo
   = {.dev       = PWM_DT_SPEC_GET(PARACHUTE_NODE),
      .min_pulse = DT_PROP(PARACHUTE_NODE, min_pulse),
      .max_pulse = DT_PROP(PARACHUTE_NODE, max_pulse)};

   static const drivers::pwm_servo::servo_t heatshield_servo
   = {.dev       = PWM_DT_SPEC_GET(HEATSHIELD_NODE),
      .min_pulse = DT_PROP(HEATSHIELD_NODE, min_pulse),
      .max_pulse = DT_PROP(HEATSHIELD_NODE, max_pulse)};

   static const drivers::pwm_servo::servo_t heatshieldrelease_servo
   = {.dev       = PWM_DT_SPEC_GET(HEATSHIELDRELEASE_NODE),
      .min_pulse = DT_PROP(HEATSHIELDRELEASE_NODE, min_pulse),
      .max_pulse = DT_PROP(HEATSHIELDRELEASE_NODE, max_pulse)};

   static const gpio_dt_spec buzzer_dev = GPIO_DT_SPEC_GET(BUZZER_NODE, gpios);
   static const gpio_dt_spec camera_first_dev = GPIO_DT_SPEC_GET(CAMERA_FIRST_NODE, gpios);
   static const gpio_dt_spec camera_bonus_dev = GPIO_DT_SPEC_GET(CAMERA_BONUS_NODE, gpios);
   static const gpio_dt_spec led_dev = GPIO_DT_SPEC_GET(LED_NODE, gpios);

   const struct device *bme280_dev = DEVICE_DT_GET(BMENODE);
   const struct device* mpu6050_dev = DEVICE_DT_GET_ONE(invensense_mpu6050);
   const struct device *xbee_node_dev = DEVICE_DT_GET(XBEE_UART_NODE);
   const struct device *gps_dev = DEVICE_DT_GET(NEO_SERIAL);

   drivers::pwm_servo::Servo parachute_servo_dev (parachute_servo);
   drivers::pwm_servo::Servo heatshield_servo_dev (heatshield_servo);
   drivers::pwm_servo::Servo heatshieldrelease_servo_dev (heatshieldrelease_servo);

   subsys::Camera camera_first(camera_first_dev);
   subsys::Camera camera_bonus(camera_bonus_dev);
   subsys::Led led(led_dev);

   drivers::bme280::BME280Sensor bme280(bme280_dev);
   drivers::mpu6050::MPU6050Sensor mpu6050(mpu6050_dev);
   drivers::adc::Adc adc;
   drivers::xbee::Xbee xbee(xbee_node_dev);
   drivers::sdcard::SDCard sd;

   commander::Commander commander(parachute_servo_dev, heatshield_servo_dev, heatshieldrelease_servo_dev, buzzer_dev, camera_first, camera_bonus, led);
   telemetry::TelemetryService telemetry(xbee, bme280, mpu6050, adc, sd, gps_dev);

   newsm::NewStateMachine newStateMachine(commander);
   controller::Controller controller(newStateMachine, telemetry, bme280);

   telemetry::CommandsHandler commands(xbee, controller, commander);

   commander.singBuzzer();
   k_msleep(200);
   commander.shutUpBuzzer();

   newStateMachine.setState(common::longStateToState(LAUNCH_WAIT));

   // IMPORTANT ON LAUNCH DAY!!!
   // TO-DO: On launch day telemetry shall start OFF and will be started by the Ground Station

   // telemetry.startTelemetry();
   //RESET SERVO
   k_msleep(2000);
   commander.resetParachute();
   k_msleep(1000);
   commander.resetHeatShieldDeploy();
   k_msleep(1000);
   commander.resetHeatShieldRelease();

   commander.singBuzzer();
   k_msleep(60);
   commander.shutUpBuzzer();
   k_msleep(60);
   commander.singBuzzer();
   k_msleep(60);
   commander.shutUpBuzzer();
   
   bool camera_first_activated = true;
   bool camera_first_stop = true;

   bool camera_bonus_activated = true;
   bool camera_bonus_stop = true;

   while (1) {
      if(newStateMachine.getActualState()==common::longStateToState(ASCENT) && camera_first_activated){
         commander.recordVideoFirst();
         camera_first_activated = false;
      }
      if(newStateMachine.getActualState()==common::longStateToState(LANDED) && camera_first_stop){
         commander.stopRecordVideoFirst();
         camera_first_stop = false;
      }
      if(newStateMachine.getActualState()==common::longStateToState(HEAT_SHIELD_DEPLOY) && camera_bonus_activated){
         commander.recordVideoBonus();
         camera_bonus_activated = false;
      }
      if(newStateMachine.getActualState()==common::longStateToState(LANDED) && camera_bonus_stop){
         commander.stopRecordVideoBonus();
         camera_bonus_stop = false;
      }
      k_msleep(1000);
   }

   return 0;
}
