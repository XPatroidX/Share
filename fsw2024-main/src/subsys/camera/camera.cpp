/** Sapienza Space Team
 * @author Niccolò Gatti 
 * @author Vittorio De Lucia
 * @date 08/06/2024
 * @copyright 2024 Copyright (c) | Sapienza Space Team, all rights reserved.
 */

#include "camera.hpp"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#define HIGH_TIME 700000

LOG_MODULE_REGISTER(camera, LOG_LEVEL_DBG);

namespace payload::subsys {
    Camera::Camera(const gpio_dt_spec& dev): drivers::gpio::Gpio(dev) {
        auto ret = configure(GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to set %s to GPIO_OUTPUT_ACTIVE", dev_.port->name);
        }
    }

    void Camera::startRecording() {
        // printk("Camera recording!\n");
        configure(GPIO_OUTPUT_ACTIVE);
        toggle_pin();
        k_busy_wait(HIGH_TIME);
        toggle_pin();
        started_ = true;
    }
    
    void Camera::stopRecording() {
        // printk("Camera stopped recording!\n");
        configure(GPIO_OUTPUT_ACTIVE);
        toggle_pin();
        k_busy_wait(HIGH_TIME);
        toggle_pin();
        started_ = false; 
    }

    void Camera::strobe() {
        toggle_pin();
        k_busy_wait(800000);
        toggle_pin();
        k_busy_wait(800000);
    }

    bool Camera::is_started() const {
        return started_;
    }
}