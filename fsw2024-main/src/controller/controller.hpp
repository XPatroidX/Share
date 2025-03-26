/** Sapienza Space Team
 * @author Niccolò Gatti 
 * @author Vittorio De Lucia
 * @date 08/06/2024
 * @copyright 2024 Copyright (c) | Sapienza Space Team, all rights reserved.
 */

#ifndef PAYLOAD_CONTROLLER_HPP
#define PAYLOAD_CONTROLLER_HPP

#include <zephyr/kernel.h>
#include "drivers/bme280/bme280.hpp"
#include "newsm/sm.hpp"
#include "telemetry/service.hpp"

namespace payload::controller {
    class Controller {
        public:
            explicit Controller(newsm::NewStateMachine& new_state_machine, telemetry::TelemetryService& telemetry, drivers::bme280::BME280Sensor& pressure_sensor);
            ~Controller();
            void run();
            static void update(void* instance, void *, void *);
            static void smRun(void* instance, void *, void *);
            static void kalmanEntry(void* instance, void* b, void* c);

            newsm::NewStateMachine& new_state_machine_;
            telemetry::TelemetryService& telemetry_;
        private:
            struct k_thread sm_thread {};
            struct k_thread controller_thread {};
            struct k_thread kalman_thread {};

            double filtered_altitude_ {};
            double filtered_velocity_ {};
            double filtered_acceleration_ {};
            void kalman();

            drivers::bme280::BME280Sensor& pressure_sensor_;
    };
}

#endif