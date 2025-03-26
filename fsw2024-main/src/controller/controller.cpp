/** Sapienza Space Team
 * @author Niccolò Gatti 
 * @author Vittorio De Lucia
 * @date 08/06/2024
 * @copyright 2024 Copyright (c) | Sapienza Space Team, all rights reserved.
 */

#include "controller.hpp"
#include <stdio.h>
#include <zephyr/logging/log.h>

#define PRIORITY 7

#define CONTROLLER_STACK_SIZE 1024
#define KALMAN_STACK_SIZE 1024
#define SM_STACK_SIZE 1024

LOG_MODULE_REGISTER(CONTROLLER, LOG_LEVEL_DBG);

K_THREAD_STACK_DEFINE(controller_stack_area, CONTROLLER_STACK_SIZE);
K_THREAD_STACK_DEFINE(kalman_stack_area, KALMAN_STACK_SIZE);
K_THREAD_STACK_DEFINE(sm_stack_area, SM_STACK_SIZE);

namespace payload::controller {
    Controller::Controller(newsm::NewStateMachine& new_state_machine, telemetry::TelemetryService& telemetry, drivers::bme280::BME280Sensor& pressure_sensor) : new_state_machine_ {new_state_machine}, telemetry_ {telemetry}, pressure_sensor_ {pressure_sensor} {
        k_thread_create(&controller_thread, controller_stack_area, K_THREAD_STACK_SIZEOF(controller_stack_area), update, this, NULL, NULL, PRIORITY, 0, K_NO_WAIT);
        k_thread_name_set(&controller_thread, "controller_thread");

        k_thread_create(&kalman_thread, kalman_stack_area, K_THREAD_STACK_SIZEOF(kalman_stack_area), kalmanEntry, this, NULL, NULL, PRIORITY, 0, K_NO_WAIT);
        k_thread_name_set(&kalman_thread, "kalman_thread");

        k_thread_create(&sm_thread, sm_stack_area, K_THREAD_STACK_SIZEOF(sm_stack_area), smRun, this, NULL, NULL, PRIORITY, 0, K_NO_WAIT);
        k_thread_name_set(&sm_thread, "sm_thread");
    }

    Controller::~Controller() = default;

    void Controller::smRun(void* instance, void *, void *) {
        auto controller = reinterpret_cast<Controller*>(instance);
        if (!controller) return;

        controller->new_state_machine_.execute();
    }

    void Controller::kalmanEntry(void* instance, void* b, void* c) {
        auto controller = reinterpret_cast<Controller*>(instance);
        if (!controller) return;

        controller->kalman();
    }

    void Controller::kalman() {
        constexpr double ALTITUDESIGMA     = 4.572;
        constexpr double ACCELERATIONSIGMA = 1.83;
        constexpr double MODELSIGMA        = 0.183;

        constexpr double altitude_variance     = ALTITUDESIGMA * ALTITUDESIGMA;
        constexpr double acceleration_variance = ACCELERATIONSIGMA * ACCELERATIONSIGMA;
        constexpr double model_variance        = MODELSIGMA * MODELSIGMA;

        int     i, j, k, notdone;
        double  alt_inovation, accel_inovation;
        int64_t time, last_time;
        double  accel, altitude;
        double  det;
        double  est[3]      = {0, 0, 0};
        double  estp[3]     = {0, 0, 0};
        double  pest[3][3]  = {{2, 0, 0}, {0, 9, 0}, {0, 0, 9}};
        double  pestp[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
        double  phi[3][3]   = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1.0}};
        double  phit[3][3]  = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1.0}};
        double  kgain[3][2] = {{0.01, 0.01}, {0.01, 0.01}, {0.01, 0.01}};
        double  lastkgain[3][2], dt;
        double  term[3][3];

        /* Initialize */

        altitude  = telemetry_.getAltitude();
        est[0]    = altitude;
        time      = k_uptime_get();
        dt        = 0.01;
        last_time = time;

        /*
        * Fill in state transition matrix and its transpose
        */
        phi[0][1]  = dt;
        phi[1][2]  = dt;
        phi[0][2]  = dt * dt / 2.0;
        phit[1][0] = dt;
        phit[2][1] = dt;
        phit[2][0] = dt * dt / 2.0;

        /* Compute the Kalman gain matrix. */
        for (i = 0; i <= 2; i++)
            for (j = 0; j <= 1; j++)
                lastkgain[i][j] = kgain[i][j];

        k = 0;

        while (true)
        {
            /* Propagate state covariance */
            term[0][0]  = phi[0][0] * pest[0][0] + phi[0][1] * pest[1][0] + phi[0][2] * pest[2][0];
            term[0][1]  = phi[0][0] * pest[0][1] + phi[0][1] * pest[1][1] + phi[0][2] * pest[2][1];
            term[0][2]  = phi[0][0] * pest[0][2] + phi[0][1] * pest[1][2] + phi[0][2] * pest[2][2];
            term[1][0]  = phi[1][0] * pest[0][0] + phi[1][1] * pest[1][0] + phi[1][2] * pest[2][0];
            term[1][1]  = phi[1][0] * pest[0][1] + phi[1][1] * pest[1][1] + phi[1][2] * pest[2][1];
            term[1][2]  = phi[1][0] * pest[0][2] + phi[1][1] * pest[1][2] + phi[1][2] * pest[2][2];
            term[2][0]  = phi[2][0] * pest[0][0] + phi[2][1] * pest[1][0] + phi[2][2] * pest[2][0];
            term[2][1]  = phi[2][0] * pest[0][1] + phi[2][1] * pest[1][1] + phi[2][2] * pest[2][1];
            term[2][2]  = phi[2][0] * pest[0][2] + phi[2][1] * pest[1][2] + phi[2][2] * pest[2][2];
            pestp[0][0] = term[0][0] * phit[0][0] + term[0][1] * phit[1][0] + term[0][2] * phit[2][0];
            pestp[0][1] = term[0][0] * phit[0][1] + term[0][1] * phit[1][1] + term[0][2] * phit[2][1];
            pestp[0][2] = term[0][0] * phit[0][2] + term[0][1] * phit[1][2] + term[0][2] * phit[2][2];
            pestp[1][0] = term[1][0] * phit[0][0] + term[1][1] * phit[1][0] + term[1][2] * phit[2][0];
            pestp[1][1] = term[1][0] * phit[0][1] + term[1][1] * phit[1][1] + term[1][2] * phit[2][1];
            pestp[1][2] = term[1][0] * phit[0][2] + term[1][1] * phit[1][2] + term[1][2] * phit[2][2];
            pestp[2][0] = term[2][0] * phit[0][0] + term[2][1] * phit[1][0] + term[2][2] * phit[2][0];
            pestp[2][1] = term[2][0] * phit[0][1] + term[2][1] * phit[1][1] + term[2][2] * phit[2][1];
            pestp[2][2] = term[2][0] * phit[0][2] + term[2][1] * phit[1][2] + term[2][2] * phit[2][2];

            pestp[2][2] = pestp[2][2] + model_variance;
            /*
            Calculate Kalman Gain
            */
            det = (pestp[0][0] + altitude_variance) * (pestp[2][2] + acceleration_variance)
                    - pestp[2][0] * pestp[0][2];
            kgain[0][0]
                = (pestp[0][0] * (pestp[2][2] + acceleration_variance) - pestp[0][2] * pestp[2][0]) / det;
            kgain[0][1]
                = (pestp[0][0] * (-pestp[0][2]) + pestp[0][2] * (pestp[0][0] + altitude_variance)) / det;
            kgain[1][0]
                = (pestp[1][0] * (pestp[2][2] + acceleration_variance) - pestp[1][2] * pestp[2][0]) / det;
            kgain[1][1]
                = (pestp[1][0] * (-pestp[0][2]) + pestp[1][2] * (pestp[0][0] + altitude_variance)) / det;
            kgain[2][0]
                = (pestp[2][0] * (pestp[2][2] + acceleration_variance) - pestp[2][2] * pestp[2][0]) / det;
            kgain[2][1]
                = (pestp[2][0] * (-pestp[0][2]) + pestp[2][2] * (pestp[0][0] + altitude_variance)) / det;
            pest[0][0] = pestp[0][0] * (1.0 - kgain[0][0]) - kgain[0][1] * pestp[2][0];
            pest[0][1] = pestp[0][1] * (1.0 - kgain[0][0]) - kgain[0][1] * pestp[2][1];
            pest[0][2] = pestp[0][2] * (1.0 - kgain[0][0]) - kgain[0][1] * pestp[2][2];
            pest[1][0] = pestp[0][0] * (-kgain[1][0]) + pestp[1][0] - kgain[1][1] * pestp[2][0];
            pest[1][1] = pestp[0][1] * (-kgain[1][0]) + pestp[1][1] - kgain[1][1] * pestp[2][1];
            pest[1][2] = pestp[0][2] * (-kgain[1][0]) + pestp[1][2] - kgain[1][1] * pestp[2][2];
            pest[2][0] = (1.0 - kgain[2][1]) * pestp[2][0] - kgain[2][0] * pestp[2][0];
            pest[2][1] = (1.0 - kgain[2][1]) * pestp[2][1] - kgain[2][0] * pestp[2][1];
            pest[2][2] = (1.0 - kgain[2][1]) * pestp[2][2] - kgain[2][0] * pestp[2][2];


            /* Check for convergence. Criteria is less than .001% change from last
            * time through the mill.
            */
            notdone = 0;
            k++;
            for (i = 0; i <= 2; i++)
                for (j = 0; j <= 1; j++)
                {
                    if ((kgain[i][j] - lastkgain[i][j]) / lastkgain[i][j] > 0.00001)
                        notdone++;
                    lastkgain[i][j] = kgain[i][j];
                }
            
            if (notdone)
                continue;
            else
                break;
        }

        /* Now run the Kalman filter on the data using previously
        * determined gains.
        */
        while (true)
        {
            time     = k_uptime_get();
            accel    = -telemetry_.getVerticalAcceleration();
            altitude = telemetry_.getAltitude();

            accel = (accel - config::G);

            if (last_time >= time)
            {
                LOG_ERR("Time does not increase");
                continue;
            }

            /* Compute the innovations */
            alt_inovation   = altitude - estp[0];
            accel_inovation = accel - estp[2];

            /* Experimental code to modify Mach transition pressure
            * disturbances.
            */
            if (std::abs(alt_inovation) > 100)
            {
                /* We have a large error in altitude. Now see how fast we are
                * going.
                */
                if (estp[1] > 273 && estp[1] < 366)
                {
                    /* Somewhere in the neighborhood of Mach 1. Now check to
                    * see if we are slowing down.
                    */
                    //
                    if (estp[2] < 0)
                    {
                        /*
                        * OK, now what do we do? Assume that velocity and
                        * acceleration estimates are accurate. Adjust current
                        * altitude estimate to be the same as the measured
                        * altitude.
                        */
                        est[0]        = altitude;
                        alt_inovation = 0;
                    }
                }
            }

            /* Simple check for over-range on pressure measurement.
            * This is just hacked in based on a single data set. Actual
            * flight software needs something more sophisticated.
            */
            if (altitude > 11000)
                alt_inovation = 0;
            
            /* Propagate state */
            estp[0] = phi[0][0] * est[0] + phi[0][1] * est[1] + phi[0][2] * est[2];
            estp[1] = phi[1][0] * est[0] + phi[1][1] * est[1] + phi[1][2] * est[2];
            estp[2] = phi[2][0] * est[0] + phi[2][1] * est[1] + phi[2][2] * est[2];

            /*
            * Update state
            */
            est[0] = estp[0] + kgain[0][0] * alt_inovation + kgain[0][1] * accel_inovation;
            est[1] = estp[1] + kgain[1][0] * alt_inovation + kgain[1][1] * accel_inovation;
            est[2] = estp[2] + kgain[2][0] * alt_inovation + kgain[2][1] * accel_inovation;

            /*
            * Output
            */
            filtered_altitude_     = est[0];
            filtered_velocity_     = est[1];
            filtered_acceleration_ = est[2];
            last_time              = time;

            static auto print_time = last_time;
            if (last_time - print_time > 1000){
                print_time = last_time;
            }
            
            if (new_state_machine_.getActualState() == common::longStateToState(LAUNCH_WAIT) && filtered_velocity_ > config::ASCENT_VELOCITY) 
            {
                new_state_machine_.setState(common::longStateToState(ASCENT));
            } 
            else if (new_state_machine_.getActualState() == common::longStateToState(ASCENT) && filtered_velocity_ < 0.0f && filtered_altitude_ >= config::APOGEE) 
            {
                new_state_machine_.setState(common::longStateToState(CANSAT_DESCENT));
            } 
            else if (new_state_machine_.getActualState() == common::longStateToState(CANSAT_DESCENT)) 
            {
                k_msleep(2000);
                new_state_machine_.setState(common::longStateToState(HEAT_SHIELD_DEPLOY));
                telemetry_.setHeatShieldDeploy(telemetry::hs_deployed_t::DEPLOYED);
            } 
            else if (new_state_machine_.getActualState() == common::longStateToState(HEAT_SHIELD_DEPLOY) && filtered_altitude_ <= config::HEAT_SHIELD_RELEASE_ALTITUDE) 
            {
                k_msleep(1000);
                new_state_machine_.setState(common::longStateToState(HEAT_SHIELD_RELEASE));
            } 
            else if (new_state_machine_.getActualState() == common::longStateToState(HEAT_SHIELD_RELEASE)) 
            {
                k_msleep(1000);
                new_state_machine_.setState(common::longStateToState(PC_DEPLOY));
                telemetry_.setParachuteDeploy(telemetry::pc_deployed_t::DEPLOYED);
                telemetry_.pc_deploy_packet_number = telemetry_.packet_count_;
            } 
            else if (new_state_machine_.getActualState() == common::longStateToState(PC_DEPLOY) && 
                    (filtered_altitude_ < 20 || (telemetry_.packet_count_ - telemetry_.pc_deploy_packet_number) >= 40))
            {
                k_msleep(2000);
                new_state_machine_.setState(common::longStateToState(LANDED));
            }
            k_sleep(K_MSEC(10));
        }
    }

    void Controller::update(void* instance, void *, void *) {
        auto controller = reinterpret_cast<Controller*>(instance);
        if (!controller) return;

        while(1) {
            controller->telemetry_.setState(common::stateToString(controller->new_state_machine_.getActualState()));
            k_msleep(1000);
        }
        return;
    }
}