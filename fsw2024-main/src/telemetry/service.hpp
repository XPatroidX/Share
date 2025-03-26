/**
 * @file service.hpp
 * @author Lorenzo Thomas Contessa <lorenzocontessa.dev@gmail.com>
 * @date 21/05/2023
 * @copyright 2023 Copyright (c) | Sapienza Space Team, all rights reserved.
 */

#ifndef PAYLOAD_TELEMETRY_SERVICE_HPP
#define PAYLOAD_TELEMETRY_SERVICE_HPP

#include "telemetry.hpp"
#include <stdio.h>
#include <iostream>
#include <string>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>

#include <drivers/xbee/xbee.hpp>
#include <drivers/mpu6050/mpu6050.hpp>
#include <drivers/gnss/ublox_neo_m8.h>
#include "drivers/bme280/bme280.hpp"
#include "drivers/adc/adc.hpp"
#include "drivers/sdcard/sdcard.hpp"


namespace payload::telemetry {
    class TelemetryService {
        public:
            explicit TelemetryService(drivers::xbee::Xbee& xbee, drivers::bme280::BME280Sensor& altimeter, drivers::mpu6050::MPU6050Sensor& mpu, drivers::adc::Adc& adc, drivers::sdcard::SDCard& sd, const struct device *gps_dev);
            ~TelemetryService();
            
            static void telemetry_entrypoint(void* instance, void*, void*);
            static void gather_entrypoint(void* instance, void*, void*);
            static void gyro_entrypoint(void* instance, void*, void*);

            void startTelemetry();
            void stopTelemetry();
            void setReferencePressure();
            void setSeaReferencePressure();
            void setState(const std::string& state);
            void setMissionTime(double raw_ts);
            void setMissionTime(utc_time& time);
            void setCmdEcho(const std::string& cmd_echo);
            double getPressure();
            double getAltitude();
            double getVoltage();
            double getAirSpeed();
            double getVerticalAcceleration() const;
            void setFlightMode(flight_mode_t mode);
            void setSimulatedPressure(uint32_t pressure);
            const utc_time& getGpsTime() const;
            void setParachuteDeploy(pc_deployed_t status);
            void setHeatShieldDeploy(hs_deployed_t status);

            double gyrox_kalman_filter(double gyro_x);
            double gyroy_kalman_filter(double gyro_y);

            flight_mode_t getFlightMode();
            double reference_altitude_ {0.0f};
            double reference_pitot_speed_ {0.0f};
            uint32_t packet_count_ {1};
            uint32_t pc_deploy_packet_number {0};
            
            
        private:

            // variable for kalman filter
            double R_x = 0.026971021, R_y = 0.026971021;
            double Q_x = 1e-05, Q_y = 1e-05;
            double Pp_x = 0.0f, Pp_y = 0.0f;
            double K_x = 0.0f, K_y = 0.0f;
            double e_x = 0.0f, e_y = 0.0f;
            double p_x = 1.0f, p_y = 1.0f;
            double Xp_x = 0.0f, Xp_y = 0.0f;
            double X_x = 0.0f, X_y = 0.0f;

            void create();
            void start_telemetry_task();
            void start_gather_task();
            void start_gyro_task();
            void updateTelemetryData();

            Telemetry payload_telemetry_ {};
            k_thread telemetry_thread_ {};
            k_mutex packet_count_mutex_ {};
            k_mutex sd_mutex_ {};

            k_thread gather_thread_ {};
            k_thread gyro_thread_ {};

            flight_mode_t mode_ {flight_mode_t::FLIGHT};
            hs_deployed_t hs_deployed_ {hs_deployed_t::NOT_DEPLOYED};
            pc_deployed_t pc_deployed_ {pc_deployed_t::NOT_DEPLOYED};

            double pressure_ {};                                           // Pascal
            double simulated_pressure_ {};                                 // Pascal
            double vertical_acceleration_ {};                              // m/s^2
            double reference_pressure_ {config::WORLD_REFERENCE_PRESSURE}; // Pascal
            double tilt_x_angle {};
            double tilt_y_angle {};
            double rot_z {};
            double actual_altitude {};          //last calculated altitude in updateTelemetry

            drivers::xbee::Xbee& xbee_;
            drivers::bme280::BME280Sensor& alt_;
            drivers::mpu6050::MPU6050Sensor& mpu_;
            drivers::adc::Adc& adc_;
            drivers::sdcard::SDCard& sd_;

            const struct device *gps_dev_;
            struct neom8_api* gps_api_;

            int first_sim = 0;
            int first_fli = 0;
    };
}

#endif // PAYLOAD_TELEMETRY_SERVICE_HPP