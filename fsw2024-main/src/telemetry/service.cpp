/** Sapienza Space Team
 * @author Niccolò Gatti 
 * @author Vittorio De Lucia
 * @date 08/06/2024
 * @copyright 2024 Copyright (c) | Sapienza Space Team, all rights reserved.
 */

#include "service.hpp"
#include "common/common.hpp"
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>

#include <stdio.h>
#include <iostream>
#include <string>

LOG_MODULE_REGISTER(TELEMETRY_SERVICE, LOG_LEVEL_DBG);

#define STACKSIZE 4096
#define PRIORITY  7
#define UPDATE_SLEEP_TIME 150
#define MAX_LANDED_SD_PACKETS 40
#define OFFSET_X 2.4
#define OFFSET_Y 0
#define MAX_PACKET_SIZE 108

K_THREAD_STACK_DEFINE(telemetry_thread_stack_area, STACKSIZE);
K_THREAD_STACK_DEFINE(gather_thread_stack_area, STACKSIZE);
K_THREAD_STACK_DEFINE(gyro_thread_stack_area, STACKSIZE);

namespace payload::telemetry {

    TelemetryService::TelemetryService(drivers::xbee::Xbee& xbee, drivers::bme280::BME280Sensor& altimeter, drivers::mpu6050::MPU6050Sensor& mpu, drivers::adc::Adc& adc, drivers::sdcard::SDCard& sd, const struct device *gps_dev): xbee_ {xbee}, alt_ {altimeter}, mpu_ {mpu}, adc_ {adc}, sd_ {sd}, gps_dev_ {gps_dev} {
        gps_api_ = (struct neom8_api *) gps_dev_->api;
        
        reference_altitude_ = common::get_altitude(alt_.getActualPressure() * 1000);
        reference_pitot_speed_ = adc_.getPitotSpeed();
        
        k_mutex_init(&packet_count_mutex_);
        k_mutex_init(&sd_mutex_);

        // Gather telemetry THREAD
        k_thread_create(&gather_thread_,
                        gather_thread_stack_area,
                        K_THREAD_STACK_SIZEOF(gather_thread_stack_area),
                        gather_entrypoint,
                        this,
                        nullptr,
                        nullptr,
                        PRIORITY,
                        0,
                        K_FOREVER);
        k_thread_name_set(&gather_thread_, "telemetry_gather");
        k_thread_start(&gather_thread_);

        // Gyro THREAD
        k_thread_create(&gyro_thread_,
                        gyro_thread_stack_area,
                        K_THREAD_STACK_SIZEOF(gyro_thread_stack_area),
                        gyro_entrypoint,
                        this,
                        nullptr,
                        nullptr,
                        PRIORITY,
                        0,
                        K_FOREVER);
        k_thread_name_set(&gyro_thread_, "gyro_thread");
        k_thread_start(&gyro_thread_);
        
        // Send telemetry THREAD
        k_thread_create(&telemetry_thread_,
                        telemetry_thread_stack_area,
                        K_THREAD_STACK_SIZEOF(telemetry_thread_stack_area),
                        telemetry_entrypoint,
                        this,
                        nullptr,
                        nullptr,
                        PRIORITY,
                        0,
                        K_NO_WAIT);
        k_thread_name_set(&telemetry_thread_, "telemetry_send");
        k_thread_suspend(&telemetry_thread_);
        
        tilt_x_angle = 0;
        tilt_y_angle = 0;
    }
    
    void TelemetryService::telemetry_entrypoint(void* instance, void*, void*) {
        auto telemetry_service_instance = reinterpret_cast<TelemetryService*>(instance);
        if (!telemetry_service_instance)
            return;
        telemetry_service_instance->start_telemetry_task();
    }
    
    void TelemetryService::gather_entrypoint(void* instance, void*, void*) {
        auto telemetry_service_instance = reinterpret_cast<TelemetryService*>(instance);
        if (!telemetry_service_instance)
            return;
        telemetry_service_instance->start_gather_task();
    }

    void TelemetryService::gyro_entrypoint(void* instance, void*, void*) {
        auto telemetry_service_instance = reinterpret_cast<TelemetryService*>(instance);
        if (!telemetry_service_instance)
            return;
        telemetry_service_instance->start_gyro_task();
    }
    
    void TelemetryService::startTelemetry() {
        k_thread_resume(&telemetry_thread_);
    }

    void TelemetryService::stopTelemetry() {
        k_thread_suspend(&telemetry_thread_);
    }

    void TelemetryService::start_telemetry_task() {
        drivers::xbee::detail::request::transmit_request tx_req;
        tx_req.set_dst_address(payload::config::GCS_ADDRESS);
        tx_req.set_frame_id(0);

        while(1) {
            k_mutex_lock(&packet_count_mutex_, K_FOREVER);
            payload_telemetry_.set_packet_count(packet_count_++);
            k_mutex_unlock(&packet_count_mutex_);

            std::string complete_packet = payload_telemetry_.to_string();
            //TO-DO: Reset parameters with pseudo-casual values
            if(complete_packet.size() > MAX_PACKET_SIZE){
                payload_telemetry_.set_airspeed(0.0);
                payload_telemetry_.set_gps_latitude(0.0);
                payload_telemetry_.set_gps_longitude(0.0);
                payload_telemetry_.set_tilt_y(0.0);
                complete_packet = payload_telemetry_.to_string();
                // printk("Lunghezza pacchetto ridotto: %d\n", complete_packet.size());
            }

            tx_req.set_payload(complete_packet);
            tx_req.calculate_length();
            tx_req.calculate_checksum();

            xbee_.send_request(tx_req);

            LOG_INF("\n%s\n", complete_packet.c_str());
            // printk("Lunghezza pacchetto: %d\n", complete_packet.size());

            payload_telemetry_.tick_mission_time();

            if(payload_telemetry_.get_software_state() == common::stateToString(common::longStateToState(LANDED))){
                stopTelemetry();
            }
            k_sleep(K_MSEC(1000));
        }
    }

    void TelemetryService::start_gather_task() {
        while(1) {
            updateTelemetryData();
            k_sleep(K_MSEC(UPDATE_SLEEP_TIME));
        }
    }

    double TelemetryService::gyrox_kalman_filter(double gyro_x) {
        Pp_x = p_x + Q_x;
        Xp_x = X_x;
        K_x = Pp_x / (Pp_x + R_x);
        e_x = gyro_x - Xp_x;
        p_x = (1 - K_x) * Pp_x;
        X_x = Xp_x + K_x*e_x;

        return X_x;
    }

    double TelemetryService::gyroy_kalman_filter(double gyro_y) {
        Pp_y = p_y + Q_y;
        Xp_y = X_y;
        K_y = Pp_y / (Pp_y + R_y);
        e_y = gyro_y - Xp_y;
        p_y = (1 - K_y) * Pp_y;
        X_y = Xp_y + K_y*e_y;

        return X_y;
    }

    void TelemetryService::start_gyro_task() {
        while(1) {
            mpu_.getGyro();

            double new_x_angle_vel = common::radiantToDegree(sensor_value_to_double(&mpu_.gyro[0]));
            double new_y_angle_vel = common::radiantToDegree(sensor_value_to_double(&mpu_.gyro[1]));

            double x_angle_vel_filtered = gyrox_kalman_filter(new_x_angle_vel + OFFSET_X);
            double y_angle_vel_filtered = gyroy_kalman_filter(new_y_angle_vel + OFFSET_Y);

            tilt_x_angle += x_angle_vel_filtered * UPDATE_SLEEP_TIME / 1000;
            tilt_y_angle += y_angle_vel_filtered * UPDATE_SLEEP_TIME / 1000;

            rot_z = common::radiantToDegree(sensor_value_to_double(&mpu_.gyro[2]));
            
            k_sleep(K_MSEC(UPDATE_SLEEP_TIME));
        }
    }

    void TelemetryService::setState(const std::string& state) {
        payload_telemetry_.set_software_state(state);
    }

    void TelemetryService::setFlightMode(flight_mode_t mode) {
        mode_ = mode;
    }

    flight_mode_t TelemetryService::getFlightMode() {
        return mode_;
    }

    void TelemetryService::setMissionTime(double raw_ts) {
        auto hour = static_cast<int32_t>(std::trunc(raw_ts / 10000.0));
        auto min  = static_cast<int32_t>((raw_ts - hour * 10000) / 100.0);
        auto sec  = raw_ts - min * 100 - hour * 10000;

        utc_time time = {.hour = hour, .min = min, .sec = static_cast<int32_t>(sec)};
        payload_telemetry_.set_mission_time(time);
    }

    void TelemetryService::setMissionTime(utc_time& time) {
        payload_telemetry_.set_mission_time(time);
    }

    void TelemetryService::setCmdEcho(const std::string& cmd_echo) {
        payload_telemetry_.set_cmd_echo(cmd_echo);
    }

    double TelemetryService::getAltitude() {
        return payload_telemetry_.get_altitude();
    }

    double TelemetryService::getPressure() {
        return payload_telemetry_.get_pressure();
    }

    double TelemetryService::getVoltage() {
        return payload_telemetry_.get_voltage();
    }

    double TelemetryService::getAirSpeed() {
        return payload_telemetry_.get_airspeed();
    }

    double TelemetryService::getVerticalAcceleration() const {
        return vertical_acceleration_;
    }

    void TelemetryService::setSimulatedPressure(uint32_t pressure) {
        simulated_pressure_ = pressure;
    }

    const utc_time& TelemetryService::getGpsTime() const {
        return payload_telemetry_.get_gps_time();
    }

    void TelemetryService::setParachuteDeploy(pc_deployed_t status) {
        payload_telemetry_.set_pc_deployed(status);
    }

    void TelemetryService::setHeatShieldDeploy(hs_deployed_t status) {
        payload_telemetry_.set_hs_deployed(status);
    }
   
    void TelemetryService::updateTelemetryData() {     
        // MODE, ALTITUDE, PRESSURE
        payload_telemetry_.set_mode(mode_);
        if (mode_ == flight_mode_t::FLIGHT) {
            if (first_fli == 0) reference_altitude_ = common::get_altitude(alt_.getActualPressure() * 1000);
            actual_altitude = common::get_altitude(alt_.getActualPressure() * 1000);
            payload_telemetry_.set_altitude(actual_altitude - reference_altitude_);
            payload_telemetry_.set_pressure(alt_.getActualPressure());

            first_fli++;
        } else if (mode_ == flight_mode_t::SIMULATION) {
            if (first_sim == 0) reference_altitude_ = common::get_altitude(simulated_pressure_);
            actual_altitude = common::get_altitude(simulated_pressure_);
            payload_telemetry_.set_altitude(actual_altitude - reference_altitude_);
            payload_telemetry_.set_pressure(simulated_pressure_ / 1000);
            first_sim++;
            first_fli = 0;
        }

        // AIR_SPEED
        double pitotSpeed = adc_.getPitotSpeed() - reference_pitot_speed_;
        payload_telemetry_.set_airspeed(pitotSpeed);

        // TEMPERATURE
        double temperature = alt_.getActualTemperature();
        payload_telemetry_.set_temperature(temperature);
        
        // VOLTAGE
        double voltage = adc_.getVoltage();
        payload_telemetry_.set_voltage(voltage);

        // GPS
		gps_api_->send_ubx(gps_dev_, 0x01, 0x07, {}, 0); // UBX-NAV-PVT
		gps_api_->send_ubx(gps_dev_, 0x01, 0x07, {}, 0); // UBX-NAV-PVT

		int rc = gps_api_->fetch_data(gps_dev_);
		if (rc) {
			printk("Error %d while reading data", rc);
		}
        struct gpsTime gps_time = gps_api_->get_time(gps_dev_);
        utc_time time
            = {.hour = gps_time.hour, .min = gps_time.min, .sec = static_cast<int32_t>(gps_time.sec)};

        double gpsAltitude = gps_api_->get_altitude(gps_dev_);
        double gpsLatitude = gps_api_->get_latitude(gps_dev_);
        double gpsLongitude = gps_api_->get_longitude(gps_dev_);
        payload_telemetry_.set_gps_time(time);
        payload_telemetry_.set_gps_altitude(gpsAltitude);
        payload_telemetry_.set_gps_latitude(gpsLatitude);
        payload_telemetry_.set_gps_longitude(gpsLongitude);
        payload_telemetry_.set_gps_sats(gps_api_->get_satellites(gps_dev_));

        // TILT_X, TILT_Y, ROT_Z
        payload_telemetry_.set_tilt_x(tilt_x_angle);
        payload_telemetry_.set_tilt_y(tilt_y_angle);
        payload_telemetry_.set_rot_z(rot_z);
        
        // Write telemetry on SD Card
        if(payload_telemetry_.get_software_state() != "S8"){
            k_mutex_lock(&sd_mutex_, K_FOREVER);
            std::string complete_sd_packet = payload_telemetry_.to_string_long_states();
            sd_.writeTelemetry(complete_sd_packet.append("\n"));
            k_mutex_unlock(&sd_mutex_);
        }

        k_msleep(1000);
    }

    void TelemetryService::create() {
        std::string telemetry = payload_telemetry_.to_string();
    }

    void TelemetryService::setReferencePressure() {
        reference_altitude_ = actual_altitude;
    }

    void TelemetryService::setSeaReferencePressure() {
        reference_pressure_ = config::WORLD_REFERENCE_PRESSURE;
    }

    TelemetryService::~TelemetryService() = default;
}