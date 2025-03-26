/** Sapienza Space Team
 * @author Niccolò Gatti 
 * @author Vittorio De Lucia
 * @date 08/06/2024
 * @copyright 2024 Copyright (c) | Sapienza Space Team, all rights reserved.
 */

#include "telemetry.hpp"

#include <charconv>
#include <zephyr/logging/log.h>
#include "common/common.hpp"

LOG_MODULE_REGISTER(TELEMETRY, LOG_LEVEL_DBG);

namespace payload::telemetry {

    Telemetry::Telemetry() {
        header_ = "TEAM_ID,MISSION_TIME,PACKET_COUNT,MODE,STATE,AIR_SPEED,HS_DEPLOYED,PC_DEPLOYED,TEMPERATURE,VOLTAGE,PRESSURE,GPS_TIME,GPS_ALTITUDE,GPS_LATITUDE,GPS_LONGITUDE,GPS_SATS,TILT_X,TILT_Y,ROT_Z,CMD_ECHO";
    }

    int Telemetry::get_team_id() const {
        return team_id_;
    }

    const utc_time& Telemetry::get_mission_time() const {
        return mission_time_;
    }
    void Telemetry::set_mission_time(const utc_time& mission_time) {
        mission_time_ = mission_time;
    }

    void Telemetry::tick_mission_time() {
        mission_time_.tick();
    }

    int Telemetry::get_packet_count() const {
        return packet_count_;
    }
    void Telemetry::set_packet_count(int packet_count) {
        packet_count_ = packet_count;
    }

    flight_mode_t Telemetry::get_mode() const {
        return mode_;
    }
    void Telemetry::set_mode(flight_mode_t mode) {
        mode_ = mode;
    }

    const std::string& Telemetry::get_software_state() const {
        return software_state_;
    }
    void Telemetry::set_software_state(const std::string& software_state) {
        software_state_ = software_state;
    } 

    double Telemetry::get_altitude() const {
        return altitude_;
    }
    void Telemetry::set_altitude(double altitude) {
        altitude_ = altitude;
    }

    hs_deployed_t Telemetry::get_hs_deployed() const {
        return hs_deployed_;
    }
    void Telemetry::set_hs_deployed(hs_deployed_t hs_deployed) {
        hs_deployed_ = hs_deployed;
    }

    pc_deployed_t Telemetry::get_pc_deployed() const {
        return pc_deployed_;
    }
    void Telemetry::set_pc_deployed(pc_deployed_t pc_deployed) {
        pc_deployed_ = pc_deployed;
    }

    double Telemetry::get_temperature() const {
        return temperature_;
    }
    void Telemetry::set_temperature(double temperature) {
        temperature_ = temperature;
    }

    double Telemetry::get_pressure() const {
        return pressure_;
    }
    void Telemetry::set_pressure(double pressure) {
        pressure_ = pressure;
    }
    
    double Telemetry::get_voltage() const {
        return voltage_;
    }
    void Telemetry::set_voltage(double voltage) {
        voltage_ = voltage;
    }

    double Telemetry::get_airspeed() const {
        return airspeed_;
    }
    void Telemetry::set_airspeed(double airspeed) {
        airspeed_ = airspeed;
    }

    const utc_time& Telemetry::get_gps_time() const {
        return gps_time_;
    }
    void Telemetry::set_gps_time(const utc_time& gps_time) {
        gps_time_ = gps_time;
    }

    double Telemetry::get_gps_altitude() const {
        return gps_altitude_;
    }
    void Telemetry::set_gps_altitude(double gps_altitude) {
        gps_altitude_ = gps_altitude;
    }

    double Telemetry::get_gps_latitude() const {
        return gps_latitude_;
    }
    void Telemetry::set_gps_latitude(double gps_latitude) {
        gps_latitude_ = gps_latitude;
    }

    double Telemetry::get_gps_longitude() const {
        return gps_longitude_;
    }
    void Telemetry::set_gps_longitude(double gps_longitude) {
        gps_longitude_ = gps_longitude;
    }

    int Telemetry::get_gps_sats() const {
        return gps_sats_;
    }
    void Telemetry::set_gps_sats(int gps_sats) {
        gps_sats_ = gps_sats;
    }

    double Telemetry::get_tilt_x() const {
        return tilt_x_;
    }
    void Telemetry::set_tilt_x(double tilt_x) {
        tilt_x_ = tilt_x;
    }

    double Telemetry::get_tilt_y() const {
        return tilt_y_;
    }
    void Telemetry::set_tilt_y(double tilt_y) {
        tilt_y_ = tilt_y;
    }

    double Telemetry::get_rot_z() const {
        return rot_z_;
    }
    void Telemetry::set_rot_z(double rot_z) {
        rot_z_ = rot_z;
    }

    const std::string& Telemetry::get_cmd_echo() const {
        return cmd_echo_;
    }
    void Telemetry::set_cmd_echo(const std::string& cmd_echo) {
        cmd_echo_ = cmd_echo;
    }

    std::string Telemetry::to_string() const {
        std::string telemetry {};
        // team_id
        telemetry.append(std::to_string(get_team_id()));
        telemetry.append(",");
        // mission_time
        telemetry.append(get_mission_time().to_string());
        telemetry.append(",");
        // packet_count
        telemetry.append(std::to_string(get_packet_count()));
        telemetry.append(",");
        // flight mode
        telemetry.append({static_cast<uint8_t>(mode_)});
        telemetry.append(",");
        // software state
        telemetry.append(get_software_state());
        telemetry.append(",");
        // altitude
        // telemetry.append(common::doubleToString(altitude_, 1));
        telemetry.append(common::adjustDP(altitude_, 1));
        telemetry.append(",");
        // airspeed
        // telemetry.append(common::doubleToString(airspeed_, 1));
        telemetry.append(common::adjustDP(airspeed_, 1));
        telemetry.append(",");
        // hs_deployed
        telemetry.append({static_cast<uint8_t>(hs_deployed_)});
        telemetry.append(",");
        // pc_deployed
        telemetry.append({static_cast<uint8_t>(pc_deployed_)});
        telemetry.append(",");
        // temperature
        // telemetry.append(common::doubleToString(temperature_, 1));
        telemetry.append(common::adjustDP(temperature_, 1));
        telemetry.append(",");
        // voltage
        // telemetry.append(common::doubleToString(voltage_, 1));
        telemetry.append(common::adjustDP(voltage_, 1));
        telemetry.append(",");
        // pressure
        // telemetry.append(common::doubleToString(pressure_, 1));
        telemetry.append(common::adjustDP(pressure_, 1));
        telemetry.append(",");
        // gps_time
        telemetry.append(gps_time_.to_string());
        telemetry.append(",");
        // gps_altitude
        // telemetry.append(common::doubleToString(gps_altitude_, 1));
        telemetry.append(common::adjustDP(gps_altitude_, 1));
        telemetry.append(",");
        // gps_latitude
        // telemetry.append(common::doubleToString(gps_latitude_, 4));
        telemetry.append(common::adjustDP(gps_latitude_, 4));
        telemetry.append(",");
        // gps_longitude
        // telemetry.append(common::doubleToString(gps_longitude_, 4));
        telemetry.append(common::adjustDP(gps_longitude_, 4));
        telemetry.append(",");
        // gps_sats
        telemetry.append(std::to_string(gps_sats_));
        telemetry.append(",");
        // tilt_x
        // telemetry.append(common::doubleToString(tilt_x_, 2));
        telemetry.append(common::adjustDP(tilt_x_, 2));
        telemetry.append(",");
        // tilt_y
        // telemetry.append(common::doubleToString(tilt_y_, 2));
        telemetry.append(common::adjustDP(tilt_y_, 2));
        telemetry.append(",");
        // rot_z
        // telemetry.append(common::doubleToString(rot_z_, 1));
        telemetry.append(common::adjustDP(rot_z_, 1));
        telemetry.append(",");
        // cmd_echo
        telemetry.append(cmd_echo_);
        return telemetry;
    }

    std::string Telemetry::to_string_long_states() const {
        std::string telemetry {};
        // team_id
        telemetry.append(std::to_string(get_team_id()));
        telemetry.append(",");
        // mission_time
        telemetry.append(get_mission_time().to_string());
        telemetry.append(",");
        // packet_count
        telemetry.append(std::to_string(get_packet_count()));
        telemetry.append(",");
        // flight mode
        telemetry.append({static_cast<uint8_t>(mode_)});
        telemetry.append(",");
        // software state
        telemetry.append(common::stateToLongStateString(get_software_state()));
        telemetry.append(",");
        // altitude
        telemetry.append(common::doubleToString(altitude_, 1));
        telemetry.append(",");
        // airspeed
        telemetry.append(common::doubleToString(airspeed_, 1));
        telemetry.append(",");
        // hs_deployed
        telemetry.append({static_cast<uint8_t>(hs_deployed_)});
        telemetry.append(",");
        // pc_deployed
        telemetry.append({static_cast<uint8_t>(pc_deployed_)});
        telemetry.append(",");
        // temperature
        telemetry.append(common::doubleToString(temperature_, 1));
        telemetry.append(",");
        // voltage
        telemetry.append(common::doubleToString(voltage_, 1));
        telemetry.append(",");
        // pressure
        telemetry.append(common::doubleToString(pressure_, 1));
        telemetry.append(",");
        // gps_time
        telemetry.append(gps_time_.to_string());
        telemetry.append(",");
        // gps_altitude
        telemetry.append(common::doubleToString(gps_altitude_, 1));
        telemetry.append(",");
        // gps_latitude
        telemetry.append(common::doubleToString(gps_latitude_, 4));
        telemetry.append(",");
        // gps_longitude
        telemetry.append(common::doubleToString(gps_longitude_, 4));
        telemetry.append(",");
        // gps_sats
        telemetry.append(std::to_string(gps_sats_));
        telemetry.append(",");
        // tilt_x
        telemetry.append(common::doubleToString(tilt_x_, 2));
        telemetry.append(",");
        // tilt_y
        telemetry.append(common::doubleToString(tilt_y_, 2));
        telemetry.append(",");
        // rot_z
        telemetry.append(common::doubleToString(rot_z_, 1));
        telemetry.append(",");
        // cmd_echo
        telemetry.append(cmd_echo_);
        return telemetry;
    }

    const std::string& Telemetry::get_telemetry_header() const {
        return header_;
    }
}