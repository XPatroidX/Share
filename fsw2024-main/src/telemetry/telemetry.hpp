/** Sapienza Space Team
 * @author Niccolò Gatti 
 * @author Vittorio De Lucia
 * @date 08/06/2024
 * @copyright 2024 Copyright (c) | Sapienza Space Team, all rights reserved.
 */
#ifndef PAYLOAD_TELEMETRY_HPP
#define PAYLOAD_TELEMETRY_HPP

#include <string>
#include <ctime>
#include "config.hpp"

namespace payload::telemetry {
    struct utc_time {
        int32_t hour;
        int32_t min;
        int32_t sec;

        [[nodiscard]] std::string to_string() const {
            std::string time {};
            if ((0 <= hour) && (hour <= 9))
            time.append("0");
            time.append(std::to_string(hour));

            time.append(":");

            if ((0 <= min) && (min <= 9))
            time.append("0");
            time.append(std::to_string(min));

            time.append(":");

            if ((0 <= sec) && (sec <= 9))
            time.append("0");
            time.append(std::to_string(sec));

            return time;
        }

        void tick() {
            tm t {};
            t.tm_year   = 70;
            t.tm_mon    = 1;
            t.tm_mday   = 1;
            t.tm_hour   = hour;
            t.tm_min    = min;
            t.tm_sec    = sec + 1;
            auto new_t  = mktime(&t);
            auto new_tm = *localtime(&new_t);
            hour        = new_tm.tm_hour;
            min         = new_tm.tm_min;
            sec         = new_tm.tm_sec;
        }
    };

    enum class flight_mode_t : std::uint8_t {
        FLIGHT     = 'F',
        SIMULATION = 'S'
    };

    enum class hs_deployed_t : std::uint8_t {
        NOT_DEPLOYED = 'N',
        DEPLOYED     = 'P'
    };

    enum class pc_deployed_t : std::uint8_t {
        NOT_DEPLOYED = 'N',
        DEPLOYED     = 'C'
    };

    class Telemetry {
        public:
            explicit Telemetry();
            
            [[nodiscard]] int                 get_team_id() const;
            [[nodiscard]] const utc_time&     get_mission_time() const;
            void                              set_mission_time(const utc_time& mission_time);
            [[nodiscard]] int                 get_packet_count() const;
            void                              set_packet_count(int packet_count);
            [[nodiscard]] const std::string&  get_software_state() const;
            void                              set_software_state(const std::string& software_state);
            void                              tick_mission_time();

            [[nodiscard]] flight_mode_t      get_mode() const;
            void                             set_mode(flight_mode_t mode);
            [[nodiscard]] hs_deployed_t      get_hs_deployed() const;
            void                             set_hs_deployed(hs_deployed_t hs_deployed);
            [[nodiscard]] pc_deployed_t      get_pc_deployed() const;
            void                             set_pc_deployed(pc_deployed_t pc_deployed);

            [[nodiscard]] double             get_altitude() const;
            void                             set_altitude(double altitude);
            [[nodiscard]] double             get_temperature() const;
            void                             set_temperature(double temperature);
            [[nodiscard]] double             get_pressure() const;
            void                             set_pressure(double pressure);
            [[nodiscard]] double             get_voltage() const;
            void                             set_voltage(double voltage);
            [[nodiscard]] double             get_airspeed() const;
            void                             set_airspeed(double airspeed);
            [[nodiscard]] const utc_time&    get_gps_time() const;
            void                             set_gps_time(const utc_time& gps_time);
            [[nodiscard]] double             get_gps_latitude() const;
            void                             set_gps_latitude(double gps_latitude);
            [[nodiscard]] double             get_gps_longitude() const;
            void                             set_gps_longitude(double gps_longitude);
            [[nodiscard]] double             get_gps_altitude() const;
            void                             set_gps_altitude(double gps_altitude);
            [[nodiscard]] int                get_gps_sats() const;
            void                             set_gps_sats(int gps_sats);
            [[nodiscard]]                    double get_tilt_x() const;
            void                             set_tilt_x(double tilt_x);
            [[nodiscard]]                    double get_tilt_y() const;
            void                             set_tilt_y(double tilt_y);
            [[nodiscard]]                    double get_rot_z() const;
            void                             set_rot_z(double tilt_y);
            [[nodiscard]] const std::string& get_cmd_echo() const;
            void                             set_cmd_echo(const std::string& cmd_echo);
            [[nodiscard]] std::string        to_string() const;
            [[nodiscard]] std::string        to_string_long_states() const;
            [[nodiscard]] const std::string&  get_telemetry_header() const;

        private:
            std::string header_;
            const int     team_id_ {config::TEAM_ID};
            utc_time      mission_time_ {};
            int           packet_count_ {};
            std::string   software_state_ {};

            flight_mode_t   mode_ {flight_mode_t::FLIGHT};
            hs_deployed_t hs_deployed_ {hs_deployed_t::NOT_DEPLOYED};
            pc_deployed_t pc_deployed_ {pc_deployed_t::NOT_DEPLOYED};

            double        altitude_ {0.0f};
            double        temperature_ {0.0f};
            double        pressure_ {0.0f};
            double        voltage_ {0.0f};
            double        airspeed_ {0.0f};
            utc_time      gps_time_ {};
            double        gps_latitude_ {0.0f};
            double        gps_longitude_ {0.0f};
            double        gps_altitude_ {0.0f};
            int           gps_sats_ {};
            double        tilt_x_ {0.0f};
            double        tilt_y_ {0.0f};
            double        rot_z_ {0.0f};
            std::string   cmd_echo_ {};
    };

}

#endif // PAYLOAD_TELEMETRY_HPP