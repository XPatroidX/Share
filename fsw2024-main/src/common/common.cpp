/** Sapienza Space Team
 * @author Niccolò Gatti 
 * @author Vittorio De Lucia
 * @date 08/06/2024
 * @copyright 2024 Copyright (c) | Sapienza Space Team, all rights reserved.
 */

#include <zephyr/kernel.h>
#include "common/common.hpp"

namespace payload::common {
    std::string stateToString(state state) {
        std::string stateStr;
        switch(state) {
            // INITIALIZE
            case S1: {
                stateStr = "S1";
                break;
            }
            // LAUNCH_WAIT
            case S2: {
                stateStr = "S2";
                break;
            }
            // ASCENT
            case S3: {
                stateStr = "S3";
                break;
            }
            // CANSAT_DESCENT
            case S4: {
                stateStr = "S4";
                break;
            }
            // HEAT_SHIELD_DEPLOY
            case S5: { 
                stateStr = "S5";
                break;
            }
            // HEAT_SHIELD_RELEASE
            case S6: { 
                stateStr = "S6";
                break;
            }
            // PC_DEPLOY
            case S7: {
                stateStr = "S7";
                break;
            }
            // LANDED
            case S8: {
                stateStr = "S8";
                break;
            }
            default: {
                stateStr = "UNKNOWN_STATE";
            }
        }
        return stateStr;
    }

    std::string longStateToString(long_state state) {
        std::string stateStr;
        switch(state) {
            // INITIALIZE
            case INITIALIZE: {
                stateStr = "INITIALIZE";
                break;
            }
            // LAUNCH_WAIT
            case LAUNCH_WAIT: {
                stateStr = "LAUNCH_WAIT";
                break;
            }
            // ASCENT
            case ASCENT: {
                stateStr = "ASCENT";
                break;
            }
            // CANSAT_DESCENT
            case CANSAT_DESCENT: {
                stateStr = "CANSAT_DESCENT";
                break;
            }
            // HEAT_SHIELD_DEPLOY
            case HEAT_SHIELD_DEPLOY: { 
                stateStr = "HEAT_SHIELD_DEPLOY";
                break;
            }
            // HEAT_SHIELD_RELEASE
            case HEAT_SHIELD_RELEASE: { 
                stateStr = "HEAT_SHIELD_RELEASE";
                break;
            }
            // PC_DEPLOY
            case PC_DEPLOY: {
                stateStr = "PC_DEPLOY";
                break;
            }
            // LANDED
            case LANDED: {
                stateStr = "LANDED";
                break;
            }
            default: {
                stateStr = "UNKNOWN_STATE";
            }
        }
        return stateStr;
    }

    state longStateToState(long_state long_state) {
        state shortState;
        switch(long_state) {
            // INITIALIZE
            case INITIALIZE: {
                shortState = S1;
                break;
            }
            // LAUNCH_WAIT
            case LAUNCH_WAIT: {
                shortState = S2;
                break;
            }
            // ASCENT
            case ASCENT: {
                shortState = S3;
                break;
            }
            // CANSAT_DESCENT
            case CANSAT_DESCENT: {
                shortState = S4;
                break;
            }
            // HEAT_SHIELD_DEPLOY
            case HEAT_SHIELD_DEPLOY: { 
                shortState = S5;
                break;
            }
            // HEAT_SHIELD_RELEASE
            case HEAT_SHIELD_RELEASE: { 
                shortState = S6;
                break;
            }
            // PC_DEPLOY
            case PC_DEPLOY: {
                shortState = S7;
                break;
            }
            // LANDED
            case LANDED: {
                shortState = S8;
                break;
            }
        }
        return shortState;
    }

    long_state stateToLongState(state short_state) {
        long_state state;
        switch(short_state) {
            // INITIALIZE
            case S1: {
                state = INITIALIZE;
                break;
            }
            // LAUNCH_WAIT
            case S2: {
                state = LAUNCH_WAIT;
                break;
            }
            // ASCENT
            case S3: {
                state = ASCENT;
                break;
            }
            // CANSAT_DESCENT
            case S4: {
                state = CANSAT_DESCENT;
                break;
            }
            // HEAT_SHIELD_DEPLOY
            case S5: { 
                state = HEAT_SHIELD_DEPLOY;
                break;
            }
            // HEAT_SHIELD_RELEASE
            case S6: { 
                state = HEAT_SHIELD_RELEASE;
                break;
            }
            // PC_DEPLOY
            case S7: {
                state = PC_DEPLOY;
                break;
            }
            // LANDED
            case S8: {
                state = LANDED;
                break;
            }
        }
        return state;
    }

    std::string stateToLongStateString(std::string stateStr) {
        std::string longStateStr;
        if (stateStr == "S1") {
            longStateStr = "INITIALIZE";
        }
        else if(stateStr == "S2"){
            longStateStr = "LAUNCH_WAIT";
        }
        else if(stateStr == "S3"){
            longStateStr = "ASCENT";
        }
        else if(stateStr == "S4"){
            longStateStr = "CANSAT_DESCENT";
        }
        else if(stateStr == "S5"){
            longStateStr = "HEAT_SHIELD_DEPLOY";
        }
        else if(stateStr == "S6"){
            longStateStr = "HEAT_SHIELD_RELEASE";
        }
        else if(stateStr == "S7"){
            longStateStr = "PC_DEPLOY";
        }
        else if(stateStr == "S8"){
            longStateStr = "LANDED";
        }
        
        return longStateStr;
    }

    void printFloat(float number) {
        int left = number;
        int right = (number - left) * 1000;
            
        printk("%d.%06d", left, right); 
    }

    std::string doubleToString(double value, int precision) {
        bool negative = false;

        if (value < 0)
        negative = true;

        auto int_part = static_cast<int>(value);
        if(int_part < 0)
        int_part *= -1.0f;

        auto dec_part = static_cast<int>((value - int_part) * std::pow(10, precision));
        if(dec_part < 0)
        dec_part *= -1.0f;

        std::array<char, 32> buff {'\0'};

        if (negative)
            std::snprintf(buff.data(), buff.size(), "-%d.%d", int_part, dec_part);
        else
            std::snprintf(buff.data(), buff.size(), "%d.%d", int_part, dec_part);
        return {buff.data()};
    }

    double get_altitude_from_pressure(double pressure, double reference_pressure_) {
        double delta_p = pressure / reference_pressure_;
        double x       = std::pow(delta_p, 0.190294f);
        return 44330 * (1 - x);
    }

    double get_altitude(double pressure) {
        double T0 = 288.15f;
        double gradient = 0.0065f;
        double P0 = 101325.00f;
        double R = 287.00f;

        double esp = (R*gradient)/config::G;
        double p_p0 = pressure/P0;

        return (T0/gradient) * (1 - pow(p_p0, esp));
    }

    parse_type<double> parse_double(const std::string& s) {
        char*  p;
        double d = ::strtod(s.c_str(), &p);
        if (*p != 0)
        {
            return {false, 0};
        }
        return {true, d};
    }

    std::vector<std::string> explode(std::string const & s, char delim) {
        std::vector<std::string> result;
        std::istringstream iss(s);

        for (std::string token; std::getline(iss, token, delim); )
        {
            result.push_back(std::move(token));
        }

        return result;
    }

    double convert_lat_lon_to_deg(const std::string& lat_lon, std::string dir) {
        auto double_val = common::parse_double(lat_lon);

        if (!double_val.valid)
            return 0;

        auto deg  = std::trunc(double_val.value / 100);
        auto mins = double_val.value - deg * 100;

        deg = deg + mins / 60.0;

        auto hdg = 'x';
        if (!dir.empty())
        {
            hdg = dir[0];
        }

        // everything should be N/E, so flip S,W
        if (hdg == 'S' || hdg == 'W')
        {
            deg *= -1.0;
        }
        return deg;
    }

    double radiantToDegree(double rad){
        return rad*180/M_PI;
    }

    std::string adjustDP(double value, int decimalPlaces) {
        // change the number of decimal places in a number
        std::stringstream result;
        result << std::setprecision(decimalPlaces) << std::fixed << value;
        return result.str();
    }
}