/** Sapienza Space Team
 * @author Niccolò Gatti 
 * @author Vittorio De Lucia
 * @date 08/06/2024
 * @copyright 2024 Copyright (c) | Sapienza Space Team, all rights reserved.
 */

#ifndef PAYLOAD_COMMON
#define PAYLOAD_COMMON

#include "config.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <array>
#include <vector>
#include <string.h>
#include <string>

#define M_PI 3.1415926

enum state { S1, S2, S3, S4, S5, S6, S7, S8 };
enum long_state { INITIALIZE, LAUNCH_WAIT, ASCENT, CANSAT_DESCENT, HEAT_SHIELD_DEPLOY, HEAT_SHIELD_RELEASE, PC_DEPLOY, LANDED };

namespace payload::common {
  template<typename T>
    struct parse_type
    {
        bool valid;
        T    value;
    };

    std::string stateToString(state state);
    std::string longStateToString(long_state state);
    std::string stateToLongStateString(std::string stateStr);
    state longStateToState(long_state long_state);
    long_state stateToLongState(state short_state);

    void printFloat(float number);
    parse_type<double> parse_double(const std::string& s);
    std::string doubleToString(double value, int precision); //deprecated
    std::string adjustDP(double value, int decimalPlaces);   //new doubleToString
    double get_altitude_from_pressure(double pressure, double reference_pressure_);
    double get_altitude(double pressure);
    std::vector<std::string> explode(std::string const & s, char delim);
    double radiantToDegree(double rad);
}

#endif // PAYLOAD_COMMON