/** Sapienza Space Team
 * @author Niccolò Gatti 
 * @author Vittorio De Lucia
 * @date 08/06/2024
 * @copyright 2024 Copyright (c) | Sapienza Space Team, all rights reserved.
 */
#include "commands.hpp"
#include "../commander/commander.hpp"
#include <ctime>
#include <vector>
#include <cmath>
#include <array>

namespace payload::telemetry {
    CommandsHandler::CommandsHandler(drivers::xbee::Xbee& xbee, controller::Controller& controller, commander::Commander& commander): xbee_ {xbee}, controller_ {controller}, commander_ {commander} {
        xbee.set_rx_callback([this](std::unique_ptr<drivers::xbee::i_response> response)
        { handleCommand(response->get_string()); });
    }
    
    /* GET COMMAND | Retrieve telemetry functions based on command received */
    void CommandsHandler::handleCommand(std::string command) {
        //if (command.length() > 32 || command.length() < 14) return;
        printf("Ho ricevuto un comando: %s\n", command.c_str());

        auto v = common::explode(command, ',');
        std::string initCommand = v.at(0);
        if(v.size() < 3){
            return;
        }

        if (initCommand == "CMD") {
            // ----------------Mission Guide Commands----------------
            if (v.at(2) == "CX") {
                if(v.at(3) == "ON") {
                    cxHandler(true);
                    //controller_.telemetry_.setReferencePressure();
                } else if(v.at(3) == "OFF") {
                    cxHandler(false);
                }
            } 
            else if (v.at(2) == "ST") {
                stHandler(v.at(3).c_str());
            } 
            else if (v.at(2) == "SIMP") {
                if(controller_.telemetry_.getFlightMode() == flight_mode_t::SIMULATION) {
                    printf("%s\n", v.at(3).c_str());
                    if (pressure_counter_from_ground == 0) {
                        controller_.telemetry_.reference_altitude_ = common::get_altitude(stoi((v.at(3))));
                    }
                    simpHandler(stoi(v.at(3)));

                    pressure_counter_from_ground++;
                }
            } 
            else if (v.at(2) == "SIM") {
                printf("%s\n", v.at(3).c_str());
                std::string state = v.at(3);
                if(state == "ENABLE") {
                    simHandler(flight_mode_state_t::ENABLE);
                } else if(state == "ACTIVATE") {
                    simHandler(flight_mode_state_t::ACTIVE);
                } else if(state == "DISABLE") {
                    simHandler(flight_mode_state_t::DISABLE);
                }
            } 
            else if (v.at(2) == "BCN") {
                std::string cmd_echo = "BCN";
                if(v.at(3) == "ON") {
                    controller_.new_state_machine_.commander_.singBuzzer();
                    cmd_echo.append("ON");
                    controller_.telemetry_.setCmdEcho(cmd_echo);
                } else if(v.at(3) == "OFF") {
                    controller_.new_state_machine_.commander_.shutUpBuzzer();
                    cmd_echo.append("OFF");
                    controller_.telemetry_.setCmdEcho(cmd_echo);
                }
            }
            else if (v.at(2) == "CAL") {
                calHandler();
            }
            // ----------------Custom Commands----------------
            else if (v.at(2) == "FORCEPC") {
                commander_.parachuteDeploy = true;
                commander_.deployParachute();
                controller_.telemetry_.setParachuteDeploy(telemetry::pc_deployed_t::DEPLOYED);
                controller_.telemetry_.setCmdEcho("FORCEPC");
            }
            else if(v.at(2) == "FORCEHD") {
                commander_.heatShieldDeploy = true;
                commander_.deployHeatShield();
                controller_.telemetry_.setHeatShieldDeploy(telemetry::hs_deployed_t::DEPLOYED);
                controller_.telemetry_.setCmdEcho("FORCEHD");
            } 
            else if(v.at(2) == "FORCEHR") {
                commander_.heatShieldRelease = true;
                commander_.releaseHeatShield();
                controller_.telemetry_.setCmdEcho("FORCEHR");
            }   
            else if (v.at(2) == "RESETPC") {
                commander_.parachuteDeployReset = true;
                commander_.resetParachute();
                controller_.telemetry_.setCmdEcho("RESETPC");
                controller_.telemetry_.setParachuteDeploy(telemetry::pc_deployed_t::NOT_DEPLOYED);
            } 
            else if (v.at(2) == "RESETHD") {
                commander_.heatShieldDeployReset = true;
                commander_.resetHeatShieldDeploy();
                controller_.telemetry_.setCmdEcho("RESETHD");
                controller_.telemetry_.setHeatShieldDeploy(telemetry::hs_deployed_t::NOT_DEPLOYED);
            } 
            else if (v.at(2) == "RESETHR") {
                commander_.heatShieldReleaseReset = true;
                commander_.resetHeatShieldRelease();
                controller_.telemetry_.setCmdEcho("RESETHR");
            } 
            else{
                return;
            }
        }
        else {
            return;
        };
    };

    void CommandsHandler::cxHandler(bool isActive) {
        std::string cmd_echo = "CX";
        if(isActive) {
            controller_.telemetry_.startTelemetry();
            cmd_echo.append("ON");
        } else {
            controller_.telemetry_.stopTelemetry();
            cmd_echo.append("OFF");
        }
        controller_.telemetry_.setCmdEcho(cmd_echo);
    };

    void CommandsHandler::stHandler(const char* missionTime) {
        printf("Ho ricevuto un orario: %s\n", missionTime);

        if(strcmp(missionTime, "GPS") == 0) {
            utc_time time = controller_.telemetry_.getGpsTime();
            controller_.telemetry_.setMissionTime(time);
            controller_.telemetry_.setCmdEcho("STGPS");
        }
        else {
            char* time_token = (char*)malloc(sizeof(char)*strlen(missionTime)+1);
            strcpy(time_token, missionTime);

            time_token = strtok(time_token, ":");
            if(time_token == NULL || strlen(time_token) != 2 || atoi(time_token) > 23 || atoi(time_token) < 0) return;
            int32_t h = atoi(time_token);

            time_token = strtok(NULL, ":");
            if(time_token == NULL || strlen(time_token) != 2 || atoi(time_token) > 59 || atoi(time_token) < 0) return;
            int32_t m = atoi(time_token);

            time_token = strtok(NULL, ":");
            if(time_token == NULL || strlen(time_token) != 2 || atoi(time_token) > 59 || atoi(time_token) < 0) return;
            int32_t s = atoi(time_token);

            utc_time* new_time = (utc_time*)malloc(sizeof(new_time));
            new_time->hour = h;
            new_time->min = m;
            new_time->sec = s;
            controller_.telemetry_.setMissionTime(*new_time);
            std::string cmd_echo = "ST";
            cmd_echo.append(missionTime);
            controller_.telemetry_.setCmdEcho(cmd_echo);
        } 
    };

    void CommandsHandler::simpHandler(uint32_t pressure) {
        std::string cmd_echo = "SP";
        controller_.telemetry_.setSimulatedPressure(pressure);
        cmd_echo.append(std::to_string(pressure));
        controller_.telemetry_.setCmdEcho(cmd_echo);
    };

    void CommandsHandler::simHandler(flight_mode_state_t state) {
        std::string cmd_echo = "SIM";
        if(state == flight_mode_state_t::ENABLE){
            printf("Sim Enabled!\n");
            simEnabled = true;
            cmd_echo.append("E");
        }
        if(state == flight_mode_state_t::ACTIVE && simEnabled) {
            printf("Sim Activated!\n");
            // controller_.telemetry_.setSimulatedPressure(config::WORLD_REFERENCE_PRESSURE);
            controller_.telemetry_.setFlightMode(flight_mode_t::SIMULATION);
            // controller_.telemetry_.setSeaReferencePressure();
            cmd_echo.append("A");
        }
        if(state == flight_mode_state_t::DISABLE) {
            printf("Sim Disabled!\n");
            simEnabled = false;
            controller_.telemetry_.setFlightMode(flight_mode_t::FLIGHT);
            cmd_echo.append("D");
        }
        controller_.telemetry_.setCmdEcho(cmd_echo);
    };

    void CommandsHandler::calHandler() {
        controller_.telemetry_.setReferencePressure();
        controller_.telemetry_.setCmdEcho("CAL");
    };

    CommandsHandler::~CommandsHandler() = default;
    

}