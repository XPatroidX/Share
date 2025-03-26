/**
 * @file sdcard.cpp
 * @author Lorenzo Thomas Contessa <lorenzocontessa.dev@gmail.com>
 * @date 19/05/2023
 * @copyright 2023 Copyright (c) | Sapienza Space Team, all rights reserved.
 */

#ifndef PAYLOAD_CONFIG_HPP
#define PAYLOAD_CONFIG_HPP

#include <cstdint>
#include <stdio.h>
#include <iostream>
#include <string>

//TO-DO Clean const

namespace payload {
    struct config {
        static inline constexpr int      TEAM_ID                    = 2007;
        // MAC Xbee Ragno: 0013A200421E1865
        static inline constexpr uint64_t GCS_ADDRESS                = 0x0013A20041CFED46;
        // static inline constexpr uint64_t GCS_ADDRESS                = 0x0013A200423AC7C6;
        static inline constexpr int      DEPLOY_PARACHUTE_ALTITUDE  = 400;
        static inline constexpr int      DEPLOY_PAYLOAD_ALTITUDE    = 300;
        static inline constexpr double   ALTIMETER_PRECISION        = 0.5f;
        static inline constexpr double   APOGEE_ALTITUDE_ERROR      = 0.1f;
        static constexpr double          G                          = 9.81f;
        static constexpr double          mG_to_MSECSEC              = (G / 1000.0f);
        static constexpr double          kPa_to_Pa                  = 1000.0f;
        static constexpr double          WORLD_REFERENCE_PRESSURE   = 101325.0f;
        static constexpr double          ASCENT_VELOCITY            = 20.0f;
        static constexpr double          APOGEE                     = 548.8f;
        static constexpr double          HEAT_SHIELD_RELEASE_ALTITUDE   = 150.0f;
    };
} // PAYLOAD_CONFIG_HPP

#endif// CFSW_CONFIG_HPP