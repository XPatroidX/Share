/** Sapienza Space Team
 * @author Niccolò Gatti 
 * @author Vittorio De Lucia
 * @date 08/06/2024
 * @copyright 2024 Copyright (c) | Sapienza Space Team, all rights reserved.
 */

#ifndef PAYLOAD_SM_HPP
#define PAYLOAD_SM_HPP

#include "common/common.hpp"
#include "commander/commander.hpp"

#include <stdio.h>
#include <iostream>
#include <string>

namespace payload::newsm {
    class NewStateMachine {
        public:
            explicit NewStateMachine(commander::Commander& commander);
            ~NewStateMachine();

            int32_t execute();
            state getActualState();

            void setState(state state);
            void forceState(state state);

            void initialize();
            void launchWait();
            void ascent();
            void cansatDescent();
            void heatShieldDeploy();
            void heatShieldRelease();
            void pcDeploy();
            void landed();
            commander::Commander& commander_;

        private:
            state actualState;
            void setActualState(state actual);

            bool initializeExecuted = false;
            bool launchWaitExecuted = false;
            bool ascentExecuted = false;
            bool cansatDescentExecuted = false;
            bool heatShieldDeployExecuted = false;
            bool heatShieldReleaseExecuted = false;
            bool pcDeployExecuted = false;
            bool landedExecuted = false;

            bool changeState = false;
    };
}

#endif