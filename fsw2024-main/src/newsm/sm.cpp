/** Sapienza Space Team
 * @author Niccolò Gatti 
 * @author Vittorio De Lucia
 * @date 08/06/2024
 * @copyright 2024 Copyright (c) | Sapienza Space Team, all rights reserved.
 */
#include <zephyr/kernel.h>
#include <zephyr/smf.h>

#include "stdio.h"
#include "sm.hpp"

namespace payload::newsm {

    NewStateMachine::NewStateMachine(commander::Commander& commander) : commander_ {commander} {
        actualState = common::longStateToState(INITIALIZE);
    }

    NewStateMachine::~NewStateMachine() = default;

    void NewStateMachine::initialize() {
        if(!initializeExecuted) {
            initializeExecuted = true;
            // printf("Ciao, sono in %s!\n", common::stateToString(getActualState()).c_str());
        }
    }

    void NewStateMachine::launchWait() {
        if(!launchWaitExecuted && initializeExecuted) {
            launchWaitExecuted = true;
            // printf("Ciao, sono in %s!\n", common::stateToString(getActualState()).c_str());
        }
    }

    void NewStateMachine::ascent() {
        if(!ascentExecuted && launchWaitExecuted) {
            ascentExecuted = true;
            // printf("Ciao, sono in %s!\n", common::stateToString(getActualState()).c_str());
        }
    }

    void NewStateMachine::cansatDescent() {
        if(!cansatDescentExecuted) {
            cansatDescentExecuted = true;
            // printf("Ciao, sono in %s!\n", common::stateToString(getActualState()).c_str());
        }
    }

    void NewStateMachine::heatShieldDeploy() {
        if(!heatShieldDeployExecuted) {
            commander_.heatShieldDeploy = true;
            heatShieldDeployExecuted = true;
            // printf("Ciao, sono in %s!\n", common::stateToString(getActualState()).c_str());
        }
    }

    void NewStateMachine::heatShieldRelease() {
        if(!heatShieldReleaseExecuted) {
            commander_.heatShieldRelease = true;
            heatShieldReleaseExecuted = true;
            // printf("Ciao, sono in %s!\n", common::stateToString(getActualState()).c_str());
        }
    }

    void NewStateMachine::pcDeploy() {
        if(!pcDeployExecuted) {
            commander_.parachuteDeploy = true;
            pcDeployExecuted = true;
            // printf("Ciao, sono in %s!\n", common::stateToString(getActualState()).c_str());
        }
    }

    void NewStateMachine::landed() {
        if(!landedExecuted) {
            commander_.goBuzzerLanded = true;
            landedExecuted = true;
            // printf("Ciao, sono in %s!\n", common::stateToString(getActualState()).c_str());
        }
    }

    int32_t NewStateMachine::execute() {
        int32_t ret;
        while(1) {
            state _state = getActualState();
            commander_.setActualState(actualState);
            switch(_state) {
                case state::S1: {
                    initialize();
                    break;
                }
                case state::S2: {
                    launchWait();
                    break;
                }
                case state::S3: {
                    ascent();
                    break;
                }
                case state::S4: {
                    cansatDescent();
                    break;
                }
                case state::S5: {
                    heatShieldDeploy();
                    break;
                }
                case state::S6: {
                    heatShieldRelease();
                    break;
                }
                case state::S7: {
                    pcDeploy();
                    break;
                }
                case state::S8: {
                    landed();
                    break;
                }
                default:
                    break;
            }
            k_msleep(1000);
        }
        return ret;
    }

    void NewStateMachine::setState(state state) {
        if(state != getActualState() && state == getActualState() + 1) {
            changeState = true;
            setActualState(state);
        }
    }

    void NewStateMachine::forceState(state state) {
        changeState = true;
        setActualState(state);
    }

    state NewStateMachine::getActualState() {
        return actualState;
    }

    void NewStateMachine::setActualState(state actual) {
        actualState = actual;
    }
}