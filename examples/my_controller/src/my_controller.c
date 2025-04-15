/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * hello_world.c - App layer application of a simple hello world debug print every
 *   2 seconds.
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

// Edit the debug name to get nice debug prints
#define DEBUG_MODULE "MYCONTROLLER"
#include "debug.h"
#include "autoconf.h" // include this if you want to access and print some of the variable definitions for this app


// We still need an appMain() function, but we will not really use it. Just let it quietly sleep.
void appMain() {
    DEBUG_PRINT("Waiting for activation ...\n");

    while(1) {
        vTaskDelay(M2T(2000));

        // Remove the DEBUG_PRINT.
        // DEBUG_PRINT("Hello World!\n");

        // Double check that CONFIG_CONTROLLER_OOT is set to True, otherwise you cannot use the
        // controller defined in this app
        DEBUG_PRINT("Is CONFIG_CONTROLLER_OOT set to true? %d", CONFIG_CONTROLLER_OOT);
    }
}

// The new controller goes here --------------------------------------------
// Move the includes to the the top of the file if you want to
#include "controller.h"

// Call the PID controller in this example to make it possible to fly. When you implement you own controller, there is
// no need to include the pid controller.
#include "controller_pid.h"

void controllerOutOfTreeInit() {
    // Initialize your controller data here...

    // Call the PID controller instead in this example to make it possible to fly
    controllerPidInit();
}

bool controllerOutOfTreeTest() {
    // Always return true
    return true;
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
    // Implement your controller here...

    // Call the PID controller instead in this example to make it possible to fly
    controllerPid(control, setpoint, sensors, state, tick);
}