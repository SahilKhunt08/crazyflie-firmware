/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 */
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "commander.h"
#include "crtp_commander.h"
#include "crtp_commander_high_level.h"

#include "cf_math.h"
#include "param.h"
#include "static_mem.h"

#include "console.h"

static bool isInit;
// Static structs are zero-initialized, so nullSetpoint corresponds to
// modeDisable for all stab_mode_t members and zero for all physical values.
// In other words, the controller should cut power upon recieving it.
const static setpoint_t nullSetpoint;
static state_t lastState;
const static int priorityDisable = COMMANDER_PRIORITY_DISABLE;

static uint32_t lastUpdate;
static bool enableHighLevel = false;

static QueueHandle_t setpointQueue;
STATIC_MEM_QUEUE_ALLOC(setpointQueue, 1, sizeof(setpoint_t));
static QueueHandle_t priorityQueue;
STATIC_MEM_QUEUE_ALLOC(priorityQueue, 1, sizeof(int));

// Custom task to send setpoints at 10Hz
void customSetpointTask(void *param)
{
    float initial_x = lastState.position.x; 
    float initial_y = lastState.position.y; 
    float initial_z = lastState.position.z; 

    while(1) {
        setCustomSetpoint(initial_x, initial_y, initial_z);
        vTaskDelay(100 / portTICK_RATE_MS); // 10Hz
    }
}

/* Public functions */
void commanderInit(void)
{
  setpointQueue = STATIC_MEM_QUEUE_CREATE(setpointQueue);
  ASSERT(setpointQueue);
  xQueueSend(setpointQueue, &nullSetpoint, 0);

  priorityQueue = STATIC_MEM_QUEUE_CREATE(priorityQueue);
  ASSERT(priorityQueue);
  xQueueSend(priorityQueue, &priorityDisable, 0);

  crtpCommanderInit();
  crtpCommanderHighLevelInit();
  lastUpdate = xTaskGetTickCount();

  isInit = true;
  xTaskCreate(customSetpointTask, "CUSTOM_SETPOINT", 2*configMINIMAL_STACK_SIZE, NULL, 2, NULL);
}

void commanderSetSetpoint(setpoint_t *setpoint, int priority)
{
  int currentPriority;

  const BaseType_t peekResult = xQueuePeek(priorityQueue, &currentPriority, 0);
  ASSERT(peekResult == pdTRUE);

  if (priority >= currentPriority) {
    setpoint->timestamp = xTaskGetTickCount();
    // This is a potential race but without effect on functionality
    xQueueOverwrite(setpointQueue, setpoint);
    xQueueOverwrite(priorityQueue, &priority);
    if (priority > COMMANDER_PRIORITY_HIGHLEVEL) {
      // Stop the high-level planner so it will forget its current state
      crtpCommanderHighLevelStop();
    }
  }
}

void setCustomSetpoint(float initial_x, float initial_y, float initial_z) {
    static setpoint_t setpoint;
    static TickType_t lastPrintTick = 0;
    TickType_t now = xTaskGetTickCount();

    memset(&setpoint, 0, sizeof(setpoint_t));
    setpoint.mode.x = modeAbs;
    setpoint.mode.y = modeAbs;
    setpoint.mode.z = modeAbs;
    setpoint.mode.yaw = modeAbs;
    setpoint.position.x = 10.0f;
    setpoint.position.y = 10.0f;
    setpoint.position.z = 21.0f;
    setpoint.attitude.yaw = 0.0f;

    // Print every 0.5 seconds (500 ms)
    if ((now - lastPrintTick) * portTICK_PERIOD_MS >= 500) {
        lastPrintTick = now;
        consolePrintf("Setpoint: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f\n",
            (double)setpoint.position.x, 
            (double)setpoint.position.y, 
            (double)setpoint.position.z, 
            (double)setpoint.attitude.yaw);

        consolePrintf("Current:  x=%.2f, y=%.2f, z=%.2f\n",
            (double)lastState.position.x,
            (double)lastState.position.y,
            (double)lastState.position.z);
    }

    commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_CRTP);
}



void commanderRelaxPriority()
{
  crtpCommanderHighLevelTellState(&lastState);
  int priority = COMMANDER_PRIORITY_LOWEST;
  xQueueOverwrite(priorityQueue, &priority);
}

void commanderGetSetpoint(setpoint_t *setpoint, const state_t *state)
{
  xQueuePeek(setpointQueue, setpoint, 0);
  lastUpdate = setpoint->timestamp;

  // This copying is not strictly necessary because stabilizer.c already keeps
  // a static state_t containing the most recent state estimate. However, it is
  // not accessible by the public interface.
  lastState = *state;
}

bool commanderTest(void)
{
  return isInit;
}

uint32_t commanderGetInactivityTime(void)
{
  return xTaskGetTickCount() - lastUpdate;
}

int commanderGetActivePriority(void)
{
  int priority;

  const BaseType_t peekResult = xQueuePeek(priorityQueue, &priority, 0);
  ASSERT(peekResult == pdTRUE);

  return priority;
}

/**
 *
 * The high level commander handles the setpoints from within the firmware
 * based on a predefined trajectory. This was merged as part of the
 * [Crazyswarm](%https://crazyswarm.readthedocs.io/en/latest/) project of the
 * [USC ACT lab](%https://act.usc.edu/) (see this
 * [blogpost](%https://www.bitcraze.io/2018/02/merging-crazyswarm-functionality-into-the-official-crazyflie-firmware/)).
 * The high-level commander uses a planner to generate smooth trajectories
 * based on actions like ‘take off’, ‘go to’ or ‘land’ with 7th order
 * polynomials. The planner generates a group of setpoints, which will be
 * handled by the High level commander and send one by one to the commander
 * framework.
 *
 * It is also possible to upload your own custom trajectory to the memory of
 * the Crazyflie, which you can try out with the script
 * `examples/autonomous_sequence_high_level of.py` in the Crazyflie python
 * library repository.
 */
PARAM_GROUP_START(commander)

/**
 * @brief Enable high level commander - deprecated (removed after August 2023)
 *
 * This parameter does not change anything and does not provide any functionality. There is no need
 * to set it before using the high level commander. See %https://github.com/bitcraze/crazyflie-firmware/pull/903
 */
PARAM_ADD_CORE(PARAM_UINT8, enHighLevel, &enableHighLevel)

PARAM_GROUP_STOP(commander)