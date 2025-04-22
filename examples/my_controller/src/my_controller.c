#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

// Edit the debug name to get nice debug prints
#define DEBUG_MODULE "MYCONTROLLER"
#include "debug.h"

static const float K[4][6]= [[-7.07106781e-01 -1.69566287e-16  5.00000000e-01  9.00091045e-15
  -5.39415035e+01  5.00000000e-01 -2.87684351e+00 -5.86385767e-16
   2.71672377e+01  6.32865340e-15 -5.96456326e+01  5.95220734e+00]
 [-4.85387485e-17 -7.07106781e-01  5.00000000e-01  5.39415035e+01
  -2.54285853e-15 -5.00000000e-01 -1.98112634e-16 -2.87684351e+00
   2.71672377e+01  5.96456326e+01 -1.73353979e-15 -5.95220734e+00]
 [ 7.07106781e-01 -9.40895896e-17  5.00000000e-01  3.76353222e-15
   5.39415035e+01  5.00000000e-01  2.87684351e+00 -2.45023215e-16
   2.71672377e+01  1.03096911e-15  5.96456326e+01  5.95220734e+00]
 [ 4.77679027e-17  7.07106781e-01  5.00000000e-01 -5.39415035e+01
   4.14567616e-15 -5.00000000e-01  2.45848488e-16  2.87684351e+00
   2.71672377e+01 -5.96456326e+01  3.56414451e-15 -5.95220734e+00]]

// We still need an appMain() function, but we will not really use it. Just let it quietly sleep.
void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");

  while(1) {
    vTaskDelay(M2T(2000));

    // Remove the DEBUG_PRINT.
     DEBUG_PRINT("Running appMain()...!\n");
  }
}// The new controller goes here --------------------------------------------
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

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint,
  const sensorData_t *sensors, const state_t *state,
  const uint32_t tick) {
control->controlMode = controlModeDisable; // Or Legacy if needed for compatibility

// State vector: [x, y, z, roll, pitch, yaw]
float x[6] = {
state->position.x,
state->position.y,
state->position.z,
state->attitude.roll,
state->attitude.pitch,
state->attitude.yaw
};

// Reference vector: desired values from setpoint
float r[6] = {
setpoint->position.x,
setpoint->position.y,
setpoint->position.z,
setpoint->attitude.roll,
setpoint->attitude.pitch,
setpoint->attitude.yaw
};

// Compute control = -K(x - r)
float u[4] = {0};
for (int i = 0; i < 4; i++) {
for (int j = 0; j < 6; j++) {
u[i] -= K[i][j] * (x[j] - r[j]);
}
}

// Saturation/clamping (if needed)
float maxRoll = 20.0f, maxPitch = 20.0f, maxYawRate = 200.0f, maxThrust = 60000.0f;
control->roll = constrain(u[0], -maxRoll, maxRoll);
control->pitch = constrain(u[1], -maxPitch, maxPitch);
control->yaw = constrain(u[2], -maxYawRate, maxYawRate);
control->thrust = constrain(u[3], 0, maxThrust);
}