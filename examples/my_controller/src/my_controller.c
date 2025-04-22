#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

// Edit the debug name to get nice debug prints
#define DEBUG_MODULE "MYCONTROLLER"
#include "debug.h"

static const float K[4][12] = {
  {-7.07106781e-01f, -1.69566287e-16f,  5.00000000e-01f,  9.00091045e-15f,
   -5.39415035e+01f,  5.00000000e-01f, -2.87684351e+00f, -5.86385767e-16f,
    2.71672377e+01f,  6.32865340e-15f, -5.96456326e+01f,  5.95220734e+00f},
   
  {-4.85387485e-17f, -7.07106781e-01f,  5.00000000e-01f,  5.39415035e+01f,
   -2.54285853e-15f, -5.00000000e-01f, -1.98112634e-16f, -2.87684351e+00f,
    2.71672377e+01f,  5.96456326e+01f, -1.73353979e-15f, -5.95220734e+00f},
   
  { 7.07106781e-01f, -9.40895896e-17f,  5.00000000e-01f,  3.76353222e-15f,
    5.39415035e+01f,  5.00000000e-01f,  2.87684351e+00f, -2.45023215e-16f,
    2.71672377e+01f,  1.03096911e-15f,  5.96456326e+01f,  5.95220734e+00f},
   
  { 4.77679027e-17f,  7.07106781e-01f,  5.00000000e-01f, -5.39415035e+01f,
    4.14567616e-15f, -5.00000000e-01f,  2.45848488e-16f,  2.87684351e+00f,
    2.71672377e+01f, -5.96456326e+01f,  3.56414451e-15f, -5.95220734e+00f}
};

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

  // State vector
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
      for (int j = 0; j < 12; j++) {   
          u[i] -= K[i][j] * (x[j % 6] - r[j % 6]);  
      }
  }

}