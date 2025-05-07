#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "app.h"
#include "console.h"

#include "FreeRTOS.h"
#include "task.h"

// Edit the debug name to get nice debug prints
#define DEBUG_MODULE "MYCONTROLLER"
#include "debug.h"
#include "math3d.h"

#define NX 12
#define NU 4

//4x12 x 12x1 = 4x1

// Computed from Trajectory Generation
#define K1 (float) 3.16e-10
#define K2 (float) 7.94e-12
#define L (float) 0.0397

static float u[NU] = {0};
static float W[4];

static const float M[NU][NU] = {
      {K1, K1, K1, K1},
      {0, L*K1, 0, -L*K1},
      {0, -L*K1, 0, L*K1},
      {K2, -K2, K2, -K2}
};
  
static const float K[NU][NX] = {
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

static inline int16_t saturateSignedInt16(float in)
{
  if (in > INT16_MAX)
    return INT16_MAX;
  else if (in < -INT16_MAX)
    return -INT16_MAX;
  else
    return (int16_t)in;
}

// Add helper function for float constraints
static inline float constrainf(float value, float min, float max) {
  if (value < min) return min;
  if (value > max) return max;
  return value;
} 


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
  // controllerPidInit();
}

bool controllerOutOfTreeTest() {
  // Always return true
  return true;
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint,
  const sensorData_t *sensors, const state_t *state,
  const uint32_t tick) {
  control->controlMode = controlModeLegacy;
  // consolePrintf("Using MYCONTROLLER \n");

  // State vector
  float x[NX] = {
    state->position.x,
    state->position.y,
    state->position.z,
    state->attitude.roll,
    state->attitude.pitch,
    state->attitude.yaw,
    state->velocity.x,
    state->velocity.y,
    state->velocity.z,
    sensors->gyro.x,
    sensors->gyro.y,
    sensors->gyro.z
  };

  // Reference vector: desired values from setpoint
  float r[NX] = {
    setpoint->position.x,
    setpoint->position.y,
    setpoint->position.z,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0
  };
  // consolePrintf("Updated 141 \n");
  // Compute control vector(u) = u_eq - K(x - r)
  for (int i = 0; i < NU; i++) {
    u[i] = 0;
      for (int j = 0; j < NX; j++) {   
          u[i] += 14475.809152959684f - K[i][j]*(x[j] - r[j]);
      }
  }

  //Square elements in u, and rearrange order to fit with paper: https://scholarsarchive.byu.edu/cgi/viewcontent.cgi?article=2324&context=facpub
  //According to paper, apply M matrix to get the (thrust, roll, pitch, yaw) torque vector

// Alter from paper as pitch, yaw and roll are not on the same axis as K-matrix
  W[0] = u[0] * u[0]; 
  W[1] = u[1] * u[1];  
  W[2] = u[2] * u[2]; 
  W[3] = u[3] * u[3];  

  // consolePrintf("Updated 160 \n");


  for (int i = 0; i < 4; i++) {
      u[i] = 0;
      for (int j = 0; j < 4; j++) {
          u[i] += M[i][j] * W[j];
      }
      
  }
  // consolePrintf("Updated u[0] \n");
  
  //rpm from each motor => thrust for each motor
  int maxThrust = 65536;
  int minThrust = 0;
  
  /* This format is for when control->mode == controlModeLegacy */
  control->thrust = constrainf(1000*u[0], minThrust, maxThrust);
  control->roll = saturateSignedInt16(u[1]);
  control->pitch = saturateSignedInt16(u[2]);
  control->yaw = saturateSignedInt16(u[3]);

  /* This format is for when control->mode == controlModeForceTorque */
  // control->thrustSi = u[0];
  // control->torqueX = u[1];
  // control->torqueY = u[2];
  // control->torqueZ = u[3];

  // consolePrintf("u[0] %f, u[1] %f, u[2] %f, u[3] %f \n", 
  //   (double)u[0], (double) u[1], (double) u[2], (double) u[3]);

  // static TickType_t lastPrintTick = 0;
  // TickType_t now = xTaskGetTickCount();
  // if ((now - lastPrintTick) * portTICK_PERIOD_MS >= 500) {
  //   lastPrintTick = now;
  consolePrintf("THR:%d R:%d P:%d Y:%d \n", 
    (int)control->thrust, (int)control->roll,
    (int)control->pitch, (int)control->yaw);

  consolePrintf("Setpoint: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f\n",
    (double)setpoint->position.x, 
    (double)setpoint->position.y, 
    (double)setpoint->position.z, 
    (double)setpoint->attitude.yaw);

  consolePrintf("Current:  x=%.2f, y=%.2f, z=%.2f\n",
    (double)state->position.x,
    (double)state->position.y,
    (double)state->position.z);
  // }
  
  /* This format is for when control->mode == controlModeLegacy */
  // consolePrintf("THR:%d R:%d P:%d Y:%d \n", 
  //             (int)control->thrust, (int)control->roll,
  //             (int)control->pitch, (int)control->yaw);

  /* This format is for when control->mode == controlModeForceTorque */
  // consolePrintf("THR:%f R:%f P:%f Y:%f \n", 
  //   (double)control->thrustSi, (double)control->torqueX,
  //   (double)control->torqueY, (double)control->torqueZ);
}


