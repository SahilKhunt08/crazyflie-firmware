/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021 Bitcraze AB
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
 */

#include "mm_sweep_angles.h"


void kalmanCoreUpdateWithSweepAngles(kalmanCoreData_t *this, sweepAngleMeasurement_t *sweepInfo, const uint32_t nowMs, OutlierFilterLhState_t* sweepOutlierFilterState) {
  // Sensor position in global reference frame
  vec3d s;
  arm_matrix_instance_f32 s_ = {3, 1, s};
  // Rotation matrix from Crazyflie to global reference frame
  arm_matrix_instance_f32 Rcf_ = {3, 3, (float32_t *)this->R};
  // Sensor position in Crazyflie reference frame
  arm_matrix_instance_f32 scf_ = {3, 1, (float32_t *)*sweepInfo->sensorPos};

  // Rotate the sensor position to the global reference frame
  mat_mult(&Rcf_, &scf_, &s_);

  // Crazyflie position in global reference frame
  // Gets the current state values of the position of the Crazyflie in the global reference frame and add the relative sensor pos
  vec3d pcf = {this->S[KC_STATE_X] + s[0], this->S[KC_STATE_Y] + s[1], this->S[KC_STATE_Z] + s[2]};

  // Rotor position in global reference frame
  // Calculates the difference between the rotor and the sensor on the Crazyflie in global reference frame
  const vec3d* pr = sweepInfo->rotorPos;
  // Difference in position between the rotor and the sensor on the Crazyflie in global reference frame
  vec3d stmp = {pcf[0] - (*pr)[0], pcf[1] - (*pr)[1], pcf[2] - (*pr)[2]};
  arm_matrix_instance_f32 stmp_ = {3, 1, stmp};

  // Rotate the difference in position to the rotor reference frame,
  // using the rotor inverse rotation matrix
  vec3d sr;
  arm_matrix_instance_f32 Rr_inv_ = {3, 3, (float32_t *)(*sweepInfo->rotorRotInv)};
  arm_matrix_instance_f32 sr_ = {3, 1, sr};
  mat_mult(&Rr_inv_, &stmp_, &sr_);

  // The following computations are in the rotor refernece frame
  const float x = sr[0];
  const float y = sr[1];
  const float z = sr[2];
  const float t = sweepInfo->t;
  const float tan_t = tanf(t);

  const float r2 = x * x + y * y;
  const float r = arm_sqrt(r2);

  const float predictedSweepAngle = sweepInfo->calibrationMeasurementModel(x, y, z, t, sweepInfo->calib);
  const float measuredSweepAngle = sweepInfo->measuredSweepAngle;
  const float error = measuredSweepAngle - predictedSweepAngle;

  if (outlierFilterLighthouseValidateSweep(sweepOutlierFilterState, r, error, nowMs)) {
    // Calculate H vector (in the rotor reference frame)
    const float z_tan_t = z * tan_t;
    const float qNum = r2 - z_tan_t * z_tan_t;
    // Avoid singularity
    if (qNum > 0.0001f) {
      // Position Jacobians: ∂α/∂X, ∂α/∂Y, ∂α/∂Z

      const float q = tan_t / arm_sqrt(qNum);

      // Jacobian of the sweep angle measurement α w.r.t sensor position (x, y, z)
      // Computed in the rotor reference frame:
      // gr = [ ∂α/∂x, ∂α/∂y, ∂α/∂z ] (rotor frame)
      //
      // For derivation review the paper:
      // Taffanel et al. (2021), "Lighthouse positioning system: Dataset, accuracy, and precision for UAV research", arXiv:2104.11523
      vec3d gr = {(-y - x * z * q) / r2, (x - y * z * q) / r2 , q};

      // Jacobian of the sweep angle measurement α w.r.t sensor position,
      // rotated from rotor frame (gr) into the global reference frame:
      // g = [ ∂α/∂X, ∂α/∂Y, ∂α/∂Z ] (global frame)
      vec3d g;

      arm_matrix_instance_f32 gr_ = {3, 1, gr};
      arm_matrix_instance_f32 Rr_ = {3, 3, (float32_t *)(*sweepInfo->rotorRot)};
      arm_matrix_instance_f32 g_ = {3, 1, g};
      mat_mult(&Rr_, &gr_, &g_);

      float h[KC_STATE_DIM] = {0};
      h[KC_STATE_X] = g[0]; // ∂α/∂X
      h[KC_STATE_Y] = g[1]; // ∂α/∂Y
      h[KC_STATE_Z] = g[2]; // ∂α/∂Z

      // Orientation Jacobians: ∂α/∂(δφ), ∂α/∂(δθ), ∂α/∂(δψ)
      //
      // Applying the chain rule, we get:
      // ∂α/∂(∂φ) = ∂α/∂x * ∂x/∂(∂φ) + ∂α/∂y * ∂y/∂(∂φ) + ∂α/∂z * ∂z/∂(∂φ)
      //          = ∂α/∂x * ∂x/∂φ * ∂φ/∂(∂φ) + ∂α/∂x * ∂x/∂θ * ∂θ/∂(∂φ) + ∂α/∂x * ∂x/∂ψ * ∂ψ/∂(∂φ) + ...
      //
      // We already have ∂α/∂X, ∂α/∂Y, ∂α/∂Z from the position Jacobians
      // We still need the partial derivatives of position w.r.t the Euler angles,
      //  and the partial derivatives of the Euler angles w.r.t the orientation error
      //
      // We need ∂x/∂φ, ∂y/∂φ, ∂z/∂φ, ∂x/∂θ, ∂y/∂θ, ∂z/∂θ, ∂x/∂ψ, ∂y/∂ψ, ∂z/∂ψ
      // These can be derived from the rotation matrices:

      float dx_droll = 0.0f; // ∂x/∂φ
      float dy_droll = -s[2]; // ∂y/∂φ
      float dz_droll = s[1]; // ∂z/∂φ

      float dx_dpitch = -s[2]; // ∂x/∂θ
      float dy_dpitch = 0.0f; // ∂y/∂θ
      float dz_dpitch = s[0]; // ∂z/∂θ

      float dx_dyaw = -s[1]; // ∂x/∂ψ
      float dy_dyaw = s[0]; // ∂y/∂ψ
      float dz_dyaw = 0.0f; // ∂z/∂ψ

      // We also need ∂φ/∂(∂φ), ∂θ/∂(∂φ), ∂ψ/∂(∂φ), ∂φ/∂(∂θ), ∂θ/∂(∂θ), ∂ψ/∂(∂θ), ∂φ/∂(∂ψ), ∂θ/∂(∂ψ), ∂ψ/∂(∂ψ)
      // These can be approached by the Euler angles:

      float phi = atan2f(2*(this->q[0]*this->q[1] + this->q[2]*this->q[3]), 1 - 2*(this->q[1]*this->q[1] + this->q[2]*this->q[2]));
      float theta = asinf(2*(this->q[0]*this->q[2] - this->q[3]*this->q[1]));
      // float psi = atan2f(2*(this->q[0]*this->q[3] + this->q[1]*this->q[2]), 1 - 2*(this->q[2]*this->q[2] + this->q[3]*this->q[3]));

      float droll_dv0 = 1.0f; // ∂φ/∂(δφ)
      float droll_dv1 = sin(phi)*tan(theta); // ∂φ/∂(δθ)
      float droll_dv2 = cos(phi)*tan(theta); // ∂φ/∂(δψ)

      float dpitch_dv0 = 0.0f; // ∂θ/∂(δφ)
      float dpitch_dv1 = cos(phi); // ∂θ/∂(δθ)
      float dpitch_dv2 = -sin(phi); // ∂θ/∂(δψ)

      float dyaw_dv0 = 0.0f; // ∂ψ/∂(δφ)
      float dyaw_dv1 = sin(phi)/cos(theta); // ∂ψ/∂(δθ)
      float dyaw_dv2 = cos(phi)/cos(theta); // ∂ψ/∂(δψ)

      h[KC_STATE_D0] = g[0]*dx_droll*droll_dv0 + g[0]*dx_dpitch*dpitch_dv0 + g[0]*dx_dyaw*dyaw_dv0 +
                      g[1]*dy_droll*droll_dv0 + g[1]*dy_dpitch*dpitch_dv0 + g[1]*dy_dyaw*dyaw_dv0 +
                      g[2]*dz_droll*droll_dv0 + g[2]*dz_dpitch*dpitch_dv0 + g[2]*dz_dyaw*dyaw_dv0; // ∂α/∂(δφ)
      h[KC_STATE_D1] = g[0]*dx_droll*droll_dv1 + g[0]*dx_dpitch*dpitch_dv1 + g[0]*dx_dyaw*dyaw_dv1 +
                      g[1]*dy_droll*droll_dv1 + g[1]*dy_dpitch*dpitch_dv1 + g[1]*dy_dyaw*dyaw_dv1 +
                      g[2]*dz_droll*droll_dv1 + g[2]*dz_dpitch*dpitch_dv1 + g[2]*dz_dyaw*dyaw_dv1; // ∂α/∂(δθ)
      h[KC_STATE_D2] = g[0]*dx_droll*droll_dv2 + g[0]*dx_dpitch*dpitch_dv2 + g[0]*dx_dyaw*dyaw_dv2 +
                      g[1]*dy_droll*droll_dv2 + g[1]*dy_dpitch*dpitch_dv2 + g[1]*dy_dyaw*dyaw_dv2 +
                      g[2]*dz_droll*droll_dv2 + g[2]*dz_dpitch*dpitch_dv2 + g[2]*dz_dyaw*dyaw_dv2; // ∂α/∂(δ\ψ)

      arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};
      kalmanCoreScalarUpdate(this, &H, error, sweepInfo->stdDev);
    }
  }
}
