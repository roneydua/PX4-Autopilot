/**
 * @author: Roney Silva <roney>
 * @date:   16-Aug-2021
 * Email:  roneyddasilva@gmail.com
 * Project: quadrirrotorUFABC
 * @file: Drone.cpp
 * Last modified by:   roney
 * Last modified time: 25-Aug-2021
 */

#include "Drone.h"

Drone::Drone(float dt) {


  half_dt = 0.5f * dt;
  // matAT = (1.0f - dt * dx / massa) * Eigen::Matrix3f::Identity();
  matAT.bottomRightCorner(3, 3) =
      (1.0f - dt * dx / massa) * Eigen::Matrix3f::Identity();
  // bloco integral
  matAT.topRightCorner(3, 3) = (1.0f + dt) * Eigen::Matrix3f::Identity();

  matBT.bottomRightCorner(3, 3) = dt * Eigen::Matrix3f::Identity();
  matBR.block<3, 3>(3, 0) = dt * inverse_matrix_inertia;
}
/**
 * @brief Updates state-dependent matrices.
 *
 * @param dt elapsed time of loop
 */
void Drone::update_state_matrices(float dt) {
  half_dt = 0.5f * dt;
  float _q0 = 0.0f;
  // HACK
  _q0 = q(0);
  if (abs(q(0)) > 0.01f) {
    matAR(0, 0) = 1.0f;
    matAR(1, 1) = 1.0f;
    matAR(2, 2) = 1.0f;
    matAR.topRightCorner(3, 3) =
        half_dt * (_q0 * Eigen::Matrix3f::Identity() + ekf::skew(q.tail(3)));
  } else {
    matAR(0, 0) = -half_dt * w(0) * q(1) + 1.0f;
    matAR(1, 1) = -half_dt * w(1) * q(2) + 1.0f;
    matAR(2, 2) = -half_dt * w(2) * q(3) + 1.0f;
    // _q0 = q(0);
    matAR.topRightCorner(3, 3) = half_dt * ekf::skew(q.tail(3));
    matAR(0, 3) = half_dt * (q(1) * q(1));
    matAR(1, 4) = half_dt * (q(2) * q(2));
    matAR(2, 5) = half_dt * (q(3) * q(3));
  }
  matAR.block<3, 3>(3, 3) =
      -dt * inverse_matrix_inertia * ekf::skew(w) * matrix_inertia;
  matAR(3, 3) = 1.0f;
  matAR(4, 4) = 1.0f;
  matAR(5, 5) = 1.0f;
}
