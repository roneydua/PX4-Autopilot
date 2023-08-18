/**
 * @author: roney
 * @date:   2021-08-03T17:16:37-03:00
 * email:  roneyddasilva@gmail.com
 * @File: Sdre.cpp
 * Last modified by:   roney
 * Last modified time: 2021-08-03T17:16:37-03:00
 */

#include "Sdre.h"
Sdre::Sdre(Eigen::MatrixXf &A, Eigen::MatrixXf &B, Eigen::MatrixXf &_Q,
           Eigen::MatrixXf &_R) {
  int n = A.cols();
  int r = B.cols();
  L = Eigen::MatrixXf::Identity(r, n);
  Q = &_Q;
  R = &_R;
  //RICCATI instance or solver
  ricObj = new Riccati(A, B, *Q, *R);
  //Saved the addresses of state and control matrices
  phi = &A;
  gamma = &B;
  E = B * _R.inverse() * B.transpose();
  // large number of iterations for the first iteration
  ricObj->dare_interation(1e-5, 10000);
}
Sdre::~Sdre() {}
/**
 * Updates the Riccati solution and the optimal control L.
 * @return True for success or false to failure.
 */
bool Sdre::update_control() {
  // Check if the algorithm converges with the desired iterations and tolerance number.
  if ((*ricObj).dare_interation()) { // Obtains Riccati's matrix.
    // Updates Kalman's gain matrix
    L = (*ricObj).Ls * (*ricObj).K * (*phi);

    return true;
  } else {
    // Returns false keeping Riccati's last gain.
    return false;
  }
}

/**
 * @brief Compute the close loop gains.
 * @return Eigen::MatrixXcf
 */
Eigen::MatrixXcf Sdre::close_loop_eig() {
  Eigen::MatrixXf closeLoop = *phi - *gamma * L;
  Eigen::EigenSolver<Eigen::MatrixXf> Eigs(closeLoop);
  return Eigs.eigenvalues();
};

;
