/**
 * @author: Roney Silva <roney>
 * @date:   16-Aug-2021
 * Email:  roneyddasilva@gmail.com
 * Project: quadrirrotorUFABC
 * @file: Drone.h
 * Last modified by:   roney
 * Last modified time: 25-Aug-2021
 */
#ifndef DRONE_H
  #define DRONE_H
  #include "eigen3/Eigen/Dense"
  #include <lib/GRUPO_QUAT/GRUPO_QUAT.h>
  #include <lib/pid/pid.h>
  #include <iostream>
using namespace std;
  #define PRINT_MAT(X) cout << #X << ":\n" << X << "\n"

// #define MOTOR_MODEL_SIMPLE
class Drone {

private:
  /*! Braço do quadro. */
  const float L = 0.225f;
  float half_dt;

public:
  /*! Coeficiente de Arrasto*/
  float dx = 0.25f;
  /*! "Massa do quadrirrotor em kg. Sem os apoios e as proteções massa=1.135,
   * coso contrário, massa = 1.363" */
  float massa = 1.136f;

  // float massa = 1.0f;
  /**
   * @brief Construct a new Drone object
   *
   * @param _dt Tempo do discretização da planta para loop do controle.
   */
  Drone(float _dt);
  /*! Matriz de momentos de inércia
  @note É utilizada a classe DiagonalMatrix para economia de memoria (n
  vezes menos). Todavia deve-se ter cuidado com a limitação de operações desta
  classe.
  */
  const Eigen::DiagonalMatrix<float, 3> matrix_inertia{25e-3f, 25e-3f, 30e-3f};
  /*! Inversa da matriz de momentos de inércia.
  @note São utilizadas as classes DiagonalMatrix para economia de memoria (n
  vezes menos). Todavia deve-se ter cuidado com a limitação de operações desta
  classe.
  */
  const Eigen::DiagonalMatrix<float, 3> inverse_matrix_inertia =
      matrix_inertia.inverse();
  /*! Posicao translacional */
  Eigen::Vector3f p{0, 0, 0};
  /*! Velocidade translacional */
  Eigen::Vector3f v{0, 0, 0};
  /*! Quaternion de atitude. */
  Eigen::Vector4f q{1.0f, 0.0f, 0.0f, 0.0f};
  /*! Vetor de velocidade angular. */
  Eigen::Vector3f w{0.0f, 0.0f, 0.0f};
  /*! Matrix de estados translacional discreta. */
  // Eigen::MatrixXf matAT = Eigen::MatrixXf::Identity(3, 3);
  Eigen::MatrixXf matAT = Eigen::MatrixXf::Identity(6, 6);
  /*! Matriz de controle translacional discreta. */
  // Eigen::MatrixXf matBT = Eigen::MatrixXf::Identity(3, 3);
  Eigen::MatrixXf matBT = Eigen::MatrixXf::Zero(6, 3);
  // TEST: INTEGRAL CONTROL
  /*! Matriz de estados rotacional discreta. */
  Eigen::MatrixXf matAR = Eigen::MatrixXf::Identity(6, 6);
  /*! Matriz de controle rotacional discreta. */
  Eigen::MatrixXf matBR = Eigen::MatrixXf::Zero(6, 3);
  void update_state_matrices(float dt);
};
#endif
/* DRONE_H */
