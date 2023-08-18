/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * @file SdreControl.hpp
 * @author roneydua (roneyddasilva@gmail.com)
 * @brief A cascaded attitude controller.
 * @version 1
 * @date 2023-08-12 06:30
 *
 ****************************************************************************/

#pragma once
// #include <uORB/uORB.h>
// #include <uORB/uORBTopics.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_attitude.h> // To use quaternions attitude
#include <uORB/topics/vehicle_attitude_setpoint.h> // To use quaternions attitude setpoint
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_angular_acceleration_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>


#include "eigen3/Eigen/Dense"

// #include <lib/SDRE/Sdre.h>
#include <lib/GRUPO_QUAT/GRUPO_QUAT.h>
#include <lib/DRONE/Drone.h>
#include <lib/SDRE/Sdre.h>
using namespace time_literals;
using namespace ekf;
// extern "C" __EXPORT int sdre_control_main(int argc, char *argv[]);

class SdreControl : public ModuleBase<SdreControl>, public ModuleParams {
public:
  SdreControl();

  virtual ~SdreControl() = default;

  /** @see ModuleBase */
  static int task_spawn(int argc, char *argv[]);

  /** @see ModuleBase */
  static SdreControl *instantiate(int argc, char *argv[]);

  /** @see ModuleBase */
  static int custom_command(int argc, char *argv[]);

  /** @see ModuleBase */
  static int print_usage(const char *reason = nullptr);

  /** @see ModuleBase::run() */
  void run() override;

  /** @see ModuleBase::print_status() */
  int print_status() override;

  void print_hello(int n);


private:
  // Sdre _sdre; // instance of sdre control loop.
  /**
   * Check for parameter changes and update them if needed.
   * @param parameter_update_sub uorb subscription to parameter_update
   * @param force for a parameter update
   */
  void parameters_update(bool force = false);

  DEFINE_PARAMETERS((ParamInt<px4::params::SYS_AUTOSTART>)
                        _param_sys_autostart, /**< example parameter */
                    (ParamInt<px4::params::SYS_AUTOCONFIG>)
                        _param_sys_autoconfig /**< another parameter */
  )

  /**
   * @brief Updates the states of plants to calculate gains as SDRE control.
   */
  void update_states();

  /**
   * @brief Update setpoint states of SDRE control class.
   *
   */
  void update_setpoint_states();

  // Subscriptions of body states
  uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
  uORB::Subscription _vehicle_angular_velocity_sub{
      ORB_ID(vehicle_angular_velocity)};
  // Subcriptions of setpoints
  uORB::Subscription _vehicle_attitude_setpoint_sub{
      ORB_ID(vehicle_attitude_setpoint)};
  uORB::Subscription _vehicle_rates_setpoint_sub{
      ORB_ID(vehicle_rates_setpoint)};

  uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update),
                                                   1_s};
  // variables of control




  float gravidade = 9.80f;
  uint8_t index_alt = 1;
  /*! Referência de velocidade translacional */
  Eigen::Vector3f r = Eigen::Vector3f::Zero();
  /*! Quaternion alvo de atitude */
  Eigen::Matrix<float, 4, 2> qa = Eigen::Matrix<float, 4, 2>::Zero(4, 2);
  /*! Quaternio de erro de atitude*/
  Eigen::Vector4f qe{1, 0, 0, 0};
  /*! Velocidade angular alvo */
  Eigen::Vector3f wa = Eigen::Vector3f::Zero();
  /*! Ponteiros de velocidade angular */
  /*! Psi alvo */
  float psi = 0.0f;
  /*! Velocidade Psi alvo */
  float diffPsi = 0.0f;
  Eigen::Matrix3f Lt = Eigen::Matrix3f::Zero();

  /*! Matriz de ponderação do main_control rotacional. */
  Eigen::MatrixXf Rr =
      (Eigen::Vector3f() << 1e1, 1e1, 1e1).finished().asDiagonal();
  /*! Matriz de ponderação do estado rotacional. */
  Eigen::MatrixXf Qr = (Eigen::VectorXf(6) << 1e1, 1e1, 1e1, 1e1, 1e1, 1e1)
                           .finished()
                           .asDiagonal();
  /* Contoles */
  /*! Vetor de main_control da dinâmica translacional */
  Eigen::Vector3f ut = Eigen::Vector3f::Zero();
  /*! Vetor de tração específica e momentos no corpo.
    @note que a tração já esta considerando a massa.
   */
  Eigen::Vector4f u = Eigen::Vector4f::Zero();

  float oldPsi = 0;

  Drone *drone;
  Sdre *sdre;

  void compute_translational_control();
  void computeRotationalControl();
  bool negative_q0 = false;
  float TSum = 10.0f;

  hrt_abstime _last_run{0};
};
