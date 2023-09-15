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
 *
 * @file SdreControl.cpp
 * @author roneydua (roneyddasilva@gmail.com)
 * @brief This module calculates the moments of inertia with the SDRE control
 *technique. For linear algebra calculations the Eigen library is used. To
 *install it, you can use the script available at
 *https://github.com/roneydua/comandosNovaInstalacao/blob/master/install_eigen.sh
 * @version 1
 * @date 2023-08-12 06:24
 ****************************************************************************/

#include "SdreControl.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>

int SdreControl::print_status() {
  PX4_INFO("Running SDRE control");
  // TODO: print additional runtime information about the state of the module

  return 0;
}

int SdreControl::custom_command(int argc, char *argv[]) {
  /*
  if (!is_running()) {
          print_usage("not running");
          return 1;
  }

  // additional custom commands can be handled like this:
  if (!strcmp(argv[0], "do-something")) {
          get_instance()->do_something();
          return 0;
  }
   */

  return print_usage("unknown command");
}

int SdreControl::task_spawn(int argc, char *argv[]) {
  _task_id =
      px4_task_spawn_cmd("module", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT, 1024,
                         (px4_main_t)&run_trampoline, (char *const *)argv);

  if (_task_id < 0) {
    _task_id = -1;
    return -errno;
  }

  return 0;
}

SdreControl *SdreControl::instantiate(int argc, char *argv[]) {
  // int example_param;
  // bool example_flag = false;
  bool error_flag = false;

  int myoptind = 1;
  int ch;
  const char *myoptarg = nullptr;

  // parse CLI arguments
  while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
    switch (ch) {
    case 'p':
      PX4_INFO("In p option");
      // example_param = (int)strtol(myoptarg, nullptr, 10);
      break;

    case 'f':
      PX4_INFO("In f flag");
      // example_flag = true;
      break;

    case '?':
      PX4_INFO("In error flag option");
      // error_flag = true;
      break;

    default:
      PX4_WARN("unrecognized flag");
      error_flag = true;
      break;
    }
  }

  if (error_flag) {
    return nullptr;
  }

  SdreControl *instance = new SdreControl();

  if (instance == nullptr) {
    PX4_ERR("alloc failed");
  }
  return instance;
}

SdreControl::SdreControl(bool vtol)
    : ModuleParams(nullptr),
      _vehicle_torque_setpoint_pub(
          vtol ? ORB_ID(vehicle_torque_setpoint_virtual_mc)
               : ORB_ID(vehicle_torque_setpoint)),
      _vehicle_thrust_setpoint_pub(
          vtol ? ORB_ID(vehicle_thrust_setpoint_virtual_mc)
               : ORB_ID(vehicle_thrust_setpoint)) {
  drone = new Drone(0.01f);
  drone->update_state_matrices(0.01f);
  sdre = new Sdre(drone->matAR, drone->matBR, Qr, Rr);
  PRINT_MAT(drone->matAR);
  PRINT_MAT(drone->matBR);
  PRINT_MAT(Qr);
  PRINT_MAT(Rr);
  PRINT_MAT(sdre->ricObj->K);
  PRINT_MAT(sdre->L);
}

/** @copydoc update_states */
float SdreControl::update_states() {

  vehicle_attitude_s _vehicle_attitude{};
  vehicle_angular_velocity_s _vehicle_angular_velocity{};

  _vehicle_attitude_sub.update(&_vehicle_attitude); // to get actual quaternion
  _vehicle_angular_velocity_sub.update(
      &_vehicle_angular_velocity); // to get actual angula velocity
  // Compute the elapsed time.
  const hrt_abstime now = _vehicle_angular_velocity.timestamp_sample;
  /*! elapsed time.*/
  const float dt = (now - _last_run) * 1e-6f;
  _last_run = now;
  // copy atual states to drone class
  drone->q = Eigen::Map<Eigen::MatrixXf>(_vehicle_attitude.q, 4, 1);
  drone->w = Eigen::Map<Eigen::MatrixXf>(_vehicle_angular_velocity.xyz, 3, 1);
  drone->update_state_matrices(dt);
  return dt;
}
/** @copydoc update_setpoint_states */
void SdreControl::update_setpoint_states() {
  vehicle_attitude_setpoint_s _vehicle_attitude_setpoint{};
  vehicle_rates_setpoint_s _vehicle_rates_setpoint{};
  _vehicle_attitude_setpoint_sub.update(&_vehicle_attitude_setpoint);
  _vehicle_rates_setpoint_sub.update(&_vehicle_attitude_setpoint);

  q_sp = Eigen::Map<Eigen::MatrixXf>(_vehicle_attitude_setpoint.q_d, 4, 1);
  w_sp(0) = _vehicle_rates_setpoint.roll;
  w_sp(1) = _vehicle_rates_setpoint.pitch;
  w_sp(2) = _vehicle_rates_setpoint.yaw;
  // w_sp(0) = Eigen::Map<Eigen::MatrixXf>(_vehicle_rates_setpoint.xyz, 3, 1);
}

void SdreControl::run() {

  // Example: run the loop synchronized to the sensor_combined topic publication
  int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
  px4_pollfd_struct_t fds[1];
  fds[0].fd = sensor_combined_sub;
  fds[0].events = POLLIN;

  // initialize parameters
  parameters_update(true);

  while (!should_exit()) {

    // Updates the states
    update_states();
    // Updates the setpoint states
    update_setpoint_states();
    // Calculates Torques
    sdre->update_control();
    // PRINT_MAT(sdre->L);
    Eigen::VectorXf _x(6);
    qe = (ekf::S_l(drone->q)).transpose() * q_sp;
    // _qI = ;
    if (drone->q(0) < 0.0f) {
      _x << qe.tail(3), drone->w - w_sp;
    } else {
      _x << -qe.tail(3), drone->w - w_sp;
    }
    u = -sdre->L * _x;
    // public te moments
    vehicle_torque_setpoint_s vehicle_torque_setpoint{};
    vehicle_torque_setpoint.xyz[0] = PX4_ISFINITE(u(0)) ? u(0) : 0.f;
    vehicle_torque_setpoint.xyz[1] = PX4_ISFINITE(u(1)) ? u(1) : 0.f;
    vehicle_torque_setpoint.xyz[2] = PX4_ISFINITE(u(2)) ? u(2) : 0.f;
    _vehicle_torque_setpoint_pub.publish(vehicle_torque_setpoint);
    // wait for up to 1000ms for data
    int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

    if (pret == 0) {
      // Timeout: let the loop run anyway, don't do `continue` here

    } else if (pret < 0) {
      // this is undesirable but not much we can do
      PX4_ERR("poll error %d, %d", pret, errno);
      px4_usleep(50000);
      continue;

    } else if (fds[0].revents & POLLIN) {

      struct sensor_combined_s sensor_combined;
      orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor_combined);
      // TODO: do something with the data...
    }

    parameters_update();
  }
  orb_unsubscribe(sensor_combined_sub);
}

void SdreControl::parameters_update(bool force) {
  // check for parameter updates
  if (_parameter_update_sub.updated() || force) {
    // clear update
    parameter_update_s update;
    _parameter_update_sub.copy(&update);

    // update parameters from storage
    updateParams();
  }
}

int SdreControl::print_usage(const char *reason) {
  if (reason) {
    PX4_WARN("%s\n", reason);
  }
  PRINT_MODULE_DESCRIPTION(
      R"DESCR_STR(
### Description
This implement the nonlinar control SDRE.
)DESCR_STR");

  PRINT_MODULE_USAGE_NAME("module", "SDRE");
  PRINT_MODULE_USAGE_COMMAND("start");
  PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
  PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter",
                               true);
  PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

  return 0;
}
void SdreControl::print_hello(int n) {
  for (int i = 0; i < n; i++) {
    PX4_INFO("ola \n");
  }
}



extern "C" __EXPORT int mc_sdre_control_main(int argc, char *argv[]) {
  return SdreControl::main(argc, argv);
}
