// Copyright 2021 The Autoware Foundation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_DELAY_STEER_ACC_HPP_
#define SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_DELAY_STEER_ACC_HPP_

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/LU"
#include "simple_planning_simulator/vehicle_model/sim_model_interface.hpp"

#include <deque>
#include <iostream>
#include <queue>

class SimModelDelaySteerAcc : public SimModelInterface
{
public:
  /**
   * @brief constructor
   * @param [in] vx_lim velocity limit [m/s]
   * @param [in] steer_lim steering limit [rad]
   * @param [in] vx_rate_lim acceleration limit [m/ss]
   * @param [in] steer_rate_lim steering angular velocity limit [rad/ss]
   * @param [in] wheelbase vehicle wheelbase length [m]
   * @param [in] dt delta time information to set input buffer for delay
   * @param [in] acc_delay time delay for accel command [s]
   * @param [in] acc_time_constant time constant for 1D model of accel dynamics
   * @param [in] steer_delay time delay for steering command [s]
   * @param [in] steer_time_constant time constant for 1D model of steering dynamics
   */
  SimModelDelaySteerAcc(
    double vx_lim, double steer_lim, double vx_rate_lim, double steer_rate_lim, double wheelbase,
    double dt, double acc_delay, double acc_time_constant, double steer_delay,
    double steer_time_constant);

  /**
   * @brief default destructor
   */
  ~SimModelDelaySteerAcc() = default;

private:
  const double MIN_TIME_CONSTANT;  //!< @brief minimum time constant

  enum IDX {
    X = 0,
    Y,
    YAW,
    VX,
    STEER,
    ACCX,
  };
  enum IDX_U {
    ACCX_DES = 0,
    STEER_DES,
    DRIVE_SHIFT,
  };

  const double vx_lim_;          //!< @brief velocity limit [m/s]
  const double vx_rate_lim_;     //!< @brief acceleration limit [m/ss]
  const double steer_lim_;       //!< @brief steering limit [rad]
  const double steer_rate_lim_;  //!< @brief steering angular velocity limit [rad/s]
  const double wheelbase_;       //!< @brief vehicle wheelbase length [m]

  std::deque<double> acc_input_queue_;    //!< @brief buffer for accel command
  std::deque<double> steer_input_queue_;  //!< @brief buffer for steering command
  const double acc_delay_;                //!< @brief time delay for accel command [s]
  const double acc_time_constant_;        //!< @brief time constant for accel dynamics
  const double steer_delay_;              //!< @brief time delay for steering command [s]
  const double steer_time_constant_;      //!< @brief time constant for steering dynamics

  /**
   * @brief set queue buffer for input command
   * @param [in] dt delta time
   */
  void initializeInputQueue(const double & dt);

  /**
   * @brief get vehicle position x
   */
  double getX() override;

  /**
   * @brief get vehicle position y
   */
  double getY() override;

  /**
   * @brief get vehicle angle yaw
   */
  double getYaw() override;

  /**
   * @brief get vehicle velocity vx
   */
  double getVx() override;

  /**
   * @brief get vehicle lateral velocity
   */
  double getVy() override;

  /**
   * @brief get vehicle longitudinal acceleration
   */
  double getAx() override;

  /**
   * @brief get vehicle angular-velocity wz
   */
  double getWz() override;

  /**
   * @brief get vehicle steering angle
   */
  double getSteer() override;

  /**
   * @brief update vehicle states
   * @param [in] dt delta time [s]
   */
  void update(const double & dt) override;

  /**
   * @brief calculate derivative of states with time delay steering model
   * @param [in] state current model state
   * @param [in] input input vector to model
   */
  Eigen::VectorXd calcModel(const Eigen::VectorXd & state, const Eigen::VectorXd & input) override;
};

#endif  // SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_DELAY_STEER_ACC_HPP_
