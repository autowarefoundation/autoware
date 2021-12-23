// Copyright 2021 The Autoware Foundation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_INTERFACE_HPP_
#define SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_INTERFACE_HPP_

#include "eigen3/Eigen/Core"
#include "autoware_auto_vehicle_msgs/msg/gear_command.hpp"
#include "common/types.hpp"

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware::common::types::bool8_t;

/**
 * @class SimModelInterface
 * @brief simple_planning_simulator vehicle model class to calculate vehicle dynamics
 */
class SimModelInterface
{
protected:
  const int dim_x_;        //!< @brief dimension of state x
  const int dim_u_;        //!< @brief dimension of input u
  Eigen::VectorXd state_;  //!< @brief vehicle state vector
  Eigen::VectorXd input_;  //!< @brief vehicle input vector

  //!< @brief gear command defined in autoware_auto_msgs/GearCommand
  uint8_t gear_ = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE;

public:
  /**
   * @brief constructor
   * @param [in] dim_x dimension of state x
   * @param [in] dim_u dimension of input u
   */
  SimModelInterface(int dim_x, int dim_u);

  /**
   * @brief destructor
   */
  ~SimModelInterface() = default;

  /**
   * @brief get state vector of model
   * @param [out] state state vector
   */
  void getState(Eigen::VectorXd & state);

  /**
   * @brief get input vector of model
   * @param [out] input input vector
   */
  void getInput(Eigen::VectorXd & input);

  /**
   * @brief set state vector of model
   * @param [in] state state vector
   */
  void setState(const Eigen::VectorXd & state);

  /**
   * @brief set input vector of model
   * @param [in] input input vector
   */
  void setInput(const Eigen::VectorXd & input);

  /**
   * @brief set gear
   * @param [in] gear gear command defined in autoware_auto_msgs/GearCommand
   */
  void setGear(const uint8_t gear);

  /**
   * @brief update vehicle states with Runge-Kutta methods
   * @param [in] dt delta time [s]
   * @param [in] input vehicle input
   */
  void updateRungeKutta(const float64_t & dt, const Eigen::VectorXd & input);

  /**
   * @brief update vehicle states with Euler methods
   * @param [in] dt delta time [s]
   * @param [in] input vehicle input
   */
  void updateEuler(const float64_t & dt, const Eigen::VectorXd & input);

  /**
   * @brief update vehicle states
   * @param [in] dt delta time [s]
   */
  virtual void update(const float64_t & dt) = 0;

  /**
   * @brief get vehicle position x
   */
  virtual float64_t getX() = 0;

  /**
   * @brief get vehicle position y
   */
  virtual float64_t getY() = 0;

  /**
   * @brief get vehicle angle yaw
   */
  virtual float64_t getYaw() = 0;

  /**
   * @brief get vehicle velocity vx
   */
  virtual float64_t getVx() = 0;

  /**
   * @brief get vehicle lateral velocity
   */
  virtual float64_t getVy() = 0;

  /**
   * @brief get vehicle longiudinal acceleration
   */
  virtual float64_t getAx() = 0;

  /**
   * @brief get vehicle angular-velocity wz
   */
  virtual float64_t getWz() = 0;

  /**
   * @brief get vehicle steering angle
   */
  virtual float64_t getSteer() = 0;

  /**
   * @brief get state vector dimension
   */
  inline int getDimX() {return dim_x_;}

  /**
   * @brief get input vector demension
   */
  inline int getDimU() {return dim_u_;}

  /**
   * @brief calculate derivative of states with vehicle model
   * @param [in] state current model state
   * @param [in] input input vector to model
   */
  virtual Eigen::VectorXd calcModel(
    const Eigen::VectorXd & state, const Eigen::VectorXd & input) = 0;
};

#endif  // SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_INTERFACE_HPP_
