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

#ifndef SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_INTERFACE_HPP_
#define SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_INTERFACE_HPP_

#include <Eigen/Core>

#include "autoware_vehicle_msgs/msg/gear_command.hpp"
#include "tier4_vehicle_msgs/msg/actuation_status_stamped.hpp"

#include <optional>

/**
 * @class SimModelInterface
 * @brief simple_planning_simulator vehicle model class to calculate vehicle dynamics
 */
class SimModelInterface
{
protected:
  using ActuationStatusStamped = tier4_vehicle_msgs::msg::ActuationStatusStamped;

  const int dim_x_;        //!< @brief dimension of state x
  const int dim_u_;        //!< @brief dimension of input u
  Eigen::VectorXd state_;  //!< @brief vehicle state vector
  Eigen::VectorXd input_;  //!< @brief vehicle input vector

  //!< @brief gear command defined in autoware_vehicle_msgs/GearCommand
  uint8_t gear_ = autoware_vehicle_msgs::msg::GearCommand::DRIVE;

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
   * @param [in] gear gear command defined in autoware_vehicle_msgs/GearCommand
   */
  void setGear(const uint8_t gear);

  /**
   * @brief update vehicle states with Runge-Kutta methods
   * @param [in] dt delta time [s]
   * @param [in] input vehicle input
   */
  void updateRungeKutta(const double & dt, const Eigen::VectorXd & input);

  /**
   * @brief update vehicle states with Euler methods
   * @param [in] dt delta time [s]
   * @param [in] input vehicle input
   */
  void updateEuler(const double & dt, const Eigen::VectorXd & input);

  /**
   * @brief update vehicle states
   * @param [in] dt delta time [s]
   */
  virtual void update(const double & dt) = 0;

  /**
   * @brief get vehicle position x
   */
  virtual double getX() = 0;

  /**
   * @brief get vehicle position y
   */
  virtual double getY() = 0;

  /**
   * @brief get vehicle angle yaw
   */
  virtual double getYaw() = 0;

  /**
   * @brief get vehicle velocity vx
   */
  virtual double getVx() = 0;

  /**
   * @brief get vehicle lateral velocity
   */
  virtual double getVy() = 0;

  /**
   * @brief get vehicle longitudinal acceleration
   */
  virtual double getAx() = 0;

  /**
   * @brief get vehicle angular-velocity wz
   */
  virtual double getWz() = 0;

  /**
   * @brief get vehicle steering angle
   */
  virtual double getSteer() = 0;

  /**
   * @brief get vehicle gear
   */
  uint8_t getGear() const;

  /**
   * @brief get state vector dimension
   */
  inline int getDimX() { return dim_x_; }

  /**
   * @brief get input vector dimension
   */
  inline int getDimU() { return dim_u_; }

  /**
   * @brief is publish actuation status enabled
   */
  virtual bool shouldPublishActuationStatus() const { return false; }

  /*
   * @brief get actuation status
   */
  virtual std::optional<ActuationStatusStamped> getActuationStatus() const { return std::nullopt; }

  /**
   * @brief calculate derivative of states with vehicle model
   * @param [in] state current model state
   * @param [in] input input vector to model
   */
  virtual Eigen::VectorXd calcModel(
    const Eigen::VectorXd & state, const Eigen::VectorXd & input) = 0;
};

#endif  // SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_INTERFACE_HPP_
