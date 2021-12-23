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

#ifndef SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_IDEAL_STEER_ACC_GEARED_HPP_
#define SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_IDEAL_STEER_ACC_GEARED_HPP_

#include <iostream>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/LU"

#include "simple_planning_simulator/vehicle_model/sim_model_interface.hpp"

/**
 * @class SimModelIdealSteerAccGeared
 * @brief calculate ideal steering dynamics
 */
class SimModelIdealSteerAccGeared : public SimModelInterface
{
public:
  /**
   * @brief constructor
   * @param [in] wheelbase vehicle wheelbase length [m]
   */
  explicit SimModelIdealSteerAccGeared(float64_t wheelbase);

  /**
   * @brief destructor
   */
  ~SimModelIdealSteerAccGeared() = default;

private:
  enum IDX
  {
    X = 0,
    Y,
    YAW,
    VX
  };
  enum IDX_U
  {
    AX_DES = 0,
    STEER_DES,
  };

  const float64_t wheelbase_;  //!< @brief vehicle wheelbase length
  float64_t current_acc_;  //!< @brief current_acc with gear consideration

  /**
   * @brief get vehicle position x
   */
  float64_t getX() override;

  /**
   * @brief get vehicle position y
   */
  float64_t getY() override;

  /**
   * @brief get vehicle angle yaw
   */
  float64_t getYaw() override;

  /**
   * @brief get vehicle longitudinal velocity
   */
  float64_t getVx() override;

  /**
   * @brief get vehicle lateral velocity
   */
  float64_t getVy() override;

  /**
   * @brief get vehicle longiudinal acceleration
   */
  float64_t getAx() override;

  /**
   * @brief get vehicle angular-velocity wz
   */
  float64_t getWz() override;

  /**
   * @brief get vehicle steering angle
   */
  float64_t getSteer() override;

  /**
   * @brief update vehicle states
   * @param [in] dt delta time [s]
   */
  void update(const float64_t & dt) override;

  /**
   * @brief calculate derivative of states with ideal steering model
   * @param [in] state current model state
   * @param [in] input input vector to model
   */
  Eigen::VectorXd calcModel(const Eigen::VectorXd & state, const Eigen::VectorXd & input) override;

  /**
   * @brief calculate velocity with considering current velocity and gear
   * @param [in] state current state
   * @param [in] gear current gear (defined in autoware_auto_msgs/GearCommand)
   */
  float64_t calcVelocityWithGear(const Eigen::VectorXd & state, const uint8_t gear) const;
};

#endif  // SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_IDEAL_STEER_ACC_GEARED_HPP_
