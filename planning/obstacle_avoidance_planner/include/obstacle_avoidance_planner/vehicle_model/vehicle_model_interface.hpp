// Copyright 2018-2019 Autoware Foundation
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

/**
 * @file vehicle_model_interface.h
 * @brief vehicle model interface class
 * @author Takamasa Horibe
 * @date 2019.05.01
 */

#ifndef OBSTACLE_AVOIDANCE_PLANNER__VEHICLE_MODEL__VEHICLE_MODEL_INTERFACE_HPP_
#define OBSTACLE_AVOIDANCE_PLANNER__VEHICLE_MODEL__VEHICLE_MODEL_INTERFACE_HPP_

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Sparse"

#include <vector>

/**
 * @class vehicle model class
 * @brief calculate model-related values
 */
class VehicleModelInterface
{
protected:
  const int dim_x_;                 // !< @brief dimension of kinematics x
  const int dim_u_;                 // !< @brief dimension of input u
  const int dim_y_;                 // !< @brief dimension of output y
  double velocity_;                 // !< @brief vehicle velocity
  double curvature_;                // !< @brief curvature on the linearized point on path
  double wheel_base_;               // !< @brief wheel base of vehicle
  double steer_limit_;              // !< @brief vehicle velocity
  double center_offset_from_base_;  // !< @brief length from base lin to optimization center [m]

public:
  /**
   * @brief constructor
   * @param [in] dim_x dimension of kinematics x
   * @param [in] dim_u dimension of input u
   * @param [in] dim_y dimension of output y
   */
  VehicleModelInterface(int dim_x, int dim_u, int dim_y, double wheel_base, double steer_limit);

  /**
   * @brief destructor
   */
  virtual ~VehicleModelInterface() = default;

  /**
   * @brief get kinematics x dimension
   * @return kinematics dimension
   */
  int getDimX();

  /**
   * @brief get input u dimension
   * @return input dimension
   */
  int getDimU();

  /**
   * @brief get output y dimension
   * @return output dimension
   */
  int getDimY();

  void updateCenterOffset(const double center_offset_from_base);

  /**
   * @brief set curvature
   * @param [in] curvature curvature on the linearized point on path
   */
  void setCurvature(const double curvature);

  /**
   * @brief calculate discrete kinematics equation matrix of x_k+1 = Ad * x_k + Bd * uk + Wd
   * @param [out] Ad coefficient matrix
   * @param [out] Bd coefficient matrix
   * @param [out] Wd coefficient matrix
   * @param [in] ds discretization arc length
   */
  virtual void calculateStateEquationMatrix(
    Eigen::MatrixXd & Ad, Eigen::MatrixXd & Bd, Eigen::MatrixXd & Wd, const double ds) = 0;

  /**
   * @brief calculate discrete observation matrix of y_k = Cd * x_k
   * @param [out] Cd coefficient matrix
   */
  virtual void calculateObservationMatrix(Eigen::MatrixXd & Cd) = 0;

  /**
   * @brief calculate discrete observation matrix of y_k = Cd * x_k
   * @param [out] Cd_vec sparse matrix information of coefficient matrix
   */
  virtual void calculateObservationSparseMatrix(std::vector<Eigen::Triplet<double>> & Cd_vec) = 0;

  /**
   * @brief calculate reference input
   * @param [out] reference input
   */
  virtual void calculateReferenceInput(Eigen::MatrixXd * Uref) = 0;
};

#endif  // OBSTACLE_AVOIDANCE_PLANNER__VEHICLE_MODEL__VEHICLE_MODEL_INTERFACE_HPP_
