/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file vehicle_model_interface.h
 * @brief vehicle model interface class
 * @author Takamasa Horibe
 * @date 2019.05.01
 */

#pragma once
#include <eigen3/Eigen/Core>

/** 
 * @class vehicle model class
 * @brief calculate model-related values
 */
class VehicleModelInterface
{
protected:
  const int dim_x_;  //!< @brief dimension of state x
  const int dim_u_;  //!< @brief dimension of input u
  const int dim_y_;  //!< @brief dimension of output y
  double velocity_;  //!< @brief vehicle velocity
  double curvature_; //!< @brief curvature on the linearized point on path

public:
  /**
   * @brief constructor
   * @param [in] dim_x dimension of state x
   * @param [in] dim_u dimension of input u
   * @param [in] dim_y dimension of output y
   */
  VehicleModelInterface(int dim_x, int dim_u, int dim_y);

  /**
   * @brief get state x dimension
   * @return state dimension
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

  /**
   * @brief set velocity
   * @param [in] vehicle velocity
   */
  void setVelocity(const double &velocity);

  /**
   * @brief set curvature
   * @param [in] curvature curvature on the linearized point on path
   */
  void setCurvature(const double &curvature);

  /**
   * @brief calculate discrete model matrix of x_k+1 = Ad * xk + Bd * uk + Wd, yk = Cd * xk 
   * @param [out] Ad coefficient matrix
   * @param [out] Bd coefficient matrix
   * @param [out] Cd coefficient matrix
   * @param [out] Wd coefficient matrix
   * @param [in] dt Discretization time
   */
  virtual void calculateDiscreteMatrix(Eigen::MatrixXd &Ad, Eigen::MatrixXd &Bd, Eigen::MatrixXd &Cd,
                                       Eigen::MatrixXd &Wd, const double &dt) = 0;

  /**
   * @brief calculate reference input
   * @param [out] reference input
   */
  virtual void calculateReferenceInput(Eigen::MatrixXd &Uref) = 0;
};
