// Copyright 2024 The Autoware Foundation.
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

#ifndef LEARNING_BASED_VEHICLE_MODEL__SUBMODEL_INTERFACE_HPP_
#define LEARNING_BASED_VEHICLE_MODEL__SUBMODEL_INTERFACE_HPP_

#include <vector>

class SubModelInterface
{
public:
  /**
   * @brief set time step of the model
   * @param [in] dt time step
   */
  virtual void dtSet(double dt) = 0;

  /**
   * @brief get names of inputs of python model
   */
  virtual std::vector<char *> getInputNames() = 0;

  /**
   * @brief get names of states of python model
   */
  virtual std::vector<char *> getStateNames() = 0;

  /**
   * @brief create a map from model signal vector to python model inputs
   * @param [in] signal_vec_names names of signals in model signal vector
   */
  virtual void mapInputs(std::vector<char *> signals_vec_names) = 0;

  /**
   * @brief create a map from python outputs to model signal vector
   * @param [in] signal_vec_names names of signals in model signal vector
   */
  virtual void mapOutputs(std::vector<char *> signals_vec_names) = 0;

  /**
   * @brief calculate the next state of this submodule
   * @param [in] model_signals_vec values of signals in model signal vector
   * @param [in] model_signals_vec_next values of signals in model signal vector to update
   */
  virtual std::vector<double> getNextState(
    std::vector<double> model_signals_vec, std::vector<double> model_signals_vec_next) = 0;
};

#endif  // LEARNING_BASED_VEHICLE_MODEL__SUBMODEL_INTERFACE_HPP_
