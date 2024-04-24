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

// cspell:ignore pymodel
#ifndef LEARNING_BASED_VEHICLE_MODEL__SIMPLE_PYMODEL_HPP_
#define LEARNING_BASED_VEHICLE_MODEL__SIMPLE_PYMODEL_HPP_

#include "learning_based_vehicle_model/model_connections_helpers.hpp"
#include "learning_based_vehicle_model/submodel_interface.hpp"

#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <string>
#include <vector>

namespace py = pybind11;

/**
 * @class SimplePyModel
 * @brief This class is an interface between C++ and python models.
 */
class SimplePyModel : public SubModelInterface
{
private:
  std::string py_model_import_name;

  int num_inputs_py;
  int num_outputs_py;

  py::object py_model_class;

  std::vector<int>
    map_sig_vec_to_py_model_inputs;  // index in "map_sig_vec_to_py_model_inputs" is index in
                                     // "py_inputs" and value in "map_sig_vec_to_py_model_inputs" is
                                     // index in "signals_vec_names"
  std::vector<int> map_py_model_outputs_to_sig_vec;  // index in "map_py_model_outputs_to_sig_vec"
                                                     // is index in "py_model_outputs" and value in
                                                     // "map_py_model_outputs_to_sig_vec" is index
                                                     // in "signals_vec_names"

  std::vector<char *> py_model_input_names;
  std::vector<char *> py_model_state_names;

public:
  /**
   * @brief constructor
   * @param [in] py_model_import_name_ path to python model
   * @param [in] param_file_path path to saved parameter file of the python sub-model
   * @param [in] py_class_name name of the python class
   */
  SimplePyModel(
    std::string py_model_import_name_, std::string param_file_path, std::string py_class_name);

  /**
   * @brief calculate the next state of a python model
   * @param [in] model_signals_vec all available inputs from PSIM
   */
  std::vector<double> getNextState(
    std::vector<double> model_signals_vec, std::vector<double> model_signals_vec_next) override;

  /**
   * @brief set time step of the model
   * @param [in] dt time step
   */
  void dtSet(double dt) override;

  /**
   * @brief get names of inputs of python model
   */
  std::vector<char *> getInputNames() override;

  /**
   * @brief get names of states of python model
   */
  std::vector<char *> getStateNames() override;

  /**
   * @brief create a map from model signal vector to python model inputs
   * @param [in] signal_vec_names names of signals in model signal vector
   */
  void mapInputs(std::vector<char *> signals_vec_names) override;

  /**
   * @brief create a map from python outputs to model signal vector
   * @param [in] signal_vec_names names of signals in model signal vector
   */
  void mapOutputs(std::vector<char *> signals_vec_names) override;
};

#endif  // LEARNING_BASED_VEHICLE_MODEL__SIMPLE_PYMODEL_HPP_
