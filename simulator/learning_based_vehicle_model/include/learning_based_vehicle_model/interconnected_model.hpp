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

#ifndef LEARNING_BASED_VEHICLE_MODEL__INTERCONNECTED_MODEL_HPP_
#define LEARNING_BASED_VEHICLE_MODEL__INTERCONNECTED_MODEL_HPP_

#include "learning_based_vehicle_model/model_connections_helpers.hpp"
#include "learning_based_vehicle_model/simple_pymodel.hpp"
#include "learning_based_vehicle_model/submodel_interface.hpp"

#include <dlfcn.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

namespace py = pybind11;

class __attribute__((visibility("default"))) InterconnectedModel
{
  // Vector of unique names of inputs and outputs of sub-models
  std::vector<char *> signals_vec_names;
  std::vector<double> model_signals_vec;
  int num_signals;

  std::vector<std::unique_ptr<SubModelInterface>> submodels;

  // index in "map_in_to_sig_vec" is index in "py_inputs" and value in "map_in_to_sig_vec" is index
  // in "all_variables_names"
  std::vector<int> map_in_to_sig_vec;

  // index in "map_sig_vec_to_out" is index in "py_model_outputs" and value in "map_sig_vec_to_out"
  // is index in "all_variables_names"
  std::vector<int> map_sig_vec_to_out;

public:
  py::scoped_interpreter guard{};  // start the interpreter and keep it alive

  /**
   * @brief constructor
   */
  InterconnectedModel()
  {
    // Initialize python library
    // cspell:ignore libpython
    // Manually load libpython3.10.so as we need it for python.h.
    dlopen("libpython3.10.so", RTLD_GLOBAL | RTLD_NOW);
    /*
    More about the line above here:
      https://stackoverflow.com/questions/60719987/embedding-python-which-uses-numpy-in-c-doesnt-work-in-library-dynamically-loa
      https://mail.python.org/pipermail/new-bugs-announce/2008-November/003322.html
      https://stackoverflow.com/questions/67891197/ctypes-cpython-39-x86-64-linux-gnu-so-undefined-symbol-pyfloat-type-in-embedd
      https://man7.org/linux/man-pages/man3/dlopen.3.html
    */
  }

private:
  /**
   * @brief create a mapping between vector of signal input names from PSIM to vector of signals
   * @param [in] in_names vector of signal input names from PSIM
   */
  void mapInputs(std::vector<char *> in_names);

  /**
   * @brief create a mapping between vector of signal output names from PSIM to vector of signals
   * @param [in] out_names vector of signal output names from PSIM
   */
  void mapOutputs(std::vector<char *> out_names);

  /**
   * @brief add unique names to the vector of signal names
   * @param [in] names vector of signal names
   */
  void addNamesToSigVec(const std::vector<char *> & names);

  /**
   * @brief create of signal names from all sub-models and PSIM signal names
   */
  void getSignalNames(std::vector<char *> in_names, std::vector<char *> out_names);

public:
  /**
   * @brief automatically create connections between PSIM and all of the sub-models
   * @param [in] in_names string names of inputs available from PSIM
   * @param [in] out_names string names of outputs required by PSIM
   */
  void generateConnections(std::vector<char *> in_names, std::vector<char *> out_names);

  /**
   * @brief add a sub-model consisting of base + error model
   * @param [in] submodel_desc descriptor of the sub-model
   */
  void addSubmodel(std::tuple<std::string, std::string, std::string> submodel_desc);

  /**
   * @brief set a new model state if it was changed using PSIM interface (mainly position and
   * orientation)
   * @param [in] new_state new state set by PSIM
   */
  void initState(std::vector<double> new_state);

  /**
   * @brief set time step for all the models
   * @param [in] dt time step
   */
  void dtSet(double dt);

  /**
   * @brief compute next step of the PSIM model using python sub-models
   * @param [in] psim_input vector of input values provided by PSIM
   */
  std::vector<double> updatePyModel(std::vector<double> psim_input);
};

#endif  // LEARNING_BASED_VEHICLE_MODEL__INTERCONNECTED_MODEL_HPP_
