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

#include "learning_based_vehicle_model/interconnected_model.hpp"

void InterconnectedModel::mapInputs(std::vector<char *> in_names)
{
  // index in "map_in_to_sig_vec" is index in "in_names" and value in "map_in_to_sig_vec" is index
  // in "signals_vec_names"
  map_in_to_sig_vec = createConnectionsMap(signals_vec_names, in_names);
}

void InterconnectedModel::mapOutputs(std::vector<char *> out_names)
{
  // index in "map_sig_vec_to_out" is index in "out_names" and value in "map_sig_vec_to_out" is
  // index in "signals_vec_names"
  map_sig_vec_to_out = createConnectionsMap(signals_vec_names, out_names);
}

void InterconnectedModel::addNamesToSigVec(const std::vector<char *> & names)
{
  // Check if the name is already in the vector. If not add it.
  for (char * name : names) {
    if (
      std::find(signals_vec_names.begin(), signals_vec_names.end(), name) ==
      signals_vec_names.end()) {
      signals_vec_names.push_back(name);
    }
  }
}

void InterconnectedModel::getSignalNames(
  std::vector<char *> in_names, std::vector<char *> out_names)
{
  addNamesToSigVec(in_names);
  addNamesToSigVec(out_names);
  for (auto & submodel : submodels) {
    addNamesToSigVec(submodel->getInputNames());
    addNamesToSigVec(submodel->getStateNames());
  }
}

void InterconnectedModel::generateConnections(
  std::vector<char *> in_names, std::vector<char *> out_names)
{
  // Create vector of signal names
  getSignalNames(in_names, out_names);
  num_signals = signals_vec_names.size();
  // Init vector of signal values
  for (int i = 0; i < num_signals; i++) model_signals_vec.push_back(0);

  // For every sub-model create mapping from vector of signals to inputs and outputs
  for (auto & submodel : submodels) {
    submodel->mapInputs(signals_vec_names);
    submodel->mapOutputs(signals_vec_names);
  }

  // Create mapping from vector of signals to inputs and outputs of PSIM
  mapInputs(in_names);
  mapOutputs(out_names);
}

void InterconnectedModel::addSubmodel(
  std::tuple<std::string, std::string, std::string> submodel_desc)
{
  const auto [lib_path, param_path, class_name] = submodel_desc;
  auto new_model = new SimplePyModel(lib_path, param_path, class_name);
  submodels.push_back(std::unique_ptr<SimplePyModel>(new_model));
}

void InterconnectedModel::initState(std::vector<double> new_state)
{
  bool state_changed_externally = false;

  // Check if some state was changed externally
  for (size_t PSIM_STATE_IDX = 0; PSIM_STATE_IDX < new_state.size(); PSIM_STATE_IDX++) {
    if (
      abs(model_signals_vec[map_sig_vec_to_out[PSIM_STATE_IDX]] - new_state[PSIM_STATE_IDX]) >
      1e-6) {
      state_changed_externally = true;
      break;
    }
  }

  if (state_changed_externally) {
    // Reinitialize model
    // Currently initializing model to zero -> TODO find a way how to initialize them to some
    // other default values
    std::fill(model_signals_vec.begin(), model_signals_vec.end(), 0.0);

    for (size_t PSIM_STATE_IDX = 0; PSIM_STATE_IDX < new_state.size(); PSIM_STATE_IDX++) {
      model_signals_vec[map_sig_vec_to_out[PSIM_STATE_IDX]] = new_state[PSIM_STATE_IDX];
    }
  }
}

void InterconnectedModel::dtSet(double dt)
{
  for (auto & submodel : submodels) {
    submodel->dtSet(dt);
  }
}

std::vector<double> InterconnectedModel::updatePyModel(std::vector<double> psim_input)
{
  // map input to vector of all variables
  for (size_t PSIM_INPUT_IDX = 0; PSIM_INPUT_IDX < psim_input.size(); PSIM_INPUT_IDX++) {
    model_signals_vec[map_in_to_sig_vec[PSIM_INPUT_IDX]] = psim_input[PSIM_INPUT_IDX];
  }

  // Compute forward pass through all models (order should not matter)
  std::vector<double> model_signals_vec_next = model_signals_vec;
  for (auto & submodel : submodels) {
    model_signals_vec_next = submodel->getNextState(model_signals_vec, model_signals_vec_next);
  }

  // Map vector of all variables to
  std::vector<double> psim_next_state;
  for (size_t PSIM_STATE_IDX = 0; PSIM_STATE_IDX < map_sig_vec_to_out.size(); PSIM_STATE_IDX++) {
    psim_next_state.push_back(model_signals_vec_next[map_sig_vec_to_out[PSIM_STATE_IDX]]);
  }

  // Update vector of all variables
  model_signals_vec = model_signals_vec_next;

  return psim_next_state;
}
