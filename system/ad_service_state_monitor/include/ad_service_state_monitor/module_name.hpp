// Copyright 2020 Tier IV, Inc.
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

#ifndef AD_SERVICE_STATE_MONITOR__MODULE_NAME_HPP_
#define AD_SERVICE_STATE_MONITOR__MODULE_NAME_HPP_

struct ModuleName
{
  static constexpr const char * map = "map";
  static constexpr const char * sensing = "sensing";
  static constexpr const char * localization = "localization";
  static constexpr const char * perception = "perception";
  static constexpr const char * planning = "planning";
  static constexpr const char * control = "control";
  static constexpr const char * vehicle = "vehicle";
  static constexpr const char * system = "system";
};

#endif  // AD_SERVICE_STATE_MONITOR__MODULE_NAME_HPP_
