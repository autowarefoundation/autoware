// Copyright 2023 Autoware Foundation
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

#include "ekf_localizer/warning_message.hpp"

#include <fmt/core.h>

#include <string>

std::string poseDelayStepWarningMessage(
  const double delay_time, const int extend_state_step, const double ekf_dt)
{
  const std::string s =
    "Pose delay exceeds the compensation limit, ignored. "
    "delay: {:.3f}[s], limit = extend_state_step * ekf_dt : {:.3f}[s]";
  return fmt::format(s, delay_time, extend_state_step * ekf_dt);
}

std::string twistDelayStepWarningMessage(
  const double delay_time, const int extend_state_step, const double ekf_dt)
{
  const std::string s =
    "Twist delay exceeds the compensation limit, ignored. "
    "delay: {:.3f}[s], limit = extend_state_step * ekf_dt : {:.3f}[s]";
  return fmt::format(s, delay_time, extend_state_step * ekf_dt);
}

std::string poseDelayTimeWarningMessage(const double delay_time)
{
  const std::string s = "Pose time stamp is inappropriate, set delay to 0[s]. delay = {:.3f}";
  return fmt::format(s, delay_time);
}

std::string twistDelayTimeWarningMessage(const double delay_time)
{
  const std::string s = "Twist time stamp is inappropriate, set delay to 0[s]. delay = {:.3f}";
  return fmt::format(s, delay_time);
}

std::string mahalanobisWarningMessage(const double distance, const double max_distance)
{
  const std::string s = "The Mahalanobis distance {:.4f} is over the limit {:.4f}.";
  return fmt::format(s, distance, max_distance);
}
