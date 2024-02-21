// Copyright 2024 The Autoware Contributors
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

#include "mrm_goal.hpp"

namespace rviz_plugins
{

MrmGoalTool::MrmGoalTool()
{
  shortcut_key_ = 'm';
}

void MrmGoalTool::onInitialize()
{
  GoalTool::onInitialize();
  setName("MRM Goal Pose");
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::MrmGoalTool, rviz_common::Tool)
