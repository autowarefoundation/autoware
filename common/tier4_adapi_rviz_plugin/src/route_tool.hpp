//  Copyright 2023 The Autoware Contributors
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#ifndef ROUTE_TOOL_HPP_
#define ROUTE_TOOL_HPP_

#include <rviz_default_plugins/tools/goal_pose/goal_tool.hpp>

namespace tier4_adapi_rviz_plugins
{

class RouteTool : public rviz_default_plugins::tools::GoalTool
{
  Q_OBJECT

public:
  RouteTool();
  void onInitialize() override;
};

}  // namespace tier4_adapi_rviz_plugins

#endif  // ROUTE_TOOL_HPP_
