// Copyright 2021 Tier IV, Inc. All rights reserved.
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

#ifndef PATH_FOOTPRINT__DISPLAY_HPP_
#define PATH_FOOTPRINT__DISPLAY_HPP_

#include <path_footprint/display_base.hpp>
#include <rviz_rendering/objects/movable_text.hpp>

#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>

#include <memory>
#include <utility>
#include <vector>

namespace rviz_plugins
{
class AutowarePathWithLaneIdFootprintDisplay
: public AutowarePathFootBaseprintDisplay<autoware_auto_planning_msgs::msg::PathWithLaneId>
{
  Q_OBJECT

public:
  AutowarePathWithLaneIdFootprintDisplay();

private:
  void resetDetail() override;
  void preprocessMessageDetail(
    const autoware_auto_planning_msgs::msg::PathWithLaneId::ConstSharedPtr msg_ptr) override;
  void processMessageDetail(
    const autoware_auto_planning_msgs::msg::PathWithLaneId::ConstSharedPtr msg_ptr,
    const size_t p_idx) override;

  rviz_common::properties::BoolProperty property_lane_id_view_;
  rviz_common::properties::FloatProperty property_lane_id_scale_;

  using LaneIdObject =
    std::pair<std::unique_ptr<Ogre::SceneNode>, std::unique_ptr<rviz_rendering::MovableText>>;
  std::vector<LaneIdObject> lane_id_obj_ptrs_;
};
class AutowarePathFootprintDisplay
: public AutowarePathFootBaseprintDisplay<autoware_auto_planning_msgs::msg::Path>
{
  Q_OBJECT
};

class AutowareTrajectoryFootprintDisplay
: public AutowarePathFootBaseprintDisplay<autoware_auto_planning_msgs::msg::Trajectory>
{
  Q_OBJECT
};
}  // namespace rviz_plugins

#endif  // PATH_FOOTPRINT__DISPLAY_HPP_
