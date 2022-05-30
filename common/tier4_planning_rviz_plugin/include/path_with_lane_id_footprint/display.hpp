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

#ifndef PATH_WITH_LANE_ID_FOOTPRINT__DISPLAY_HPP_
#define PATH_WITH_LANE_ID_FOOTPRINT__DISPLAY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/parse_color.hpp>
#include <rviz_common/validate_floats.hpp>
#include <rviz_rendering/objects/movable_text.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <OgreBillboardSet.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <memory>
#include <utility>
#include <vector>

namespace rviz_plugins
{
using vehicle_info_util::VehicleInfo;
using vehicle_info_util::VehicleInfoUtil;

class AutowarePathWithLaneIdFootprintDisplay
: public rviz_common::MessageFilterDisplay<autoware_auto_planning_msgs::msg::PathWithLaneId>
{
  Q_OBJECT

public:
  AutowarePathWithLaneIdFootprintDisplay();
  virtual ~AutowarePathWithLaneIdFootprintDisplay();

  void onInitialize() override;
  void reset() override;

private Q_SLOTS:
  void updateVisualization();
  void updateVehicleInfo();

protected:
  void processMessage(
    const autoware_auto_planning_msgs::msg::PathWithLaneId::ConstSharedPtr msg_ptr) override;
  Ogre::ManualObject * path_footprint_manual_object_;
  rviz_common::properties::BoolProperty * property_path_footprint_view_;
  rviz_common::properties::ColorProperty * property_path_footprint_color_;
  rviz_common::properties::FloatProperty * property_path_footprint_alpha_;
  rviz_common::properties::FloatProperty * property_vehicle_length_;
  rviz_common::properties::FloatProperty * property_vehicle_width_;
  rviz_common::properties::FloatProperty * property_rear_overhang_;
  rviz_common::properties::BoolProperty * property_lane_id_view_;
  rviz_common::properties::FloatProperty * property_lane_id_scale_;
  struct VehicleFootprintInfo
  {
    VehicleFootprintInfo(const float l, const float w, const float r)
    : length(l), width(w), rear_overhang(r)
    {
    }
    float length, width, rear_overhang;
  };
  std::shared_ptr<VehicleInfo> vehicle_info_;
  std::shared_ptr<VehicleFootprintInfo> vehicle_footprint_info_;

  using LaneIdObject =
    std::pair<std::unique_ptr<Ogre::SceneNode>, std::unique_ptr<rviz_rendering::MovableText>>;
  std::vector<LaneIdObject> lane_id_obj_ptrs_;

private:
  autoware_auto_planning_msgs::msg::PathWithLaneId::ConstSharedPtr last_msg_ptr_;
  bool validateFloats(
    const autoware_auto_planning_msgs::msg::PathWithLaneId::ConstSharedPtr & msg_ptr);

  void allocateLaneIdObjects(const std::size_t size);
};

}  // namespace rviz_plugins

#endif  // PATH_WITH_LANE_ID_FOOTPRINT__DISPLAY_HPP_
