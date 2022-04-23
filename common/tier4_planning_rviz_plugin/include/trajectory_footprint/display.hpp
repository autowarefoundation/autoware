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

#ifndef TRAJECTORY_FOOTPRINT__DISPLAY_HPP_
#define TRAJECTORY_FOOTPRINT__DISPLAY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/parse_color.hpp>
#include <rviz_common/validate_floats.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>

#include <OgreBillboardSet.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <memory>

namespace rviz_plugins
{
using vehicle_info_util::VehicleInfo;
using vehicle_info_util::VehicleInfoUtil;

class AutowareTrajectoryFootprintDisplay
: public rviz_common::MessageFilterDisplay<autoware_auto_planning_msgs::msg::Trajectory>
{
  Q_OBJECT

public:
  AutowareTrajectoryFootprintDisplay();
  virtual ~AutowareTrajectoryFootprintDisplay();

  void onInitialize() override;
  void reset() override;

private Q_SLOTS:
  void updateVisualization();
  void updateVehicleInfo();

protected:
  void processMessage(
    const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr msg_ptr) override;
  Ogre::ManualObject * trajectory_footprint_manual_object_;
  rviz_common::properties::BoolProperty * property_trajectory_footprint_view_;
  rviz_common::properties::ColorProperty * property_trajectory_footprint_color_;
  rviz_common::properties::FloatProperty * property_trajectory_footprint_alpha_;
  rviz_common::properties::FloatProperty * property_vehicle_length_;
  rviz_common::properties::FloatProperty * property_vehicle_width_;
  rviz_common::properties::FloatProperty * property_rear_overhang_;

  Ogre::ManualObject * trajectory_point_manual_object_;
  rviz_common::properties::BoolProperty * property_trajectory_point_view_;
  rviz_common::properties::ColorProperty * property_trajectory_point_color_;
  rviz_common::properties::FloatProperty * property_trajectory_point_alpha_;
  rviz_common::properties::FloatProperty * property_trajectory_point_radius_;
  rviz_common::properties::FloatProperty * property_trajectory_point_offset_;

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

private:
  autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr last_msg_ptr_;
  bool validateFloats(const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr & msg_ptr);
};

}  // namespace rviz_plugins

#endif  // TRAJECTORY_FOOTPRINT__DISPLAY_HPP_
