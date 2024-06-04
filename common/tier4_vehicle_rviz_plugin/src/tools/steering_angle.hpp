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

#ifndef TOOLS__STEERING_ANGLE_HPP_
#define TOOLS__STEERING_ANGLE_HPP_

#include <memory>
#include <mutex>

#ifndef Q_MOC_RUN
#include "jsk_overlay_utils.hpp"

#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/ros_topic_display.hpp>

#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#endif

namespace rviz_plugins
{
class SteeringAngleDisplay
: public rviz_common::RosTopicDisplay<autoware_vehicle_msgs::msg::SteeringReport>
{
  Q_OBJECT

public:
  SteeringAngleDisplay();
  ~SteeringAngleDisplay() override;

  void onInitialize() override;
  void onDisable() override;
  void onEnable() override;

private Q_SLOTS:
  void updateVisualization();

protected:
  void update(float wall_dt, float ros_dt) override;
  void processMessage(
    const autoware_vehicle_msgs::msg::SteeringReport::ConstSharedPtr msg_ptr) override;

  jsk_rviz_plugins::OverlayObject::Ptr overlay_;
  rviz_common::properties::ColorProperty * property_text_color_;
  rviz_common::properties::IntProperty * property_left_;
  rviz_common::properties::IntProperty * property_top_;
  rviz_common::properties::IntProperty * property_length_;
  rviz_common::properties::FloatProperty * property_handle_angle_scale_;
  rviz_common::properties::IntProperty * property_value_height_offset_;
  rviz_common::properties::FloatProperty * property_value_scale_;
  QPixmap handle_image_;
  // QImage hud_;

private:
  std::mutex mutex_;
  autoware_vehicle_msgs::msg::SteeringReport::ConstSharedPtr last_msg_ptr_;
};

}  // namespace rviz_plugins

#endif  // TOOLS__STEERING_ANGLE_HPP_
