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

#ifndef SPEED_LIMIT_DISPLAY_HPP_
#define SPEED_LIMIT_DISPLAY_HPP_
#include "overlay_utils.hpp"

#include <QImage>
#include <QString>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/ros_topic_display.hpp>

#include "autoware_vehicle_msgs/msg/velocity_report.hpp"
#include <tier4_planning_msgs/msg/velocity_limit.hpp>

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreTexture.h>

namespace autoware_overlay_rviz_plugin
{

class SpeedLimitDisplay
{
public:
  SpeedLimitDisplay();
  void drawSpeedLimitIndicator(
    QPainter & painter, const QRectF & backgroundRect, const QColor & color,
    const QColor & light_color, const QColor & dark_color, const QColor & bg_color,
    const float bg_alpha);
  void updateSpeedLimitData(const tier4_planning_msgs::msg::VelocityLimit::ConstSharedPtr msg);
  void updateSpeedData(const autoware_vehicle_msgs::msg::VelocityReport::ConstSharedPtr & msg);

private:
  float current_limit;   // Internal variable to store current gear
  float current_speed_;  // Internal variable to store current speed
};

}  // namespace autoware_overlay_rviz_plugin

#endif  // SPEED_LIMIT_DISPLAY_HPP_
