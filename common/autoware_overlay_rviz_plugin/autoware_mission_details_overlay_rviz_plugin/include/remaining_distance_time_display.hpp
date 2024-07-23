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

#ifndef REMAINING_DISTANCE_TIME_DISPLAY_HPP_
#define REMAINING_DISTANCE_TIME_DISPLAY_HPP_
#include "overlay_utils.hpp"

#include <QImage>
#include <QString>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/ros_topic_display.hpp>

#include <autoware_internal_msgs/msg/mission_remaining_distance_time.hpp>

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreTexture.h>

namespace autoware::mission_details_overlay_rviz_plugin
{

class RemainingDistanceTimeDisplay
{
public:
  RemainingDistanceTimeDisplay();
  void drawRemainingDistanceTimeDisplay(
    QPainter & painter, const QRectF & backgroundRect, const QColor & text_color);
  void updateRemainingDistanceTimeData(
    const autoware_internal_msgs::msg::MissionRemainingDistanceTime::ConstSharedPtr & msg);

private:
  double remaining_distance_;  // Internal variable to store remaining distance
  double remaining_time_;      // Internal variable to store remaining time

  QImage icon_dist_;
  QImage icon_dist_scaled_;
  QImage icon_time_;
  QImage icon_time_scaled_;
};

}  // namespace autoware::mission_details_overlay_rviz_plugin

#endif  // REMAINING_DISTANCE_TIME_DISPLAY_HPP_
