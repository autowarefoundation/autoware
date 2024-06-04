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

#ifndef TURN_SIGNALS_DISPLAY_HPP_
#define TURN_SIGNALS_DISPLAY_HPP_
#include "overlay_utils.hpp"

#include <QImage>
#include <QString>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/ros_topic_display.hpp>

#include <autoware_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreTexture.h>

#include <chrono>

namespace autoware_overlay_rviz_plugin
{

class TurnSignalsDisplay
{
public:
  TurnSignalsDisplay();
  void drawArrows(QPainter & painter, const QRectF & backgroundRect, const QColor & color);
  void updateTurnSignalsData(
    const autoware_vehicle_msgs::msg::TurnIndicatorsReport::ConstSharedPtr & msg);
  void updateHazardLightsData(
    const autoware_vehicle_msgs::msg::HazardLightsReport::ConstSharedPtr & msg);

private:
  QImage arrowImage;
  QColor gray = QColor(79, 79, 79);

  int current_turn_signal_;    // Internal variable to store turn signal state
  int current_hazard_lights_;  // Internal variable to store hazard lights state
  QImage coloredImage(const QImage & source, const QColor & color);

  std::chrono::steady_clock::time_point last_toggle_time_;
  bool blink_on_ = false;
  const std::chrono::milliseconds blink_interval_{500};  // Blink interval in milliseconds
};

}  // namespace autoware_overlay_rviz_plugin

#endif  // TURN_SIGNALS_DISPLAY_HPP_
