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

#ifndef TRAFFIC_DISPLAY_HPP_
#define TRAFFIC_DISPLAY_HPP_
#include "overlay_utils.hpp"

#include <QImage>
#include <QString>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/ros_topic_display.hpp>

#include <autoware_perception_msgs/msg/traffic_signal.hpp>
#include <autoware_perception_msgs/msg/traffic_signal_array.hpp>
#include <autoware_perception_msgs/msg/traffic_signal_element.hpp>

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreTexture.h>

namespace awf_2d_overlay_vehicle
{

class TrafficDisplay
{
public:
  TrafficDisplay();
  void drawTrafficLightIndicator(QPainter & painter, const QRectF & backgroundRect);
  void updateTrafficLightData(
    const autoware_perception_msgs::msg::TrafficSignalArray::ConstSharedPtr & msg);
  autoware_perception_msgs::msg::TrafficSignalArray current_traffic_;

private:
  QImage traffic_light_image_;
  // yellow #CFC353
  QColor yellow = QColor(207, 195, 83);
  // red #CF5353
  QColor red = QColor(207, 83, 83);
  // green #53CF5F
  QColor green = QColor(83, 207, 95);
  // gray #C2C2C2
  QColor gray = QColor(194, 194, 194);

  QImage coloredImage(const QImage & source, const QColor & color);
};

}  // namespace awf_2d_overlay_vehicle

#endif  // TRAFFIC_DISPLAY_HPP_
