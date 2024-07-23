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

#ifndef MISSION_DETAILS_DISPLAY_HPP_
#define MISSION_DETAILS_DISPLAY_HPP_
#ifndef Q_MOC_RUN
#include "overlay_utils.hpp"
#include "remaining_distance_time_display.hpp"

#include <QImage>
#include <QString>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreTexture.h>

#include <memory>
#include <mutex>
#endif

namespace autoware::mission_details_overlay_rviz_plugin
{
class MissionDetailsDisplay : public rviz_common::Display
{
  Q_OBJECT
public:
  MissionDetailsDisplay();
  ~MissionDetailsDisplay() override;

protected:
  void onInitialize() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;
  void onEnable() override;
  void onDisable() override;

private Q_SLOTS:
  void update_size();
  void topic_updated_remaining_distance_time();

private:
  std::mutex mutex_;
  autoware::mission_details_overlay_rviz_plugin::OverlayObject::SharedPtr overlay_;
  rviz_common::properties::IntProperty * property_width_;
  rviz_common::properties::IntProperty * property_height_;
  rviz_common::properties::IntProperty * property_right_;
  rviz_common::properties::IntProperty * property_top_;
  rviz_common::properties::ColorProperty * property_bg_color_;
  rviz_common::properties::FloatProperty * property_bg_alpha_;
  rviz_common::properties::ColorProperty * property_text_color_;

  std::unique_ptr<rviz_common::properties::RosTopicProperty>
    remaining_distance_time_topic_property_;

  void draw_rounded_rect(QPainter & painter, const QRectF & backgroundRect);
  void setupRosSubscriptions();

  std::unique_ptr<RemainingDistanceTimeDisplay> remaining_distance_time_display_;

  rclcpp::Subscription<autoware_internal_msgs::msg::MissionRemainingDistanceTime>::SharedPtr
    remaining_distance_time_sub_;

  std::mutex property_mutex_;

  void cb_remaining_distance_time(
    const autoware_internal_msgs::msg::MissionRemainingDistanceTime::ConstSharedPtr & msg);
  void draw_widget(QImage & hud);
};
}  // namespace autoware::mission_details_overlay_rviz_plugin

#endif  // MISSION_DETAILS_DISPLAY_HPP_
