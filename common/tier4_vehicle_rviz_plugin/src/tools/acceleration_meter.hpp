// Copyright 2023 Tier IV, Inc.
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

#ifndef TOOLS__ACCELERATION_METER_HPP_
#define TOOLS__ACCELERATION_METER_HPP_

#include <memory>
#include <mutex>

#ifndef Q_MOC_RUN
#include "jsk_overlay_utils.hpp"

#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>

#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#endif

namespace rviz_plugins
{
class AccelerationMeterDisplay
: public rviz_common::RosTopicDisplay<geometry_msgs::msg::AccelWithCovarianceStamped>
{
  Q_OBJECT

public:
  AccelerationMeterDisplay();
  ~AccelerationMeterDisplay() override;

  void onInitialize() override;
  void onDisable() override;
  void onEnable() override;

private Q_SLOTS:
  void updateVisualization();

protected:
  void update(float wall_dt, float ros_dt) override;
  void processMessage(
    const geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr msg_ptr) override;
  jsk_rviz_plugins::OverlayObject::Ptr overlay_;
  rviz_common::properties::ColorProperty * property_normal_text_color_;
  rviz_common::properties::ColorProperty * property_emergency_text_color_;
  rviz_common::properties::IntProperty * property_left_;
  rviz_common::properties::IntProperty * property_top_;
  rviz_common::properties::IntProperty * property_length_;
  rviz_common::properties::IntProperty * property_value_height_offset_;
  rviz_common::properties::FloatProperty * property_value_scale_;
  rviz_common::properties::FloatProperty * property_emergency_threshold_max_;
  rviz_common::properties::FloatProperty * property_emergency_threshold_min_;
  // QImage hud_;

private:
  static constexpr float meter_min_acceleration_ = -10.0f;
  static constexpr float meter_max_acceleration_ = 10.0f;
  static constexpr float meter_min_angle_ = tier4_autoware_utils::deg2rad(40.f);
  static constexpr float meter_max_angle_ = tier4_autoware_utils::deg2rad(320.f);
  static constexpr int line_width_ = 2;
  static constexpr int hand_width_ = 4;
  struct Line  // for drawLine
  {
    int x0, y0;
    int x1, y1;
  };
  Line min_range_line_;
  Line max_range_line_;
  struct Arc  // for drawArc
  {
    int x0, y0;
    int x1, y1;
    float start_angle, end_angle;
  };
  Arc inner_arc_;
  Arc outer_arc_;

  std::mutex mutex_;
  geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr last_msg_ptr_;
};

}  // namespace rviz_plugins

#endif  // TOOLS__ACCELERATION_METER_HPP_
