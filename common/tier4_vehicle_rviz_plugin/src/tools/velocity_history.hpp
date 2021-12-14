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

#ifndef TOOLS__VELOCITY_HISTORY_HPP_
#define TOOLS__VELOCITY_HISTORY_HPP_

#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/parse_color.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <rviz_common/validate_floats.hpp>

#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>

#include <OgreColourValue.h>
#include <OgreManualObject.h>
#include <OgreVector3.h>

#include <deque>
#include <memory>
#include <mutex>
#include <tuple>

namespace rviz_plugins
{
class VelocityHistoryDisplay
: public rviz_common::RosTopicDisplay<autoware_auto_vehicle_msgs::msg::VelocityReport>
{
  Q_OBJECT

public:
  VelocityHistoryDisplay();
  ~VelocityHistoryDisplay() override;

  void onInitialize() override;
  void reset() override;

private Q_SLOTS:
  void updateVisualization();

protected:
  void update(float wall_dt, float ros_dt) override;
  void processMessage(
    const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr msg_ptr) override;
  std::unique_ptr<Ogre::ColourValue> setColorDependsOnVelocity(
    const double vel_max, const double cmd_vel);
  std::unique_ptr<Ogre::ColourValue> gradation(
    const QColor & color_min, const QColor & color_max, const double ratio);
  Ogre::ManualObject * velocity_manual_object_;
  rviz_common::properties::FloatProperty * property_velocity_timeout_;
  rviz_common::properties::FloatProperty * property_velocity_alpha_;
  rviz_common::properties::FloatProperty * property_velocity_scale_;
  rviz_common::properties::BoolProperty * property_velocity_color_view_;
  rviz_common::properties::ColorProperty * property_velocity_color_;
  rviz_common::properties::FloatProperty * property_vel_max_;

private:
  std::deque<
    std::tuple<autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr, Ogre::Vector3>>
    histories_;
  bool validateFloats(
    const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr & msg_ptr);
  std::mutex mutex_;
};

}  // namespace rviz_plugins

#endif  // TOOLS__VELOCITY_HISTORY_HPP_
