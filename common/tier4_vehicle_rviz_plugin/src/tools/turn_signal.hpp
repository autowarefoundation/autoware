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

#ifndef TOOLS__TURN_SIGNAL_HPP_
#define TOOLS__TURN_SIGNAL_HPP_

#include <memory>
#include <mutex>

#ifndef Q_MOC_RUN
#include "jsk_overlay_utils.hpp"

#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/ros_topic_display.hpp>

#include <autoware_auto_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp>
#endif

namespace rviz_plugins
{
class TurnSignalDisplay
: public rviz_common::RosTopicDisplay<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>
{
  Q_OBJECT

public:
  TurnSignalDisplay();
  ~TurnSignalDisplay() override;

  void onInitialize() override;
  void onDisable() override;
  void onEnable() override;

private Q_SLOTS:
  void updateVisualization();

protected:
  void update(float wall_dt, float ros_dt) override;
  void processMessage(
    const autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ConstSharedPtr msg_ptr) override;
  jsk_rviz_plugins::OverlayObject::Ptr overlay_;
  rviz_common::properties::IntProperty * property_left_;
  rviz_common::properties::IntProperty * property_top_;
  rviz_common::properties::IntProperty * property_width_;
  rviz_common::properties::IntProperty * property_height_;
  // QImage hud_;

private:
  QPointF right_arrow_polygon_[7];
  QPointF left_arrow_polygon_[7];

  std::mutex mutex_;
  autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ConstSharedPtr last_msg_ptr_;
};

}  // namespace rviz_plugins

#endif  // TOOLS__TURN_SIGNAL_HPP_
