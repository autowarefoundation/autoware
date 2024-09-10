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

#ifndef POSE_HISTORY__POSE_HISTORY_DISPLAY_HPP_
#define POSE_HISTORY__POSE_HISTORY_DISPLAY_HPP_

#include <rviz_common/message_filter_display.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <deque>
#include <memory>
#include <string>

namespace rviz_rendering
{
class BillboardLine;
}  // namespace rviz_rendering
namespace rviz_common::properties
{
class ColorProperty;
class FloatProperty;
class IntProperty;
class BoolProperty;
}  // namespace rviz_common::properties

namespace rviz_plugins
{
class PoseHistory : public rviz_common::MessageFilterDisplay<geometry_msgs::msg::PoseStamped>
{
  Q_OBJECT

public:
  PoseHistory();
  ~PoseHistory() override;
  PoseHistory(const PoseHistory &) = delete;
  PoseHistory(const PoseHistory &&) = delete;
  PoseHistory & operator=(const PoseHistory &) = delete;
  PoseHistory & operator=(const PoseHistory &&) = delete;

protected:
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void update(float wall_dt, float ros_dt) override;

private:
  void subscribe() override;
  void unsubscribe() override;
  void processMessage(const geometry_msgs::msg::PoseStamped::ConstSharedPtr message) override;
  void update_history();
  void update_lines();

  std::string target_frame_;
  std::deque<geometry_msgs::msg::PoseStamped::ConstSharedPtr> history_;
  std::unique_ptr<rviz_rendering::BillboardLine> lines_;
  rclcpp::Time last_stamp_;

  rviz_common::properties::IntProperty * property_buffer_size_;
  rviz_common::properties::BoolProperty * property_line_view_;
  rviz_common::properties::FloatProperty * property_line_width_;
  rviz_common::properties::FloatProperty * property_line_alpha_;
  rviz_common::properties::ColorProperty * property_line_color_;
};

}  // namespace rviz_plugins

#endif  // POSE_HISTORY__POSE_HISTORY_DISPLAY_HPP_
