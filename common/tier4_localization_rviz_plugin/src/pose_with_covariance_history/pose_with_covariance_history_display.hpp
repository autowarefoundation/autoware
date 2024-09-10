// Copyright 2024 Tier IV, Inc.
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

#ifndef POSE_WITH_COVARIANCE_HISTORY__POSE_WITH_COVARIANCE_HISTORY_DISPLAY_HPP_
#define POSE_WITH_COVARIANCE_HISTORY__POSE_WITH_COVARIANCE_HISTORY_DISPLAY_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rviz_common/message_filter_display.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <deque>
#include <memory>
#include <string>
#include <vector>

namespace rviz_rendering
{
class Shape;
class BillboardLine;
class Arrow;
}  // namespace rviz_rendering
namespace rviz_common::properties
{
class ColorProperty;
class FloatProperty;
class IntProperty;
class BoolProperty;
class EnumProperty;
}  // namespace rviz_common::properties

namespace rviz_plugins
{
class PoseWithCovarianceHistory
: public rviz_common::MessageFilterDisplay<geometry_msgs::msg::PoseWithCovarianceStamped>
{
  Q_OBJECT

public:
  PoseWithCovarianceHistory();
  ~PoseWithCovarianceHistory() override;
  PoseWithCovarianceHistory(const PoseWithCovarianceHistory &) = delete;
  PoseWithCovarianceHistory(const PoseWithCovarianceHistory &&) = delete;
  PoseWithCovarianceHistory & operator=(const PoseWithCovarianceHistory &) = delete;
  PoseWithCovarianceHistory & operator=(const PoseWithCovarianceHistory &&) = delete;

protected:
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void update(float wall_dt, float ros_dt) override;

private Q_SLOTS:
  void update_shape_type();

private:  // NOLINT : suppress redundancy warnings
          //          followings cannot be declared with the Q_SLOTS macro
  void subscribe() override;
  void unsubscribe() override;
  void processMessage(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr message) override;
  void update_history();
  void update_shapes();

  std::string target_frame_;
  std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr> history_;
  std::unique_ptr<rviz_rendering::BillboardLine> lines_;
  std::vector<std::unique_ptr<rviz_rendering::Shape>> spheres_;
  std::vector<std::unique_ptr<rviz_rendering::Arrow>> arrows_;
  rclcpp::Time last_stamp_;

  rviz_common::properties::BoolProperty * property_line_view_{};
  rviz_common::properties::FloatProperty * property_line_width_;
  rviz_common::properties::FloatProperty * property_line_alpha_;
  rviz_common::properties::ColorProperty * property_line_color_;
  rviz_common::properties::IntProperty * property_buffer_size_;

  rviz_common::properties::BoolProperty * property_sphere_view_;
  rviz_common::properties::FloatProperty * property_sphere_width_{};
  rviz_common::properties::FloatProperty * property_sphere_alpha_;
  rviz_common::properties::ColorProperty * property_sphere_color_;
  rviz_common::properties::FloatProperty * property_sphere_scale_;

  rviz_common::properties::BoolProperty * property_arrow_view_{};
  rviz_common::properties::FloatProperty * property_arrow_shaft_length_;
  rviz_common::properties::FloatProperty * property_arrow_shaft_diameter_;
  rviz_common::properties::FloatProperty * property_arrow_head_length_;
  rviz_common::properties::FloatProperty * property_arrow_head_diameter_;
  rviz_common::properties::FloatProperty * property_arrow_alpha_;
  rviz_common::properties::ColorProperty * property_arrow_color_;

  rviz_common::properties::BoolProperty * property_path_view_;
  rviz_common::properties::EnumProperty * property_shape_type_;
};

}  // namespace rviz_plugins

#endif  // POSE_WITH_COVARIANCE_HISTORY__POSE_WITH_COVARIANCE_HISTORY_DISPLAY_HPP_
