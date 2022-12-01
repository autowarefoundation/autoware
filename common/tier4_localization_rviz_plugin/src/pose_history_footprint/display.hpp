// Copyright 2022 Tier IV, Inc.
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

#ifndef POSE_HISTORY_FOOTPRINT__DISPLAY_HPP_
#define POSE_HISTORY_FOOTPRINT__DISPLAY_HPP_

#include <rviz_common/message_filter_display.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

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

using vehicle_info_util::VehicleInfo;
using vehicle_info_util::VehicleInfoUtil;

class PoseHistoryFootprint
: public rviz_common::MessageFilterDisplay<geometry_msgs::msg::PoseStamped>
{
  Q_OBJECT

public:
  PoseHistoryFootprint();
  ~PoseHistoryFootprint() override;
  PoseHistoryFootprint(const PoseHistoryFootprint &) = delete;
  PoseHistoryFootprint(const PoseHistoryFootprint &&) = delete;
  PoseHistoryFootprint & operator=(const PoseHistoryFootprint &) = delete;
  PoseHistoryFootprint & operator=(const PoseHistoryFootprint &&) = delete;

protected:
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void updateFootprint();

private Q_SLOTS:
  void updateVisualization();
  void updateVehicleInfo();

private:
  void subscribe() override;
  void unsubscribe() override;
  void processMessage(const geometry_msgs::msg::PoseStamped::ConstSharedPtr message) override;
  void updateHistory(const geometry_msgs::msg::PoseStamped::ConstSharedPtr message);

  std::string target_frame_;
  std::deque<geometry_msgs::msg::PoseStamped::ConstSharedPtr> history_;
  rclcpp::Time last_stamp_;

  // pose history
  rviz_common::properties::IntProperty * property_buffer_size_;

  // trajectory footprint
  Ogre::ManualObject * trajectory_footprint_manual_object_;
  rviz_common::properties::BoolProperty * property_trajectory_footprint_view_;
  rviz_common::properties::ColorProperty * property_trajectory_footprint_color_;
  rviz_common::properties::FloatProperty * property_trajectory_footprint_alpha_;
  rviz_common::properties::FloatProperty * property_vehicle_length_;
  rviz_common::properties::FloatProperty * property_vehicle_width_;
  rviz_common::properties::FloatProperty * property_rear_overhang_;
  rviz_common::properties::FloatProperty * property_offset_;
  rviz_common::properties::FloatProperty * property_interval_;

  struct VehicleFootprintInfo
  {
    VehicleFootprintInfo(const float l, const float w, const float r)
    : length(l), width(w), rear_overhang(r)
    {
    }
    float length, width, rear_overhang;
  };
  std::shared_ptr<VehicleInfo> vehicle_info_;
  std::shared_ptr<VehicleFootprintInfo> vehicle_footprint_info_;

  geometry_msgs::msg::PoseStamped::ConstSharedPtr last_msg_ptr_;
};

}  // namespace rviz_plugins

#endif  // POSE_HISTORY_FOOTPRINT__DISPLAY_HPP_
