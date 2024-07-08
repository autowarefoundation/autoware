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
#ifndef GNSS_POSER__GNSS_POSER_CORE_HPP_
#define GNSS_POSER__GNSS_POSER_CORE_HPP_

#include <component_interface_specs/map.hpp>
#include <component_interface_utils/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tier4_debug_msgs/msg/bool_stamped.hpp>

#include <boost/circular_buffer.hpp>

#include <tf2/transform_datatypes.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <string>

namespace gnss_poser
{
class GNSSPoser : public rclcpp::Node
{
public:
  explicit GNSSPoser(const rclcpp::NodeOptions & node_options);

private:
  using MapProjectorInfo = map_interface::MapProjectorInfo;

  void callback_map_projector_info(const MapProjectorInfo::Message::ConstSharedPtr msg);
  void callback_nav_sat_fix(const sensor_msgs::msg::NavSatFix::ConstSharedPtr nav_sat_fix_msg_ptr);
  void callback_gnss_ins_orientation_stamped(
    const autoware_sensing_msgs::msg::GnssInsOrientationStamped::ConstSharedPtr msg);

  static bool is_fixed(const sensor_msgs::msg::NavSatStatus & nav_sat_status_msg);
  static bool can_get_covariance(const sensor_msgs::msg::NavSatFix & nav_sat_fix_msg);
  static geometry_msgs::msg::Point get_median_position(
    const boost::circular_buffer<geometry_msgs::msg::Point> & position_buffer);
  static geometry_msgs::msg::Point get_average_position(
    const boost::circular_buffer<geometry_msgs::msg::Point> & position_buffer);
  static geometry_msgs::msg::Quaternion get_quaternion_by_heading(const int heading);
  static geometry_msgs::msg::Quaternion get_quaternion_by_position_difference(
    const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Point & prev_point);

  bool get_transform(
    const std::string & target_frame, const std::string & source_frame,
    const geometry_msgs::msg::TransformStamped::SharedPtr transform_stamped_ptr);
  bool get_static_transform(
    const std::string & target_frame, const std::string & source_frame,
    const geometry_msgs::msg::TransformStamped::SharedPtr transform_stamped_ptr,
    const builtin_interfaces::msg::Time & stamp);
  void publish_tf(
    const std::string & frame_id, const std::string & child_frame_id,
    const geometry_msgs::msg::PoseStamped & pose_msg);

  tf2::BufferCore tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;
  tf2_ros::TransformBroadcaster tf2_broadcaster_;

  component_interface_utils::Subscription<MapProjectorInfo>::SharedPtr sub_map_projector_info_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sat_fix_sub_;
  rclcpp::Subscription<autoware_sensing_msgs::msg::GnssInsOrientationStamped>::SharedPtr
    autoware_orientation_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::BoolStamped>::SharedPtr fixed_pub_;

  MapProjectorInfo::Message projector_info_;
  const std::string base_frame_;
  const std::string gnss_base_frame_;
  const std::string map_frame_;
  bool received_map_projector_info_ = false;
  bool use_gnss_ins_orientation_;

  boost::circular_buffer<geometry_msgs::msg::Point> position_buffer_;

  autoware_sensing_msgs::msg::GnssInsOrientationStamped::SharedPtr
    msg_gnss_ins_orientation_stamped_;
  int gnss_pose_pub_method_;
};
}  // namespace gnss_poser

#endif  // GNSS_POSER__GNSS_POSER_CORE_HPP_
