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

#include "gnss_poser/gnss_poser_core.hpp"

#include <autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace gnss_poser
{
GNSSPoser::GNSSPoser(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("gnss_poser", node_options),
  tf2_listener_(tf2_buffer_),
  tf2_broadcaster_(*this),
  base_frame_(declare_parameter("base_frame", "base_link")),
  gnss_frame_(declare_parameter("gnss_frame", "gnss")),
  gnss_base_frame_(declare_parameter("gnss_base_frame", "gnss_base_link")),
  map_frame_(declare_parameter("map_frame", "map")),
  use_gnss_ins_orientation_(declare_parameter("use_gnss_ins_orientation", true)),
  plane_zone_(declare_parameter<int>("plane_zone", 9)),
  msg_gnss_ins_orientation_stamped_(
    std::make_shared<autoware_sensing_msgs::msg::GnssInsOrientationStamped>())
{
  int coordinate_system =
    declare_parameter("coordinate_system", static_cast<int>(CoordinateSystem::MGRS));
  coordinate_system_ = static_cast<CoordinateSystem>(coordinate_system);

  nav_sat_fix_origin_.latitude = declare_parameter("latitude", 0.0);
  nav_sat_fix_origin_.longitude = declare_parameter("longitude", 0.0);
  nav_sat_fix_origin_.altitude = declare_parameter("altitude", 0.0);

  int buff_epoch = declare_parameter("buff_epoch", 1);
  position_buffer_.set_capacity(buff_epoch);

  nav_sat_fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    "fix", rclcpp::QoS{1}, std::bind(&GNSSPoser::callbackNavSatFix, this, std::placeholders::_1));
  autoware_orientation_sub_ =
    create_subscription<autoware_sensing_msgs::msg::GnssInsOrientationStamped>(
      "autoware_orientation", rclcpp::QoS{1},
      std::bind(&GNSSPoser::callbackGnssInsOrientationStamped, this, std::placeholders::_1));

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("gnss_pose", rclcpp::QoS{1});
  pose_cov_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "gnss_pose_cov", rclcpp::QoS{1});
  fixed_pub_ = create_publisher<tier4_debug_msgs::msg::BoolStamped>("gnss_fixed", rclcpp::QoS{1});
}

void GNSSPoser::callbackNavSatFix(
  const sensor_msgs::msg::NavSatFix::ConstSharedPtr nav_sat_fix_msg_ptr)
{
  // check fixed topic
  const bool is_fixed = isFixed(nav_sat_fix_msg_ptr->status);

  // publish is_fixed topic
  tier4_debug_msgs::msg::BoolStamped is_fixed_msg;
  is_fixed_msg.stamp = this->now();
  is_fixed_msg.data = is_fixed;
  fixed_pub_->publish(is_fixed_msg);

  if (!is_fixed) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "Not Fixed Topic. Skipping Calculate.");
    return;
  }

  // get position in coordinate_system
  const auto gnss_stat = convert(*nav_sat_fix_msg_ptr, coordinate_system_);
  const auto position = getPosition(gnss_stat);

  // calc median position
  position_buffer_.push_front(position);
  if (!position_buffer_.full()) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "Buffering Position. Output Skipped.");
    return;
  }
  const auto median_position = getMedianPosition(position_buffer_);

  // calc gnss antenna orientation
  geometry_msgs::msg::Quaternion orientation;
  if (use_gnss_ins_orientation_) {
    orientation = msg_gnss_ins_orientation_stamped_->orientation.orientation;
  } else {
    static auto prev_position = median_position;
    orientation = getQuaternionByPositionDifference(median_position, prev_position);
    prev_position = median_position;
  }

  // generate gnss_antenna_pose
  geometry_msgs::msg::Pose gnss_antenna_pose{};
  gnss_antenna_pose.position = median_position;
  gnss_antenna_pose.orientation = orientation;

  // get TF from gnss_antenna to map
  tf2::Transform tf_map2gnss_antenna{};
  tf2::fromMsg(gnss_antenna_pose, tf_map2gnss_antenna);

  // get TF from base_link to gnss_antenna
  auto tf_gnss_antenna2base_link_msg_ptr = std::make_shared<geometry_msgs::msg::TransformStamped>();
  getStaticTransform(
    gnss_frame_, base_frame_, tf_gnss_antenna2base_link_msg_ptr, nav_sat_fix_msg_ptr->header.stamp);
  tf2::Transform tf_gnss_antenna2base_link{};
  tf2::fromMsg(tf_gnss_antenna2base_link_msg_ptr->transform, tf_gnss_antenna2base_link);

  // transform pose from gnss_antenna(in map frame) to base_link(in map frame)
  tf2::Transform tf_map2base_link{};
  tf_map2base_link = tf_map2gnss_antenna * tf_gnss_antenna2base_link;

  geometry_msgs::msg::PoseStamped gnss_base_pose_msg{};
  gnss_base_pose_msg.header.stamp = nav_sat_fix_msg_ptr->header.stamp;
  gnss_base_pose_msg.header.frame_id = map_frame_;
  tf2::toMsg(tf_map2base_link, gnss_base_pose_msg.pose);

  // publish gnss_base_link pose in map frame
  pose_pub_->publish(gnss_base_pose_msg);

  // publish gnss_base_link pose_cov in map frame
  geometry_msgs::msg::PoseWithCovarianceStamped gnss_base_pose_cov_msg;
  gnss_base_pose_cov_msg.header = gnss_base_pose_msg.header;
  gnss_base_pose_cov_msg.pose.pose = gnss_base_pose_msg.pose;
  gnss_base_pose_cov_msg.pose.covariance[7 * 0] =
    canGetCovariance(*nav_sat_fix_msg_ptr) ? nav_sat_fix_msg_ptr->position_covariance[0] : 10.0;
  gnss_base_pose_cov_msg.pose.covariance[7 * 1] =
    canGetCovariance(*nav_sat_fix_msg_ptr) ? nav_sat_fix_msg_ptr->position_covariance[4] : 10.0;
  gnss_base_pose_cov_msg.pose.covariance[7 * 2] =
    canGetCovariance(*nav_sat_fix_msg_ptr) ? nav_sat_fix_msg_ptr->position_covariance[8] : 10.0;

  if (use_gnss_ins_orientation_) {
    gnss_base_pose_cov_msg.pose.covariance[7 * 3] =
      std::pow(msg_gnss_ins_orientation_stamped_->orientation.rmse_rotation_x, 2);
    gnss_base_pose_cov_msg.pose.covariance[7 * 4] =
      std::pow(msg_gnss_ins_orientation_stamped_->orientation.rmse_rotation_y, 2);
    gnss_base_pose_cov_msg.pose.covariance[7 * 5] =
      std::pow(msg_gnss_ins_orientation_stamped_->orientation.rmse_rotation_z, 2);
  } else {
    gnss_base_pose_cov_msg.pose.covariance[7 * 3] = 0.1;
    gnss_base_pose_cov_msg.pose.covariance[7 * 4] = 0.1;
    gnss_base_pose_cov_msg.pose.covariance[7 * 5] = 1.0;
  }

  pose_cov_pub_->publish(gnss_base_pose_cov_msg);

  // broadcast map to gnss_base_link
  publishTF(map_frame_, gnss_base_frame_, gnss_base_pose_msg);
}

void GNSSPoser::callbackGnssInsOrientationStamped(
  const autoware_sensing_msgs::msg::GnssInsOrientationStamped::ConstSharedPtr msg)
{
  *msg_gnss_ins_orientation_stamped_ = *msg;
}

bool GNSSPoser::isFixed(const sensor_msgs::msg::NavSatStatus & nav_sat_status_msg)
{
  return nav_sat_status_msg.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX;
}

bool GNSSPoser::canGetCovariance(const sensor_msgs::msg::NavSatFix & nav_sat_fix_msg)
{
  return nav_sat_fix_msg.position_covariance_type >
         sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
}

GNSSStat GNSSPoser::convert(
  const sensor_msgs::msg::NavSatFix & nav_sat_fix_msg, CoordinateSystem coordinate_system)
{
  GNSSStat gnss_stat;
  if (coordinate_system == CoordinateSystem::UTM) {
    gnss_stat = NavSatFix2UTM(nav_sat_fix_msg, this->get_logger());
  } else if (coordinate_system == CoordinateSystem::LOCAL_CARTESIAN_UTM) {
    gnss_stat =
      NavSatFix2LocalCartesianUTM(nav_sat_fix_msg, nav_sat_fix_origin_, this->get_logger());
  } else if (coordinate_system == CoordinateSystem::MGRS) {
    gnss_stat = NavSatFix2MGRS(nav_sat_fix_msg, MGRSPrecision::_100MICRO_METER, this->get_logger());
  } else if (coordinate_system == CoordinateSystem::PLANE) {
    gnss_stat = NavSatFix2PLANE(nav_sat_fix_msg, plane_zone_, this->get_logger());
  } else if (coordinate_system == CoordinateSystem::LOCAL_CARTESIAN_WGS84) {
    gnss_stat =
      NavSatFix2LocalCartesianWGS84(nav_sat_fix_msg, nav_sat_fix_origin_, this->get_logger());
  } else {
    RCLCPP_ERROR_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "Unknown Coordinate System");
  }
  return gnss_stat;
}

geometry_msgs::msg::Point GNSSPoser::getPosition(const GNSSStat & gnss_stat)
{
  geometry_msgs::msg::Point point;
  point.x = gnss_stat.x;
  point.y = gnss_stat.y;
  point.z = gnss_stat.z;
  return point;
}

geometry_msgs::msg::Point GNSSPoser::getMedianPosition(
  const boost::circular_buffer<geometry_msgs::msg::Point> & position_buffer)
{
  auto getMedian = [](std::vector<double> array) {
    std::sort(std::begin(array), std::end(array));
    const size_t median_index = array.size() / 2;
    double median = (array.size() % 2)
                      ? (array.at(median_index))
                      : ((array.at(median_index) + array.at(median_index - 1)) / 2);
    return median;
  };

  std::vector<double> array_x;
  std::vector<double> array_y;
  std::vector<double> array_z;
  for (const auto & position : position_buffer) {
    array_x.push_back(position.x);
    array_y.push_back(position.y);
    array_z.push_back(position.z);
  }

  geometry_msgs::msg::Point median_point;
  median_point.x = getMedian(array_x);
  median_point.y = getMedian(array_y);
  median_point.z = getMedian(array_z);
  return median_point;
}

geometry_msgs::msg::Quaternion GNSSPoser::getQuaternionByHeading(const int heading)
{
  int heading_conv = 0;
  // convert heading[0(North)~360] to yaw[-M_PI(West)~M_PI]
  if (heading >= 0 && heading <= 27000000) {
    heading_conv = 9000000 - heading;
  } else {
    heading_conv = 45000000 - heading;
  }
  const double yaw = (heading_conv * 1e-5) * M_PI / 180.0;

  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, yaw);

  return tf2::toMsg(quaternion);
}

geometry_msgs::msg::Quaternion GNSSPoser::getQuaternionByPositionDifference(
  const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Point & prev_point)
{
  const double yaw = std::atan2(point.y - prev_point.y, point.x - prev_point.x);
  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, yaw);
  return tf2::toMsg(quaternion);
}

bool GNSSPoser::getTransform(
  const std::string & target_frame, const std::string & source_frame,
  const geometry_msgs::msg::TransformStamped::SharedPtr transform_stamped_ptr)
{
  if (target_frame == source_frame) {
    transform_stamped_ptr->header.stamp = this->now();
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return true;
  }

  try {
    *transform_stamped_ptr =
      tf2_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(), ex.what());
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "Please publish TF " << target_frame.c_str() << " to " << source_frame.c_str());

    transform_stamped_ptr->header.stamp = this->now();
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return false;
  }
  return true;
}

bool GNSSPoser::getStaticTransform(
  const std::string & target_frame, const std::string & source_frame,
  const geometry_msgs::msg::TransformStamped::SharedPtr transform_stamped_ptr,
  const builtin_interfaces::msg::Time & stamp)
{
  if (target_frame == source_frame) {
    transform_stamped_ptr->header.stamp = stamp;
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return true;
  }

  try {
    *transform_stamped_ptr = tf2_buffer_.lookupTransform(
      target_frame, source_frame,
      tf2::TimePoint(std::chrono::seconds(stamp.sec) + std::chrono::nanoseconds(stamp.nanosec)));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(), ex.what());
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "Please publish TF " << target_frame.c_str() << " to " << source_frame.c_str());

    transform_stamped_ptr->header.stamp = stamp;
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return false;
  }
  return true;
}

void GNSSPoser::publishTF(
  const std::string & frame_id, const std::string & child_frame_id,
  const geometry_msgs::msg::PoseStamped & pose_msg)
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.header.stamp = pose_msg.header.stamp;

  transform_stamped.transform.translation.x = pose_msg.pose.position.x;
  transform_stamped.transform.translation.y = pose_msg.pose.position.y;
  transform_stamped.transform.translation.z = pose_msg.pose.position.z;

  tf2::Quaternion tf_quaternion;
  tf2::fromMsg(pose_msg.pose.orientation, tf_quaternion);
  transform_stamped.transform.rotation.x = tf_quaternion.x();
  transform_stamped.transform.rotation.y = tf_quaternion.y();
  transform_stamped.transform.rotation.z = tf_quaternion.z();
  transform_stamped.transform.rotation.w = tf_quaternion.w();

  tf2_broadcaster_.sendTransform(transform_stamped);
}
}  // namespace gnss_poser

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(gnss_poser::GNSSPoser)
