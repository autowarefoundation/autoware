// Copyright 2023 Autoware Foundation
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

#include "lidar_marker_localizer.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/transform/transforms.hpp>
#include <autoware_point_types/types.hpp>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/qos.hpp>

#include <geometry_msgs/msg/vector3.hpp>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <utility>

namespace autoware::lidar_marker_localizer
{

landmark_manager::Landmark get_nearest_landmark(
  const geometry_msgs::msg::Pose & self_pose,
  const std::vector<landmark_manager::Landmark> & landmarks);
std::array<double, 36> rotate_covariance(
  const std::array<double, 36> & src_covariance, const Eigen::Matrix3d & rotation);

LidarMarkerLocalizer::LidarMarkerLocalizer(const rclcpp::NodeOptions & node_options)
: Node("lidar_marker_localizer", node_options), is_activated_(false)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  param_.marker_name = this->declare_parameter<std::string>("marker_name");
  param_.resolution = this->declare_parameter<double>("resolution");
  param_.intensity_pattern = this->declare_parameter<std::vector<int64_t>>("intensity_pattern");
  param_.match_intensity_difference_threshold =
    this->declare_parameter<int64_t>("match_intensity_difference_threshold");
  param_.positive_match_num_threshold =
    this->declare_parameter<int64_t>("positive_match_num_threshold");
  param_.negative_match_num_threshold =
    this->declare_parameter<int64_t>("negative_match_num_threshold");
  param_.vote_threshold_for_detect_marker =
    this->declare_parameter<int64_t>("vote_threshold_for_detect_marker");
  param_.marker_height_from_ground = this->declare_parameter<double>("marker_height_from_ground");
  param_.self_pose_timeout_sec = this->declare_parameter<double>("self_pose_timeout_sec");
  param_.self_pose_distance_tolerance_m =
    this->declare_parameter<double>("self_pose_distance_tolerance_m");
  param_.limit_distance_from_self_pose_to_nearest_marker =
    this->declare_parameter<double>("limit_distance_from_self_pose_to_nearest_marker");
  param_.limit_distance_from_self_pose_to_marker =
    this->declare_parameter<double>("limit_distance_from_self_pose_to_marker");
  std::vector<double> base_covariance =
    this->declare_parameter<std::vector<double>>("base_covariance");
  for (std::size_t i = 0; i < base_covariance.size(); ++i) {
    param_.base_covariance[i] = base_covariance[i];
  }
  param_.marker_width = this->declare_parameter<double>("marker_width");
  param_.enable_save_log = this->declare_parameter<bool>("enable_save_log");
  param_.save_file_directory_path =
    this->declare_parameter<std::string>("save_file_directory_path");
  param_.save_file_name = this->declare_parameter<std::string>("save_file_name");
  param_.save_frame_id = this->declare_parameter<std::string>("save_frame_id");

  ekf_pose_buffer_ = std::make_unique<SmartPoseBuffer>(
    this->get_logger(), param_.self_pose_timeout_sec, param_.self_pose_distance_tolerance_m);

  rclcpp::CallbackGroup::SharedPtr points_callback_group;
  points_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto points_sub_opt = rclcpp::SubscriptionOptions();
  points_sub_opt.callback_group = points_callback_group;
  sub_points_ = this->create_subscription<PointCloud2>(
    "~/input/pointcloud", rclcpp::QoS(1).best_effort(),
    std::bind(&LidarMarkerLocalizer::points_callback, this, _1), points_sub_opt);

  rclcpp::CallbackGroup::SharedPtr self_pose_callback_group;
  self_pose_callback_group =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto self_pose_sub_opt = rclcpp::SubscriptionOptions();
  self_pose_sub_opt.callback_group = self_pose_callback_group;
  sub_self_pose_ = this->create_subscription<PoseWithCovarianceStamped>(
    "~/input/ekf_pose", rclcpp::QoS(1),
    std::bind(&LidarMarkerLocalizer::self_pose_callback, this, _1), self_pose_sub_opt);
  sub_map_bin_ = this->create_subscription<HADMapBin>(
    "~/input/lanelet2_map", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal),
    std::bind(&LidarMarkerLocalizer::map_bin_callback, this, _1));

  pub_base_link_pose_with_covariance_on_map_ =
    this->create_publisher<PoseWithCovarianceStamped>("~/output/pose_with_covariance", 10);
  rclcpp::QoS qos_marker = rclcpp::QoS(rclcpp::KeepLast(10));
  qos_marker.transient_local();
  qos_marker.reliable();
  pub_marker_mapped_ = this->create_publisher<MarkerArray>("~/debug/marker_mapped", qos_marker);
  pub_marker_detected_ = this->create_publisher<PoseArray>("~/debug/marker_detected", 10);
  pub_debug_pose_with_covariance_ =
    this->create_publisher<PoseWithCovarianceStamped>("~/debug/pose_with_covariance", 10);
  pub_marker_pointcloud_ = this->create_publisher<PointCloud2>("~/debug/marker_pointcloud", 10);
  service_trigger_node_ = this->create_service<SetBool>(
    "~/service/trigger_node_srv",
    std::bind(&LidarMarkerLocalizer::service_trigger_node, this, _1, _2),
    rclcpp::ServicesQoS().get_rmw_qos_profile(), points_callback_group);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

  diagnostics_module_.reset(new DiagnosticsModule(this, "marker_detection_status"));
}

void LidarMarkerLocalizer::initialize_diagnostics()
{
  diagnostics_module_->clear();
  diagnostics_module_->add_key_value("is_received_map", false);
  diagnostics_module_->add_key_value("is_received_self_pose", false);
  diagnostics_module_->add_key_value("detect_marker_num", 0);
  diagnostics_module_->add_key_value("distance_self_pose_to_nearest_marker", 0.0);
  diagnostics_module_->add_key_value(
    "limit_distance_from_self_pose_to_nearest_marker",
    param_.limit_distance_from_self_pose_to_nearest_marker);
  diagnostics_module_->add_key_value("distance_lanelet2_marker_to_detected_marker", 0.0);
  diagnostics_module_->add_key_value(
    "limit_distance_from_lanelet2_marker_to_detected_marker",
    param_.limit_distance_from_self_pose_to_marker);
}

void LidarMarkerLocalizer::map_bin_callback(const HADMapBin::ConstSharedPtr & map_bin_msg_ptr)
{
  landmark_manager_.parse_landmarks(map_bin_msg_ptr, param_.marker_name);
  const MarkerArray marker_msg = landmark_manager_.get_landmarks_as_marker_array_msg();
  pub_marker_mapped_->publish(marker_msg);
}

void LidarMarkerLocalizer::self_pose_callback(
  const PoseWithCovarianceStamped::ConstSharedPtr & self_pose_msg_ptr)
{
  // TODO(YamatoAndo)
  // if (!is_activated_) return;

  if (self_pose_msg_ptr->header.frame_id == "map") {
    ekf_pose_buffer_->push_back(self_pose_msg_ptr);
  } else {
    RCLCPP_ERROR_STREAM_THROTTLE(
      get_logger(), *this->get_clock(), 1000,
      "Received initial pose message with frame_id "
        << self_pose_msg_ptr->header.frame_id << ", but expected map. "
        << "Please check the frame_id in the input topic and ensure it is correct.");
  }
}

void LidarMarkerLocalizer::points_callback(const PointCloud2::ConstSharedPtr & points_msg_ptr)
{
  const auto sensor_ros_time = points_msg_ptr->header.stamp;

  initialize_diagnostics();

  main_process(points_msg_ptr);

  diagnostics_module_->publish(sensor_ros_time);
}

void LidarMarkerLocalizer::main_process(const PointCloud2::ConstSharedPtr & points_msg_ptr)
{
  const builtin_interfaces::msg::Time sensor_ros_time = points_msg_ptr->header.stamp;

  // (1) check if the map have be received
  const std::vector<landmark_manager::Landmark> map_landmarks = landmark_manager_.get_landmarks();
  const bool is_received_map = !map_landmarks.empty();
  diagnostics_module_->add_key_value("is_received_map", is_received_map);
  if (!is_received_map) {
    std::stringstream message;
    message << "Not receive the landmark information. Please check if the vector map is being "
            << "published and if the landmark information is correctly specified.";
    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
    diagnostics_module_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    return;
  }

  // (2) get Self Pose
  const std::optional<SmartPoseBuffer::InterpolateResult> interpolate_result =
    ekf_pose_buffer_->interpolate(sensor_ros_time);

  const bool is_received_self_pose = interpolate_result != std::nullopt;
  diagnostics_module_->add_key_value("is_received_self_pose", is_received_self_pose);
  if (!is_received_self_pose) {
    std::stringstream message;
    message << "Could not get self_pose. Please check if the self pose is being published and if "
            << "the timestamp of the self pose is correctly specified";
    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
    diagnostics_module_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    return;
  }

  ekf_pose_buffer_->pop_old(sensor_ros_time);
  const Pose self_pose = interpolate_result.value().interpolated_pose.pose.pose;

  // (3) detect marker
  const std::vector<landmark_manager::Landmark> detected_landmarks =
    detect_landmarks(points_msg_ptr);

  const bool is_detected_marker = !detected_landmarks.empty();
  diagnostics_module_->add_key_value("detect_marker_num", detected_landmarks.size());

  // (4) check distance to the nearest marker
  const landmark_manager::Landmark nearest_marker = get_nearest_landmark(self_pose, map_landmarks);
  const Pose nearest_marker_pose_on_base_link =
    autoware::universe_utils::inverseTransformPose(nearest_marker.pose, self_pose);

  const double distance_from_self_pose_to_nearest_marker =
    std::abs(nearest_marker_pose_on_base_link.position.x);
  diagnostics_module_->add_key_value(
    "distance_self_pose_to_nearest_marker", distance_from_self_pose_to_nearest_marker);

  const bool is_exist_marker_within_self_pose =
    distance_from_self_pose_to_nearest_marker <
    param_.limit_distance_from_self_pose_to_nearest_marker;

  if (!is_detected_marker) {
    if (!is_exist_marker_within_self_pose) {
      std::stringstream message;
      message << "Could not detect marker, because the distance from self_pose to nearest_marker "
              << "is too far (" << distance_from_self_pose_to_nearest_marker << " [m]).";
      diagnostics_module_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::OK, message.str());
    } else {
      std::stringstream message;
      message << "Could not detect marker, although the distance from self_pose to nearest_marker "
              << "is near (" << distance_from_self_pose_to_nearest_marker << " [m]).";
      RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
      diagnostics_module_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    }
    return;
  }

  // for debug
  if (pub_marker_detected_->get_subscription_count() > 0) {
    PoseArray pose_array_msg;
    pose_array_msg.header.stamp = sensor_ros_time;
    pose_array_msg.header.frame_id = "map";
    for (const landmark_manager::Landmark & landmark : detected_landmarks) {
      const Pose detected_marker_on_map =
        autoware::universe_utils::transformPose(landmark.pose, self_pose);
      pose_array_msg.poses.push_back(detected_marker_on_map);
    }
    pub_marker_detected_->publish(pose_array_msg);
  }

  // (4) calculate diff pose
  const Pose new_self_pose =
    landmark_manager_.calculate_new_self_pose(detected_landmarks, self_pose, false);

  const double diff_x = new_self_pose.position.x - self_pose.position.x;
  const double diff_y = new_self_pose.position.y - self_pose.position.y;

  const double diff_norm = std::hypot(diff_x, diff_y);
  const bool is_exist_marker_within_lanelet2_map =
    diff_norm < param_.limit_distance_from_self_pose_to_marker;

  diagnostics_module_->add_key_value("distance_lanelet2_marker_to_detected_marker", diff_norm);
  if (!is_exist_marker_within_lanelet2_map) {
    std::stringstream message;
    message << "The distance from lanelet2 to the detect marker is too far(" << diff_norm
            << " [m]). The limit is " << param_.limit_distance_from_self_pose_to_marker << ".";
    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
    diagnostics_module_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    return;
  }

  // (5) Apply diff pose to self pose
  // only x and y is changed
  PoseWithCovarianceStamped result;
  result.header.stamp = sensor_ros_time;
  result.header.frame_id = "map";
  result.pose.pose.position.x = new_self_pose.position.x;
  result.pose.pose.position.y = new_self_pose.position.y;
  result.pose.pose.position.z = self_pose.position.z;
  result.pose.pose.orientation = self_pose.orientation;

  // set covariance
  const Eigen::Quaterniond map_to_base_link_quat = Eigen::Quaterniond(
    result.pose.pose.orientation.w, result.pose.pose.orientation.x, result.pose.pose.orientation.y,
    result.pose.pose.orientation.z);
  const Eigen::Matrix3d map_to_base_link_rotation =
    map_to_base_link_quat.normalized().toRotationMatrix();
  result.pose.covariance = rotate_covariance(param_.base_covariance, map_to_base_link_rotation);

  pub_base_link_pose_with_covariance_on_map_->publish(result);
  pub_debug_pose_with_covariance_->publish(result);

  // for debug
  const landmark_manager::Landmark nearest_detected_landmark =
    get_nearest_landmark(self_pose, detected_landmarks);
  const auto marker_pointcloud_msg_ptr =
    extract_marker_pointcloud(points_msg_ptr, nearest_detected_landmark.pose);
  pub_marker_pointcloud_->publish(*marker_pointcloud_msg_ptr);

  // save log
  if (param_.enable_save_log) {
    save_detected_marker_log(marker_pointcloud_msg_ptr);
  }
}

void LidarMarkerLocalizer::service_trigger_node(
  const SetBool::Request::SharedPtr req, SetBool::Response::SharedPtr res)
{
  is_activated_ = req->data;
  if (is_activated_) {
    ekf_pose_buffer_->clear();
  }
  res->success = true;
}

std::vector<landmark_manager::Landmark> LidarMarkerLocalizer::detect_landmarks(
  const PointCloud2::ConstSharedPtr & points_msg_ptr)
{
  // TODO(YamatoAndo)
  // Transform sensor_frame to base_link

  pcl::PointCloud<autoware_point_types::PointXYZIRC>::Ptr points_ptr(
    new pcl::PointCloud<autoware_point_types::PointXYZIRC>);
  pcl::fromROSMsg(*points_msg_ptr, *points_ptr);

  if (points_ptr->empty()) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No points!");
    return std::vector<landmark_manager::Landmark>{};
  }

  std::vector<pcl::PointCloud<autoware_point_types::PointXYZIRC>> ring_points(128);

  float min_x = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  for (const autoware_point_types::PointXYZIRC & point : points_ptr->points) {
    ring_points[point.channel].push_back(point);
    min_x = std::min(min_x, point.x);
    max_x = std::max(max_x, point.x);
  }

  // Check that the leaf size is not too small, given the size of the data
  const int bin_num = static_cast<int>((max_x - min_x) / param_.resolution + 1);

  // initialize variables
  std::vector<int> vote(bin_num, 0);
  std::vector<float> min_y(bin_num, std::numeric_limits<float>::max());

  // for each channel
  for (const pcl::PointCloud<autoware_point_types::PointXYZIRC> & one_ring : ring_points) {
    std::vector<double> intensity_sum(bin_num, 0.0);
    std::vector<int> intensity_num(bin_num, 0);
    std::vector<double> average_intensity(bin_num, 0.0);

    for (const autoware_point_types::PointXYZIRC & point : one_ring.points) {
      const int bin_index = static_cast<int>((point.x - min_x) / param_.resolution);
      intensity_sum[bin_index] += point.intensity;
      intensity_num[bin_index]++;
      min_y[bin_index] = std::min(min_y[bin_index], point.y);
    }

    // calc average
    for (int bin_index = 0; bin_index < bin_num; bin_index++) {
      if (intensity_num[bin_index] == 0) {
        continue;
      }

      average_intensity[bin_index] = intensity_sum[bin_index] / intensity_num[bin_index];
    }

    // pattern matching
    for (size_t i = 0; i <= bin_num - param_.intensity_pattern.size(); i++) {
      int64_t pos = 0;
      int64_t neg = 0;
      double min_intensity = std::numeric_limits<double>::max();
      double max_intensity = std::numeric_limits<double>::lowest();

      // find max_min
      for (size_t j = 0; j < param_.intensity_pattern.size(); j++) {
        if (intensity_num[i + j] == 0) {
          continue;
        }

        min_intensity = std::min(min_intensity, average_intensity[i + j]);
        max_intensity = std::max(max_intensity, average_intensity[i + j]);
      }

      if (max_intensity <= min_intensity) {
        continue;
      }

      const double center_intensity = (max_intensity - min_intensity) / 2.0 + min_intensity;

      for (size_t j = 0; j < param_.intensity_pattern.size(); j++) {
        if (intensity_num[i + j] == 0) {
          continue;
        }

        if (param_.intensity_pattern[j] == 1) {
          // check positive
          if (
            average_intensity[i + j] >
            center_intensity + static_cast<double>(param_.match_intensity_difference_threshold)) {
            pos++;
          }
        } else if (param_.intensity_pattern[j] == -1) {
          // check negative
          if (
            average_intensity[i + j] <
            center_intensity - static_cast<double>(param_.match_intensity_difference_threshold)) {
            neg++;
          }
        } else {
          // ignore param_.intensity_pattern[j] == 0
        }
      }

      if (
        pos >= param_.positive_match_num_threshold && neg >= param_.negative_match_num_threshold) {
        vote[i]++;
      }
    }
  }

  std::vector<landmark_manager::Landmark> detected_landmarks;

  for (size_t i = 0; i < bin_num - param_.intensity_pattern.size(); i++) {
    if (vote[i] > param_.vote_threshold_for_detect_marker) {
      const double bin_position =
        static_cast<double>(i) + static_cast<double>(param_.intensity_pattern.size()) / 2.0;
      Pose marker_pose_on_base_link;
      marker_pose_on_base_link.position.x = bin_position * param_.resolution + min_x;
      marker_pose_on_base_link.position.y = min_y[i];
      marker_pose_on_base_link.position.z = param_.marker_height_from_ground;
      marker_pose_on_base_link.orientation =
        autoware::universe_utils::createQuaternionFromRPY(M_PI_2, 0.0, 0.0);  // TODO(YamatoAndo)
      detected_landmarks.push_back(landmark_manager::Landmark{"0", marker_pose_on_base_link});
    }
  }

  return detected_landmarks;
}

landmark_manager::Landmark get_nearest_landmark(
  const geometry_msgs::msg::Pose & self_pose,
  const std::vector<landmark_manager::Landmark> & landmarks)
{
  landmark_manager::Landmark nearest_landmark;
  double min_distance = std::numeric_limits<double>::max();

  for (const auto & landmark : landmarks) {
    const double curr_distance =
      autoware::universe_utils::calcDistance3d(landmark.pose.position, self_pose.position);

    if (curr_distance > min_distance) {
      continue;
    }

    min_distance = curr_distance;
    nearest_landmark = landmark;
  }
  return nearest_landmark;
}

std::array<double, 36> rotate_covariance(
  const std::array<double, 36> & src_covariance, const Eigen::Matrix3d & rotation)
{
  std::array<double, 36> ret_covariance = src_covariance;

  Eigen::Matrix3d src_cov;
  src_cov << src_covariance[0], src_covariance[1], src_covariance[2], src_covariance[6],
    src_covariance[7], src_covariance[8], src_covariance[12], src_covariance[13],
    src_covariance[14];

  Eigen::Matrix3d ret_cov;
  ret_cov = rotation * src_cov * rotation.transpose();

  for (Eigen::Index i = 0; i < 3; ++i) {
    ret_covariance[i] = ret_cov(0, i);
    ret_covariance[i + 6] = ret_cov(1, i);
    ret_covariance[i + 12] = ret_cov(2, i);
  }

  return ret_covariance;
}

sensor_msgs::msg::PointCloud2::SharedPtr LidarMarkerLocalizer::extract_marker_pointcloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & points_msg_ptr,
  const geometry_msgs::msg::Pose marker_pose) const
{
  // convert from ROSMsg to PCL
  pcl::shared_ptr<pcl::PointCloud<autoware_point_types::PointXYZIRC>> points_ptr(
    new pcl::PointCloud<autoware_point_types::PointXYZIRC>);
  pcl::fromROSMsg(*points_msg_ptr, *points_ptr);

  pcl::shared_ptr<pcl::PointCloud<autoware_point_types::PointXYZIRC>> marker_points_ptr(
    new pcl::PointCloud<autoware_point_types::PointXYZIRC>);

  // extract marker pointcloud
  for (const autoware_point_types::PointXYZIRC & point : points_ptr->points) {
    const double xy_distance = std::sqrt(
      std::pow(point.x - marker_pose.position.x, 2.0) +
      std::pow(point.y - marker_pose.position.y, 2.0));
    if (xy_distance < param_.marker_width / 2.0) {
      marker_points_ptr->push_back(point);
    }
  }

  marker_points_ptr->width = marker_points_ptr->size();
  marker_points_ptr->height = 1;
  marker_points_ptr->is_dense = false;

  sensor_msgs::msg::PointCloud2::SharedPtr marker_points_msg_ptr(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*marker_points_ptr, *marker_points_msg_ptr);
  marker_points_msg_ptr->header = points_msg_ptr->header;

  return marker_points_msg_ptr;
}

void LidarMarkerLocalizer::save_detected_marker_log(
  const sensor_msgs::msg::PointCloud2::SharedPtr & points_msg_ptr)
{
  // transform input_frame to save_frame_id
  sensor_msgs::msg::PointCloud2::SharedPtr marker_points_msg_sensor_frame_ptr(
    new sensor_msgs::msg::PointCloud2);
  transform_sensor_measurement(
    param_.save_frame_id, points_msg_ptr->header.frame_id, points_msg_ptr,
    marker_points_msg_sensor_frame_ptr);

  // convert from ROSMsg to PCL
  pcl::shared_ptr<pcl::PointCloud<autoware_point_types::PointXYZIRC>>
    marker_points_sensor_frame_ptr(new pcl::PointCloud<autoware_point_types::PointXYZIRC>);
  pcl::fromROSMsg(*marker_points_msg_sensor_frame_ptr, *marker_points_sensor_frame_ptr);

  // to csv format
  std::stringstream log_message;
  log_message << "point.position.x,point.position.y,point.position.z,point.intensity,point.ring"
              << std::endl;
  for (const auto & point : marker_points_sensor_frame_ptr->points) {
    log_message << point.x;
    log_message << "," << point.y;
    log_message << "," << point.z;
    log_message << "," << point.intensity;
    log_message << "," << point.channel;
    log_message << std::endl;
  }

  // create file name
  const double times_seconds = rclcpp::Time(points_msg_ptr->header.stamp).seconds();
  double time_integer_tmp = 0.0;
  double time_decimal = std::modf(times_seconds, &time_integer_tmp);
  auto time_integer = static_cast<int64_t>(time_integer_tmp);
  struct tm * time_info{};
  time_info = std::localtime(&time_integer);
  std::stringstream file_name;
  file_name << param_.save_file_name << std::put_time(time_info, "%Y%m%d-%H%M%S") << "-"
            << std::setw(3) << std::setfill('0') << static_cast<int>((time_decimal) * 1000)
            << ".csv";

  // write log_message to file
  std::filesystem::path save_file_directory_path = param_.save_file_directory_path;
  std::filesystem::create_directories(save_file_directory_path);
  std::ofstream csv_file(save_file_directory_path.append(file_name.str()));
  csv_file << log_message.str();
  csv_file.close();
}

void LidarMarkerLocalizer::transform_sensor_measurement(
  const std::string & source_frame, const std::string & target_frame,
  const sensor_msgs::msg::PointCloud2::SharedPtr & sensor_points_input_ptr,
  sensor_msgs::msg::PointCloud2::SharedPtr & sensor_points_output_ptr)
{
  if (source_frame == target_frame) {
    sensor_points_output_ptr = sensor_points_input_ptr;
    return;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    RCLCPP_WARN(
      this->get_logger(), "Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());
    // Since there is no clear error handling policy, temporarily return as is.
    sensor_points_output_ptr = sensor_points_input_ptr;
    return;
  }

  const geometry_msgs::msg::PoseStamped target_to_source_pose_stamped =
    autoware::universe_utils::transform2pose(transform);
  const Eigen::Matrix4f base_to_sensor_matrix =
    pose_to_matrix4f(target_to_source_pose_stamped.pose);
  pcl_ros::transformPointCloud(
    base_to_sensor_matrix, *sensor_points_input_ptr, *sensor_points_output_ptr);
}

}  // namespace autoware::lidar_marker_localizer

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::lidar_marker_localizer::LidarMarkerLocalizer)
