// Copyright 2018-2019 Autoware Foundation
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

#include "map_based_prediction_ros.hpp"

#include "map_based_prediction.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <unique_identifier_msgs/msg/uuid.hpp>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

std::string toHexString(const unique_identifier_msgs::msg::UUID & id)
{
  std::stringstream ss;
  for (auto i = 0; i < 16; ++i) {
    ss << std::hex << std::setfill('0') << std::setw(2) << +id.uuid[i];
  }
  return ss.str();
}

MapBasedPredictionROS::MapBasedPredictionROS(const rclcpp::NodeOptions & node_options)
: Node("map_based_prediction", node_options),
  interpolating_resolution_(0.5),
  debug_accumulated_time_(0.0)
{
  [[maybe_unused]] auto ret =
    rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  tf_buffer_ptr_ = std::make_shared<tf2_ros::Buffer>(clock);
  tf_listener_ptr_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_ptr_);
  prediction_time_horizon_ = declare_parameter("prediction_time_horizon", 10.0);
  prediction_sampling_delta_time_ = declare_parameter("prediction_sampling_delta_time", 0.5);
  min_velocity_for_map_based_prediction_ =
    declare_parameter("min_velocity_for_map_based_prediction", 1.0);
  dist_threshold_for_searching_lanelet_ =
    declare_parameter("dist_threshold_for_searching_lanelet", 3.0);
  delta_yaw_threshold_for_searching_lanelet_ =
    declare_parameter("delta_yaw_threshold_for_searching_lanelet", 0.785);
  sigma_lateral_offset_ = declare_parameter("sigma_lateral_offset", 0.5);
  sigma_yaw_angle_ = declare_parameter("sigma_yaw_angle", 5.0);
  history_time_length_ = declare_parameter("history_time_length", 1.0);
  dist_ratio_threshold_to_left_bound_ =
    declare_parameter("dist_ratio_threshold_to_left_bound", -0.5);
  dist_ratio_threshold_to_right_bound_ =
    declare_parameter("dist_ratio_threshold_to_right_bound", 0.5);
  diff_dist_threshold_to_left_bound_ = declare_parameter("diff_dist_threshold_to_left_bound", 0.29);
  diff_dist_threshold_to_right_bound_ =
    declare_parameter("diff_dist_threshold_to_right_bound", -0.29);

  map_based_prediction_ = std::make_shared<MapBasedPrediction>(
    interpolating_resolution_, prediction_time_horizon_, prediction_sampling_delta_time_);

  sub_objects_ = this->create_subscription<autoware_auto_perception_msgs::msg::TrackedObjects>(
    "/perception/object_recognition/tracking/objects", 1,
    std::bind(&MapBasedPredictionROS::objectsCallback, this, std::placeholders::_1));
  sub_map_ = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&MapBasedPredictionROS::mapCallback, this, std::placeholders::_1));

  pub_objects_ = this->create_publisher<autoware_auto_perception_msgs::msg::PredictedObjects>(
    "objects", rclcpp::QoS{1});
  pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "objects_path_markers", rclcpp::QoS{1});
}

double MapBasedPredictionROS::getObjectYaw(
  const autoware_auto_perception_msgs::msg::TrackedObject & object)
{
  if (object.kinematics.orientation_availability) {
    return tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
  }

  geometry_msgs::msg::Pose object_frame_pose;
  geometry_msgs::msg::Pose map_frame_pose;
  object_frame_pose.position.x = object.kinematics.twist_with_covariance.twist.linear.x * 0.1;
  object_frame_pose.position.y = object.kinematics.twist_with_covariance.twist.linear.y * 0.1;
  tf2::Transform tf_object2future;
  tf2::Transform tf_map2object;
  tf2::Transform tf_map2future;

  tf2::fromMsg(object.kinematics.pose_with_covariance.pose, tf_map2object);
  tf2::fromMsg(object_frame_pose, tf_object2future);
  tf_map2future = tf_map2object * tf_object2future;
  tf2::toMsg(tf_map2future, map_frame_pose);
  double dx = map_frame_pose.position.x - object.kinematics.pose_with_covariance.pose.position.x;
  double dy = map_frame_pose.position.y - object.kinematics.pose_with_covariance.pose.position.y;
  return std::atan2(dy, dx);
}

double MapBasedPredictionROS::calculateLikelihood(
  const std::vector<geometry_msgs::msg::Pose> & path,
  const autoware_auto_perception_msgs::msg::TrackedObject & object)
{
  // We compute the confidence value based on the object current position and angle
  // Calculate path length
  const double path_len = tier4_autoware_utils::calcArcLength(path);
  const size_t nearest_segment_idx = tier4_autoware_utils::findNearestSegmentIndex(
    path, object.kinematics.pose_with_covariance.pose.position);
  const double l = tier4_autoware_utils::calcLongitudinalOffsetToSegment(
    path, nearest_segment_idx, object.kinematics.pose_with_covariance.pose.position);
  const double current_s_position =
    tier4_autoware_utils::calcSignedArcLength(path, 0, nearest_segment_idx) + l;
  // If the obstacle is ahead of this path, we assume the confidence for this path is 0
  if (current_s_position > path_len) {
    return 0.0;
  }

  // Euclid Lateral Distance
  const double abs_d = std::fabs(tier4_autoware_utils::calcLateralOffset(
    path, object.kinematics.pose_with_covariance.pose.position));

  // Yaw Difference between obstacle and lane angle
  const double lane_yaw = tf2::getYaw(path.at(nearest_segment_idx).orientation);
  const double object_yaw = getObjectYaw(object);
  const double delta_yaw = object_yaw - lane_yaw;
  const double abs_norm_delta_yaw = std::fabs(tier4_autoware_utils::normalizeRadian(delta_yaw));

  // Compute Chi-squared distributed (Equation (8) in the paper)
  const double sigma_d = sigma_lateral_offset_;  // Standard Deviation for lateral position
  const double sigma_yaw = M_PI * sigma_yaw_angle_ / 180.0;  // Standard Deviation for yaw angle
  Eigen::Vector2d delta;
  delta << abs_d, abs_norm_delta_yaw;
  Eigen::Matrix2d P_inv;
  P_inv << 1.0 / (sigma_d * sigma_d), 0.0, 0.0, 1.0 / (sigma_yaw * sigma_yaw);
  const double MINIMUM_DISTANCE = 1e-6;
  const double dist = std::max(delta.dot(P_inv * delta), MINIMUM_DISTANCE);
  return 1.0 / dist;
}

bool MapBasedPredictionROS::checkCloseLaneletCondition(
  const std::pair<double, lanelet::Lanelet> & lanelet,
  const autoware_auto_perception_msgs::msg::TrackedObject & object,
  const lanelet::BasicPoint2d & search_point)
{
  // Step1. If we only have one point in the centerline, we will ignore the lanelet
  if (lanelet.second.centerline().size() <= 1) {
    return false;
  }

  // Step2. Check if the obstacle is inside of this lanelet
  if (!lanelet::geometry::inside(lanelet.second, search_point)) {
    return false;
  }

  // If the object is in the object buffer, we check if the target lanelet is
  // inside the current lanelets id or following lanelets
  const std::string object_id = toHexString(object.object_id);
  if (object_buffer_.count(object_id) != 0) {
    const std::vector<lanelet::ConstLanelet> & possible_lanelet =
      object_buffer_.at(object_id).back().future_possible_lanelets;

    bool not_in_possible_lanelet =
      std::find(possible_lanelet.begin(), possible_lanelet.end(), lanelet.second) ==
      possible_lanelet.end();
    if (not_in_possible_lanelet) {
      return false;
    }
  }

  // Step3. Calculate the angle difference between the lane angle and obstacle angle
  double object_yaw = getObjectYaw(object);
  const double lane_yaw = lanelet::utils::getLaneletAngle(
    lanelet.second, object.kinematics.pose_with_covariance.pose.position);
  const double delta_yaw = object_yaw - lane_yaw;
  const double normalized_delta_yaw = std::atan2(std::sin(delta_yaw), std::cos(delta_yaw));
  const double abs_norm_delta = std::fabs(normalized_delta_yaw);

  // Step4. Check if the closest lanelet is valid, and add all
  // of the lanelets that are below max_dist and max_delta_yaw
  if (
    lanelet.first < dist_threshold_for_searching_lanelet_ &&
    abs_norm_delta < delta_yaw_threshold_for_searching_lanelet_) {
    return true;
  }

  return false;
}

bool MapBasedPredictionROS::getClosestLanelets(
  const autoware_auto_perception_msgs::msg::TrackedObject & object,
  const lanelet::LaneletMapPtr & lanelet_map_ptr_, lanelet::ConstLanelets & closest_lanelets)
{
  std::chrono::high_resolution_clock::time_point begin = std::chrono::high_resolution_clock::now();

  // obstacle point
  lanelet::BasicPoint2d search_point(
    object.kinematics.pose_with_covariance.pose.position.x,
    object.kinematics.pose_with_covariance.pose.position.y);

  // nearest lanelet
  std::vector<std::pair<double, lanelet::Lanelet>> surrounding_lanelets =
    lanelet::geometry::findNearest(lanelet_map_ptr_->laneletLayer, search_point, 10);

  std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds time = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
  debug_accumulated_time_ += time.count() / (1000.0 * 1000.0);

  // No Closest Lanelets
  if (surrounding_lanelets.empty()) {
    return false;
  }

  const double MIN_DIST = 1e-6;
  bool found_target_closest_lanelet = false;
  for (const auto & lanelet : surrounding_lanelets) {
    // Check if the close lanelets meet the necessary condition for start lanelets
    if (checkCloseLaneletCondition(lanelet, object, search_point)) {
      // If the lanelet meets the condition,
      // then check if similar lanelet is inside the closest lanelet
      bool is_duplicate = false;
      for (const auto & closest_lanelet : closest_lanelets) {
        const auto lanelet_end_p = lanelet.second.centerline2d().back();
        const auto closest_lanelet_end_p = closest_lanelet.centerline2d().back();
        const double dist = std::hypot(
          lanelet_end_p.x() - closest_lanelet_end_p.x(),
          lanelet_end_p.y() - closest_lanelet_end_p.y());
        if (dist < MIN_DIST) {
          is_duplicate = true;
          break;
        }
      }
      if (is_duplicate) {
        continue;
      }

      found_target_closest_lanelet = true;
      closest_lanelets.push_back(lanelet.second);
    }
  }

  // If the closest lanelet is valid, return true
  return found_target_closest_lanelet;
}

void MapBasedPredictionROS::removeInvalidObject(const double current_time)
{
  std::vector<std::string> invalid_object_id;
  for (auto iter = object_buffer_.begin(); iter != object_buffer_.end(); ++iter) {
    const std::string object_id = iter->first;
    std::deque<ObjectData> & object_data = iter->second;

    // If object data is empty, we are going to delete the buffer for the obstacle
    if (object_data.empty()) {
      invalid_object_id.push_back(object_id);
      continue;
    }
    const double latest_object_time = rclcpp::Time(object_data.back().pose.header.stamp).seconds();

    // Delete Old Objects
    if (current_time - latest_object_time > 2.0) {
      invalid_object_id.push_back(object_id);
      continue;
    }

    // Delete old information
    while (!object_data.empty()) {
      const double post_object_time = rclcpp::Time(object_data.front().pose.header.stamp).seconds();
      if (current_time - post_object_time > 2.0) {
        // Delete Old Position
        object_data.pop_front();
      } else {
        break;
      }
    }

    if (object_data.empty()) {
      invalid_object_id.push_back(object_id);
      continue;
    }
  }

  for (const auto & key : invalid_object_id) {
    object_buffer_.erase(key);
  }
}

bool MapBasedPredictionROS::updateObjectBuffer(
  const std_msgs::msg::Header & header,
  const autoware_auto_perception_msgs::msg::TrackedObject & object,
  lanelet::ConstLanelets & current_lanelets)
{
  using Label = autoware_auto_perception_msgs::msg::ObjectClassification;
  // Ignore non-vehicle object
  if (
    object.classification.front().label != Label::CAR &&
    object.classification.front().label != Label::BUS &&
    object.classification.front().label != Label::TRUCK) {
    return false;
  }

  // Get the current Lanelet
  std::string object_id = toHexString(object.object_id);

  // Check whether the object is on the road
  if (!getClosestLanelets(object, lanelet_map_ptr_, current_lanelets)) {
    // This Object is not on the road
    return false;
  }

  // Get current Pose
  geometry_msgs::msg::PoseStamped current_object_pose;
  current_object_pose.header = header;
  current_object_pose.pose = object.kinematics.pose_with_covariance.pose;

  // Update Object Buffer
  if (object_buffer_.count(object_id) == 0) {
    // New Object
    ObjectData single_object_data;
    single_object_data.current_lanelets = current_lanelets;
    single_object_data.future_possible_lanelets = current_lanelets;
    single_object_data.pose = current_object_pose;
    const double object_yaw = getObjectYaw(object);
    single_object_data.pose.pose.orientation =
      tier4_autoware_utils::createQuaternionFromYaw(object_yaw);

    std::deque<ObjectData> object_data;
    object_data.push_back(single_object_data);

    // Create new key, value pair in the buffer
    object_buffer_.emplace(object_id, object_data);
  } else {
    // Object that is already in the object buffer
    std::deque<ObjectData> & object_data = object_buffer_.at(object_id);

    ObjectData single_object_data;
    single_object_data.current_lanelets = current_lanelets;
    single_object_data.future_possible_lanelets = current_lanelets;
    single_object_data.pose = current_object_pose;
    const double object_yaw = getObjectYaw(object);
    single_object_data.pose.pose.orientation =
      tier4_autoware_utils::createQuaternionFromYaw(object_yaw);

    // push new object data
    object_data.push_back(single_object_data);
  }

  // If the obstacle is too slow, we do linear prediction
  if (
    std::fabs(object.kinematics.twist_with_covariance.twist.linear.x) <
    min_velocity_for_map_based_prediction_) {
    return false;
  }

  return true;
}

void MapBasedPredictionROS::updatePossibleLanelets(
  const std::string object_id, const lanelet::routing::LaneletPaths & paths)
{
  if (object_buffer_.count(object_id) != 0) {
    std::vector<lanelet::ConstLanelet> & possible_lanelets =
      object_buffer_.at(object_id).back().future_possible_lanelets;
    for (const auto & path : paths) {
      for (const auto & lanelet : path) {
        bool not_in_buffer =
          std::find(possible_lanelets.begin(), possible_lanelets.end(), lanelet) ==
          possible_lanelets.end();
        if (not_in_buffer) {
          possible_lanelets.push_back(lanelet);
        }
      }
    }
  }
}

void MapBasedPredictionROS::addValidPath(
  const lanelet::routing::LaneletPaths & candidate_paths,
  lanelet::routing::LaneletPaths & valid_paths)
{
  // Check if candidate paths are already in the valid paths
  for (const auto & candidate_path : candidate_paths) {
    bool already_searched = false;
    for (const auto & valid_path : valid_paths) {
      for (const auto & llt : valid_path) {
        if (candidate_path.back().id() == llt.id()) {
          already_searched = true;
        }
      }
    }
    if (!already_searched) {
      valid_paths.push_back(candidate_path);
    }
  }
}

double MapBasedPredictionROS::calcRightLateralOffset(
  const lanelet::ConstLineString2d & bound_line, const geometry_msgs::msg::Pose & search_pose)
{
  std::vector<geometry_msgs::msg::Point> bound_path(bound_line.size());
  for (size_t i = 0; i < bound_path.size(); ++i) {
    const double x = bound_line[i].x();
    const double y = bound_line[i].y();
    bound_path[i] = tier4_autoware_utils::createPoint(x, y, 0.0);
  }

  return std::fabs(tier4_autoware_utils::calcLateralOffset(bound_path, search_pose.position));
}

double MapBasedPredictionROS::calcLeftLateralOffset(
  const lanelet::ConstLineString2d & bound_line, const geometry_msgs::msg::Pose & search_pose)
{
  return -calcRightLateralOffset(bound_line, search_pose);
}

Maneuver MapBasedPredictionROS::detectLaneChange(
  const autoware_auto_perception_msgs::msg::TrackedObject & object,
  const lanelet::ConstLanelet & current_lanelet, const double current_time)
{
  // Step1. Check if we have the object in the buffer
  const std::string object_id = toHexString(object.object_id);
  if (object_buffer_.count(object_id) == 0) {
    return Maneuver::LANE_FOLLOW;
  }

  // Object History Data
  const std::deque<ObjectData> object_info = object_buffer_.at(object_id);

  // Step2. Get the previous id
  int prev_id = static_cast<int>(object_info.size()) - 1;
  while (prev_id >= 0) {
    const double prev_time = rclcpp::Time(object_info.at(prev_id).pose.header.stamp).seconds();
    if (current_time - prev_time > history_time_length_) {
      break;
    }
    --prev_id;
  }

  if (prev_id < 0) {
    return Maneuver::LANE_FOLLOW;
  }

  // Step3. Get closest previous lanelet ID
  const auto prev_pose = object_info.at(static_cast<size_t>(prev_id)).pose;
  const lanelet::ConstLanelets prev_lanelets =
    object_info.at(static_cast<size_t>(prev_id)).current_lanelets;
  if (prev_lanelets.empty()) {
    return Maneuver::LANE_FOLLOW;
  }
  lanelet::ConstLanelet prev_lanelet = prev_lanelets.front();
  double closest_prev_yaw = std::numeric_limits<double>::max();
  for (const auto & lanelet : prev_lanelets) {
    const double lane_yaw = lanelet::utils::getLaneletAngle(lanelet, prev_pose.pose.position);
    const double delta_yaw = tf2::getYaw(prev_pose.pose.orientation) - lane_yaw;
    const double normalized_delta_yaw = tier4_autoware_utils::normalizeRadian(delta_yaw);
    if (normalized_delta_yaw < closest_prev_yaw) {
      closest_prev_yaw = normalized_delta_yaw;
      prev_lanelet = lanelet;
    }
  }

  // Step4. Check if the vehicle has changed lane
  const auto current_pose = object.kinematics.pose_with_covariance.pose;
  const double dist = tier4_autoware_utils::calcDistance2d(prev_pose, current_pose);
  lanelet::routing::LaneletPaths possible_paths =
    routing_graph_ptr_->possiblePaths(prev_lanelet, dist + 2.0, 0, false);
  bool has_lane_changed = true;
  for (const auto & path : possible_paths) {
    for (const auto & lanelet : path) {
      if (lanelet == current_lanelet) {
        has_lane_changed = false;
        break;
      }
    }
  }

  if (has_lane_changed) {
    return Maneuver::LANE_FOLLOW;
  }

  // Step5. Lane Change Detection
  const lanelet::ConstLineString2d prev_left_bound = prev_lanelet.leftBound2d();
  const lanelet::ConstLineString2d prev_right_bound = prev_lanelet.rightBound2d();
  const lanelet::ConstLineString2d current_left_bound = current_lanelet.leftBound2d();
  const lanelet::ConstLineString2d current_right_bound = current_lanelet.rightBound2d();
  const double prev_left_dist = calcLeftLateralOffset(prev_left_bound, prev_pose.pose);
  const double prev_right_dist = calcRightLateralOffset(prev_right_bound, prev_pose.pose);
  const double current_left_dist = calcLeftLateralOffset(current_left_bound, current_pose);
  const double current_right_dist = calcRightLateralOffset(current_right_bound, current_pose);
  const double prev_lane_width = std::fabs(prev_left_dist) + std::fabs(prev_right_dist);
  const double current_lane_width = std::fabs(current_left_dist) + std::fabs(current_right_dist);
  if (prev_lane_width < 1e-3 || current_lane_width < 1e-3) {
    RCLCPP_ERROR(get_logger(), "[Map Based Prediction]: Lane Width is too small");
    return Maneuver::LANE_FOLLOW;
  }

  const double current_left_dist_ratio = current_left_dist / current_lane_width;
  const double current_right_dist_ratio = current_right_dist / current_lane_width;
  const double diff_left_current_prev = current_left_dist - prev_left_dist;
  const double diff_right_current_prev = current_right_dist - prev_right_dist;

  if (
    current_left_dist_ratio > dist_ratio_threshold_to_left_bound_ &&
    diff_left_current_prev > diff_dist_threshold_to_left_bound_) {
    return Maneuver::LEFT_LANE_CHANGE;
  } else if (
    current_right_dist_ratio < dist_ratio_threshold_to_right_bound_ &&
    diff_right_current_prev < diff_dist_threshold_to_right_bound_) {
    return Maneuver::RIGHT_LANE_CHANGE;
  }

  return Maneuver::LANE_FOLLOW;
}

void MapBasedPredictionROS::objectsCallback(
  const autoware_auto_perception_msgs::msg::TrackedObjects::ConstSharedPtr in_objects)
{
  debug_accumulated_time_ = 0.0;
  std::chrono::high_resolution_clock::time_point begin = std::chrono::high_resolution_clock::now();

  if (!lanelet_map_ptr_) {
    return;
  }

  geometry_msgs::msg::TransformStamped world2map_transform;
  geometry_msgs::msg::TransformStamped map2world_transform;
  geometry_msgs::msg::TransformStamped debug_map2lidar_transform;
  try {
    world2map_transform = tf_buffer_ptr_->lookupTransform(
      "map",                        // target
      in_objects->header.frame_id,  // src
      in_objects->header.stamp, rclcpp::Duration::from_seconds(0.1));
    map2world_transform = tf_buffer_ptr_->lookupTransform(
      in_objects->header.frame_id,  // target
      "map",                        // src
      in_objects->header.stamp, rclcpp::Duration::from_seconds(0.1));
    debug_map2lidar_transform = tf_buffer_ptr_->lookupTransform(
      "base_link",  // target
      "map",        // src
      rclcpp::Time(), rclcpp::Duration::from_seconds(0.1));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
    return;
  }

  ///////////////////////////////////////////////////////////
  /////////////////// Update Object Buffer //////////////////
  //////////////////////////////////////////////////////////
  const double objects_detected_time = rclcpp::Time(in_objects->header.stamp).seconds();
  removeInvalidObject(objects_detected_time);

  /////////////////////////////////////////////////////////
  ///////////////////// Prediction ///////////////////////
  ///////////////////////////////////////////////////////
  autoware_auto_perception_msgs::msg::PredictedObjects objects_without_map;
  objects_without_map.header = in_objects->header;
  DynamicObjectWithLanesArray prediction_input;
  prediction_input.header = in_objects->header;

  for (const auto & object : in_objects->objects) {
    std::string object_id = toHexString(object.object_id);
    DynamicObjectWithLanes transformed_object;
    transformed_object.object = object;
    if (in_objects->header.frame_id != "map") {
      geometry_msgs::msg::PoseStamped pose_in_map;
      geometry_msgs::msg::PoseStamped pose_orig;
      pose_orig.pose = object.kinematics.pose_with_covariance.pose;
      tf2::doTransform(pose_orig, pose_in_map, world2map_transform);
      transformed_object.object.kinematics.pose_with_covariance.pose = pose_in_map.pose;
    }

    std_msgs::msg::Header transformed_header = in_objects->header;
    transformed_header.frame_id = "map";
    lanelet::ConstLanelets start_lanelets;  // current lanelet
    if (!updateObjectBuffer(transformed_header, transformed_object.object, start_lanelets)) {
      objects_without_map.objects.push_back(
        map_based_prediction_->convertToPredictedObject(transformed_object.object));
      continue;
    }

    // Obtain valid Paths
    const double delta_horizon = 1.0;
    const double obj_vel = object.kinematics.twist_with_covariance.twist.linear.x;
    lanelet::routing::LaneletPaths paths;
    for (const auto & start_lanelet : start_lanelets) {
      // Step1. Lane Change Detection
      // First: Right to Left Detection Result
      // Second: Left to Right Detection Result
      const Maneuver maneuver = detectLaneChange(object, start_lanelet, objects_detected_time);

      // Step2. Get the left lanelet
      lanelet::routing::LaneletPaths left_paths;
      auto opt_left = routing_graph_ptr_->left(start_lanelet);
      if (!!opt_left) {
        for (double horizon = prediction_time_horizon_; horizon > 0; horizon -= delta_horizon) {
          const double search_dist = horizon * obj_vel + 10.0;
          lanelet::routing::LaneletPaths tmp_paths =
            routing_graph_ptr_->possiblePaths(*opt_left, search_dist, 0, false);
          addValidPath(tmp_paths, left_paths);
        }
      }

      // Step3. Get the right lanelet
      lanelet::routing::LaneletPaths right_paths;
      auto opt_right = routing_graph_ptr_->right(start_lanelet);
      if (!!opt_right) {
        for (double horizon = prediction_time_horizon_; horizon > 0; horizon -= delta_horizon) {
          const double search_dist = horizon * obj_vel + 10.0;
          lanelet::routing::LaneletPaths tmp_paths =
            routing_graph_ptr_->possiblePaths(*opt_right, search_dist, 0, false);
          addValidPath(tmp_paths, right_paths);
        }
      }

      // Step4. Get the centerline
      lanelet::routing::LaneletPaths center_paths;
      for (double horizon = prediction_time_horizon_; horizon > 0; horizon -= delta_horizon) {
        const double search_dist = horizon * obj_vel + 10.0;
        lanelet::routing::LaneletPaths tmp_paths =
          routing_graph_ptr_->possiblePaths(start_lanelet, search_dist, 0, false);
        addValidPath(tmp_paths, center_paths);
      }

      // Step5. Insert Valid Paths
      if (!left_paths.empty() && maneuver == Maneuver::LEFT_LANE_CHANGE) {
        paths.insert(paths.end(), left_paths.begin(), left_paths.end());
      } else if (!right_paths.empty() && maneuver == Maneuver::RIGHT_LANE_CHANGE) {
        paths.insert(paths.end(), right_paths.begin(), right_paths.end());
      } else {
        paths.insert(paths.end(), center_paths.begin(), center_paths.end());
      }
    }
    // If there is no valid path, we'll mark this object as map-less object
    if (paths.empty()) {
      objects_without_map.objects.push_back(
        map_based_prediction_->convertToPredictedObject(transformed_object.object));
      continue;
    }

    // Update Possible lanelet in the object buffer
    updatePossibleLanelets(object_id, paths);

    std::vector<std::vector<geometry_msgs::msg::Pose>> tmp_paths;
    std::vector<double> tmp_confidence;
    for (const auto & path : paths) {
      std::vector<geometry_msgs::msg::Pose> tmp_path;

      // Insert Positions. Note that we insert points from previous lanelet
      if (!path.empty()) {
        lanelet::ConstLanelets prev_lanelets = routing_graph_ptr_->previous(path.front());
        if (!prev_lanelets.empty()) {
          lanelet::ConstLanelet prev_lanelet = prev_lanelets.front();
          for (const auto & point : prev_lanelet.centerline()) {
            geometry_msgs::msg::Pose tmp_pose;
            tmp_pose.position.x = point.x();
            tmp_pose.position.y = point.y();
            tmp_pose.position.z = point.z();
            tmp_path.push_back(tmp_pose);
          }
        }
      }

      for (const auto & lanelet : path) {
        for (const auto & point : lanelet.centerline()) {
          geometry_msgs::msg::Pose tmp_pose;
          tmp_pose.position.x = point.x();
          tmp_pose.position.y = point.y();
          tmp_pose.position.z = point.z();

          // Prevent from inserting same points
          if (!tmp_path.empty()) {
            const auto prev_pose = tmp_path.back();
            const double tmp_dist = tier4_autoware_utils::calcDistance2d(prev_pose, tmp_pose);
            if (tmp_dist < 1e-6) {
              continue;
            }
          }

          tmp_path.push_back(tmp_pose);
        }
      }

      if (tmp_path.size() < 2) {
        continue;
      }

      // Compute yaw angles
      for (size_t pose_id = 0; pose_id < tmp_path.size() - 1; ++pose_id) {
        double tmp_yaw = std::atan2(
          tmp_path.at(pose_id + 1).position.y - tmp_path.at(pose_id).position.y,
          tmp_path.at(pose_id + 1).position.x - tmp_path.at(pose_id).position.x);
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, tmp_yaw);
        tmp_path.at(pose_id).orientation = tf2::toMsg(quat);
      }
      tmp_path.back().orientation = tmp_path.at(tmp_path.size() - 2).orientation;

      //////////////////////////////////////////////////////////////////////
      // Calculate Confidence of each path(centerline) for this obstacle //
      ////////////////////////////////////////////////////////////////////
      const double confidence = calculateLikelihood(tmp_path, transformed_object.object);
      // Ignore a path that has too low confidence
      if (confidence < 1e-6) {
        continue;
      }

      tmp_paths.push_back(tmp_path);
      tmp_confidence.push_back(confidence);
    }

    transformed_object.lanes = tmp_paths;
    transformed_object.confidence = tmp_confidence;
    prediction_input.objects.push_back(transformed_object);
  }

  std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds time = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);

  std::vector<autoware_auto_perception_msgs::msg::PredictedObject> out_objects_in_map;
  map_based_prediction_->doPrediction(prediction_input, out_objects_in_map);
  autoware_auto_perception_msgs::msg::PredictedObjects output;
  output.header = in_objects->header;
  output.header.frame_id = "map";
  output.objects = out_objects_in_map;

  std::vector<autoware_auto_perception_msgs::msg::PredictedObject> out_objects_without_map;
  map_based_prediction_->doLinearPrediction(objects_without_map, out_objects_without_map);
  output.objects.insert(
    output.objects.begin(), out_objects_without_map.begin(), out_objects_without_map.end());
  pub_objects_->publish(output);
}

void MapBasedPredictionROS::mapCallback(
  const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Start loading lanelet");
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
  RCLCPP_INFO(get_logger(), "Map is loaded");
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MapBasedPredictionROS)
