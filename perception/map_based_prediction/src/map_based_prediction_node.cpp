// Copyright 2021 Tier IV, Inc.
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

#include "map_based_prediction/map_based_prediction_node.hpp"

#include <interpolation/linear_interpolation.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace map_based_prediction
{
namespace
{
std::string toHexString(const unique_identifier_msgs::msg::UUID & id)
{
  std::stringstream ss;
  for (auto i = 0; i < 16; ++i) {
    ss << std::hex << std::setfill('0') << std::setw(2) << +id.uuid[i];
  }
  return ss.str();
}

lanelet::ConstLanelets getLanelets(const map_based_prediction::LaneletsData & data)
{
  lanelet::ConstLanelets lanelets;
  for (const auto & lanelet_data : data) {
    lanelets.push_back(lanelet_data.lanelet);
  }

  return lanelets;
}

EntryPoint getCrosswalkEntryPoint(const lanelet::ConstLanelet & crosswalk)
{
  const auto & r_p_front = crosswalk.rightBound().front();
  const auto & l_p_front = crosswalk.leftBound().front();
  const Eigen::Vector2d front_entry_point(
    (r_p_front.x() + l_p_front.x()) / 2.0, (r_p_front.y() + l_p_front.y()) / 2.0);

  const auto & r_p_back = crosswalk.rightBound().back();
  const auto & l_p_back = crosswalk.leftBound().back();
  const Eigen::Vector2d back_entry_point(
    (r_p_back.x() + l_p_back.x()) / 2.0, (r_p_back.y() + l_p_back.y()) / 2.0);

  return std::make_pair(front_entry_point, back_entry_point);
}

bool withinLanelet(const TrackedObject & object, const lanelet::ConstLanelet & lanelet)
{
  using Point = boost::geometry::model::d2::point_xy<double>;

  const auto & obj_pos = object.kinematics.pose_with_covariance.pose.position;
  const Point p_object{obj_pos.x, obj_pos.y};

  auto polygon = lanelet.polygon2d().basicPolygon();
  boost::geometry::correct(polygon);

  return boost::geometry::within(p_object, polygon);
}

bool withinRoadLanelet(const TrackedObject & object, const lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  using Point = boost::geometry::model::d2::point_xy<double>;

  const auto & obj_pos = object.kinematics.pose_with_covariance.pose.position;
  const Point p_object{obj_pos.x, obj_pos.y};

  lanelet::BasicPoint2d search_point(obj_pos.x, obj_pos.y);
  // nearest lanelet
  constexpr double search_radius = 10.0;  // [m]
  const auto surrounding_lanelets =
    lanelet::geometry::findNearest(lanelet_map_ptr->laneletLayer, search_point, search_radius);

  for (const auto & lanelet : surrounding_lanelets) {
    if (lanelet.second.hasAttribute(lanelet::AttributeName::Subtype)) {
      lanelet::Attribute attr = lanelet.second.attribute(lanelet::AttributeName::Subtype);
      if (
        attr.value() == lanelet::AttributeValueString::Crosswalk ||
        attr.value() == lanelet::AttributeValueString::Walkway) {
        continue;
      }
    }

    if (withinLanelet(object, lanelet.second)) {
      return true;
    }
  }

  return false;
}

boost::optional<EntryPoint> isReachableEntryPoint(
  const TrackedObject & object, const EntryPoint & entry_point,
  const lanelet::LaneletMapPtr & lanelet_map_ptr, const double time_horizon,
  const double min_object_vel)
{
  using Point = boost::geometry::model::d2::point_xy<double>;
  using Line = boost::geometry::model::linestring<Point>;

  const auto & obj_pos = object.kinematics.pose_with_covariance.pose.position;
  const auto & obj_vel = object.kinematics.twist_with_covariance.twist.linear;
  const auto yaw = tier4_autoware_utils::getRPY(object.kinematics.pose_with_covariance.pose).z;

  const auto & p1 = entry_point.first;
  const auto & p2 = entry_point.second;

  auto ret = std::make_pair(p1, p2);
  auto distance_pedestrian_to_p1 = std::hypot(p1.x() - obj_pos.x, p1.y() - obj_pos.y);
  auto distance_pedestrian_to_p2 = std::hypot(p2.x() - obj_pos.x, p2.y() - obj_pos.y);

  if (distance_pedestrian_to_p2 < distance_pedestrian_to_p1) {
    std::swap(ret.first, ret.second);
    std::swap(distance_pedestrian_to_p1, distance_pedestrian_to_p2);
  }

  constexpr double stop_velocity_th = 0.14;  // [m/s]
  const auto estimated_velocity = std::hypot(obj_vel.x, obj_vel.y);
  const auto is_stop_object = estimated_velocity < stop_velocity_th;
  const auto velocity = std::max(min_object_vel, estimated_velocity);

  lanelet::BasicPoint2d search_point(obj_pos.x, obj_pos.y);
  // nearest lanelet
  const auto surrounding_lanelets = lanelet::geometry::findNearest(
    lanelet_map_ptr->laneletLayer, search_point, time_horizon * velocity);

  bool first_intersect_load = false;
  bool second_intersect_load = false;
  std::vector<Point> intersects_first;
  std::vector<Point> intersects_second;
  for (const auto & lanelet : surrounding_lanelets) {
    if (withinLanelet(object, lanelet.second)) {
      return {};
    }

    lanelet::Attribute attr = lanelet.second.attribute(lanelet::AttributeName::Subtype);
    if (attr.value() != "road") {
      continue;
    }

    {
      const Line object_to_entry_point{{obj_pos.x, obj_pos.y}, {ret.first.x(), ret.first.y()}};
      std::vector<Point> tmp_intersects;
      boost::geometry::intersection(
        object_to_entry_point, lanelet.second.polygon2d().basicPolygon(), tmp_intersects);
      for (const auto & p : tmp_intersects) {
        intersects_first.push_back(p);
      }
    }

    {
      const Line object_to_entry_point{{obj_pos.x, obj_pos.y}, {ret.second.x(), ret.second.y()}};
      std::vector<Point> tmp_intersects;
      boost::geometry::intersection(
        object_to_entry_point, lanelet.second.polygon2d().basicPolygon(), tmp_intersects);
      for (const auto & p : tmp_intersects) {
        intersects_second.push_back(p);
      }
    }
  }

  if (1 < intersects_first.size()) {
    first_intersect_load = true;
  }

  if (1 < intersects_second.size()) {
    second_intersect_load = true;
  }

  if (first_intersect_load && second_intersect_load) {
    return {};
  }

  if (first_intersect_load && !second_intersect_load) {
    std::swap(ret.first, ret.second);
  }

  const Eigen::Vector2d pedestrian_to_crosswalk(
    (ret.first.x() + ret.second.x()) / 2.0 - obj_pos.x,
    (ret.first.y() + ret.second.y()) / 2.0 - obj_pos.y);
  const Eigen::Vector2d pedestrian_heading_direction(
    obj_vel.x * std::cos(yaw), obj_vel.x * std::sin(yaw));
  const auto reachable =
    std::min(distance_pedestrian_to_p1, distance_pedestrian_to_p2) < velocity * time_horizon;
  const auto heading_for_crosswalk =
    pedestrian_to_crosswalk.dot(pedestrian_heading_direction) > 0.0;

  if ((reachable && heading_for_crosswalk) || (reachable && is_stop_object)) {
    return ret;
  }

  return {};
}

bool hasPotentialToReach(
  const TrackedObject & object, const Eigen::Vector2d & point, const double time_horizon,
  const double min_object_vel)
{
  const auto & obj_pos = object.kinematics.pose_with_covariance.pose.position;
  const auto & obj_vel = object.kinematics.twist_with_covariance.twist.linear;
  const auto yaw = tier4_autoware_utils::getRPY(object.kinematics.pose_with_covariance.pose).z;

  constexpr double stop_velocity_th = 0.14;  // [m/s]
  const auto estimated_velocity = std::hypot(obj_vel.x, obj_vel.y);
  const auto is_stop_object = estimated_velocity < stop_velocity_th;
  const auto velocity = std::max(min_object_vel, estimated_velocity);

  const Eigen::Vector2d pedestrian_to_crosswalk(point.x() - obj_pos.x, point.y() - obj_pos.y);
  const Eigen::Vector2d pedestrian_heading_direction(
    obj_vel.x * std::cos(yaw), obj_vel.x * std::sin(yaw));
  const auto heading_for_crosswalk =
    pedestrian_to_crosswalk.dot(pedestrian_heading_direction) > 0.0;

  const auto reachable = pedestrian_to_crosswalk.norm() < velocity * time_horizon;

  if (reachable && (heading_for_crosswalk || is_stop_object)) {
    return true;
  }

  return false;
}
}  // namespace

MapBasedPredictionNode::MapBasedPredictionNode(const rclcpp::NodeOptions & node_options)
: Node("map_based_prediction", node_options), debug_accumulated_time_(0.0)
{
  enable_delay_compensation_ = declare_parameter("enable_delay_compensation", true);
  prediction_time_horizon_ = declare_parameter("prediction_time_horizon", 10.0);
  prediction_sampling_time_interval_ = declare_parameter("prediction_sampling_delta_time", 0.5);
  min_velocity_for_map_based_prediction_ =
    declare_parameter("min_velocity_for_map_based_prediction", 1.0);
  min_crosswalk_user_velocity_ = declare_parameter("min_crosswalk_user_velocity", 1.0);
  dist_threshold_for_searching_lanelet_ =
    declare_parameter("dist_threshold_for_searching_lanelet", 3.0);
  delta_yaw_threshold_for_searching_lanelet_ =
    declare_parameter("delta_yaw_threshold_for_searching_lanelet", 0.785);
  sigma_lateral_offset_ = declare_parameter("sigma_lateral_offset", 0.5);
  sigma_yaw_angle_deg_ = declare_parameter("sigma_yaw_angle_deg", 5.0);
  object_buffer_time_length_ = declare_parameter("object_buffer_time_length", 2.0);
  history_time_length_ = declare_parameter("history_time_length", 1.0);
  dist_ratio_threshold_to_left_bound_ =
    declare_parameter("dist_ratio_threshold_to_left_bound", -0.5);
  dist_ratio_threshold_to_right_bound_ =
    declare_parameter("dist_ratio_threshold_to_right_bound", 0.5);
  diff_dist_threshold_to_left_bound_ = declare_parameter("diff_dist_threshold_to_left_bound", 0.29);
  diff_dist_threshold_to_right_bound_ =
    declare_parameter("diff_dist_threshold_to_right_bound", -0.29);
  reference_path_resolution_ = declare_parameter("reference_path_resolution", 0.5);

  path_generator_ = std::make_shared<PathGenerator>(
    prediction_time_horizon_, prediction_sampling_time_interval_, min_crosswalk_user_velocity_);

  sub_objects_ = this->create_subscription<TrackedObjects>(
    "/perception/object_recognition/tracking/objects", 1,
    std::bind(&MapBasedPredictionNode::objectsCallback, this, std::placeholders::_1));
  sub_map_ = this->create_subscription<HADMapBin>(
    "/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&MapBasedPredictionNode::mapCallback, this, std::placeholders::_1));

  pub_objects_ = this->create_publisher<PredictedObjects>("objects", rclcpp::QoS{1});
  pub_debug_markers_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("maneuver", rclcpp::QoS{1});
}

PredictedObjectKinematics MapBasedPredictionNode::convertToPredictedKinematics(
  const TrackedObjectKinematics & tracked_object)
{
  PredictedObjectKinematics output;
  output.initial_pose_with_covariance = tracked_object.pose_with_covariance;
  output.initial_twist_with_covariance = tracked_object.twist_with_covariance;
  output.initial_acceleration_with_covariance = tracked_object.acceleration_with_covariance;
  return output;
}

PredictedObject MapBasedPredictionNode::convertToPredictedObject(
  const TrackedObject & tracked_object)
{
  PredictedObject predicted_object;
  predicted_object.kinematics = convertToPredictedKinematics(tracked_object.kinematics);
  predicted_object.classification = tracked_object.classification;
  predicted_object.object_id = tracked_object.object_id;
  predicted_object.shape = tracked_object.shape;
  predicted_object.existence_probability = tracked_object.existence_probability;

  return predicted_object;
}

void MapBasedPredictionNode::mapCallback(const HADMapBin::ConstSharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "[Map Based Prediction]: Start loading lanelet");
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
  RCLCPP_INFO(get_logger(), "[Map Based Prediction]: Map is loaded");

  const auto all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  const auto crosswalks = lanelet::utils::query::crosswalkLanelets(all_lanelets);
  const auto walkways = lanelet::utils::query::walkwayLanelets(all_lanelets);
  crosswalks_.insert(crosswalks_.end(), crosswalks.begin(), crosswalks.end());
  crosswalks_.insert(crosswalks_.end(), walkways.begin(), walkways.end());
}

void MapBasedPredictionNode::objectsCallback(const TrackedObjects::ConstSharedPtr in_objects)
{
  // Guard for map pointer and frame transformation
  if (!lanelet_map_ptr_) {
    return;
  }

  auto world2map_transform = transform_listener_.getTransform(
    "map",                        // target
    in_objects->header.frame_id,  // src
    in_objects->header.stamp, rclcpp::Duration::from_seconds(0.1));
  auto map2world_transform = transform_listener_.getTransform(
    in_objects->header.frame_id,  // target
    "map",                        // src
    in_objects->header.stamp, rclcpp::Duration::from_seconds(0.1));
  auto debug_map2lidar_transform = transform_listener_.getTransform(
    "base_link",  // target
    "map",        // src
    rclcpp::Time(), rclcpp::Duration::from_seconds(0.1));

  if (!world2map_transform || !map2world_transform || !debug_map2lidar_transform) {
    return;
  }

  // Remove old objects information in object history
  const double objects_detected_time = rclcpp::Time(in_objects->header.stamp).seconds();
  removeOldObjectsHistory(objects_detected_time);

  // result output
  PredictedObjects output;
  output.header = in_objects->header;
  output.header.frame_id = "map";

  // result debug
  visualization_msgs::msg::MarkerArray debug_markers;

  for (const auto & object : in_objects->objects) {
    std::string object_id = toHexString(object.object_id);
    TrackedObject transformed_object = object;

    // transform object frame if it's based on map frame
    if (in_objects->header.frame_id != "map") {
      geometry_msgs::msg::PoseStamped pose_in_map;
      geometry_msgs::msg::PoseStamped pose_orig;
      pose_orig.pose = object.kinematics.pose_with_covariance.pose;
      tf2::doTransform(pose_orig, pose_in_map, *world2map_transform);
      transformed_object.kinematics.pose_with_covariance.pose = pose_in_map.pose;
    }

    const auto & label = transformed_object.classification.front().label;

    // For crosswalk user
    if (label == ObjectClassification::PEDESTRIAN || label == ObjectClassification::BICYCLE) {
      const auto predicted_object = getPredictedObjectAsCrosswalkUser(transformed_object);
      output.objects.push_back(predicted_object);
      // For road user
    } else if (
      label == ObjectClassification::CAR || label == ObjectClassification::BUS ||
      label == ObjectClassification::TRAILER || label == ObjectClassification::MOTORCYCLE ||
      label == ObjectClassification::TRUCK) {
      // Update object yaw and velocity
      updateObjectData(transformed_object);

      // Get Closest Lanelet
      const auto current_lanelets = getCurrentLanelets(transformed_object);

      // Update Objects History
      updateObjectsHistory(output.header, transformed_object, current_lanelets);

      // For off lane obstacles
      if (current_lanelets.empty()) {
        PredictedPath predicted_path =
          path_generator_->generatePathForOffLaneVehicle(transformed_object);
        predicted_path.confidence = 1.0;
        if (predicted_path.path.empty()) {
          continue;
        }

        auto predicted_object = convertToPredictedObject(transformed_object);
        predicted_object.kinematics.predicted_paths.push_back(predicted_path);
        output.objects.push_back(predicted_object);
        continue;
      }

      // For too-slow vehicle
      if (
        std::fabs(transformed_object.kinematics.twist_with_covariance.twist.linear.x) <
        min_velocity_for_map_based_prediction_) {
        PredictedPath predicted_path =
          path_generator_->generatePathForLowSpeedVehicle(transformed_object);
        predicted_path.confidence = 1.0;
        if (predicted_path.path.empty()) {
          continue;
        }

        auto predicted_object = convertToPredictedObject(transformed_object);
        predicted_object.kinematics.predicted_paths.push_back(predicted_path);
        output.objects.push_back(predicted_object);
        continue;
      }

      // Get Predicted Reference Path for Each Maneuver and current lanelets
      // return: <probability, paths>
      const auto ref_paths =
        getPredictedReferencePath(transformed_object, current_lanelets, objects_detected_time);

      // If predicted reference path is empty, assume this object is out of the lane
      if (ref_paths.empty()) {
        PredictedPath predicted_path =
          path_generator_->generatePathForLowSpeedVehicle(transformed_object);
        predicted_path.confidence = 1.0;
        if (predicted_path.path.empty()) {
          continue;
        }

        auto predicted_object = convertToPredictedObject(transformed_object);
        predicted_object.kinematics.predicted_paths.push_back(predicted_path);
        output.objects.push_back(predicted_object);
        continue;
      }

      // Get Debug Marker for On Lane Vehicles
      const auto max_prob_path = std::max_element(
        ref_paths.begin(), ref_paths.end(),
        [](const PredictedRefPath & a, const PredictedRefPath & b) {
          return a.probability < b.probability;
        });
      const auto debug_marker =
        getDebugMarker(object, max_prob_path->maneuver, debug_markers.markers.size());
      debug_markers.markers.push_back(debug_marker);

      // Generate Predicted Path
      std::vector<PredictedPath> predicted_paths;
      for (const auto & ref_path : ref_paths) {
        PredictedPath predicted_path =
          path_generator_->generatePathForOnLaneVehicle(transformed_object, ref_path.path);
        predicted_path.confidence = ref_path.probability;

        predicted_paths.push_back(predicted_path);
      }

      // Normalize Path Confidence and output the predicted object
      {
        float sum_confidence = 0.0;
        for (const auto & predicted_path : predicted_paths) {
          sum_confidence += predicted_path.confidence;
        }
        const float min_sum_confidence_value = 1e-3;
        sum_confidence = std::max(sum_confidence, min_sum_confidence_value);

        for (auto & predicted_path : predicted_paths) {
          predicted_path.confidence = predicted_path.confidence / sum_confidence;
        }

        auto predicted_object = convertToPredictedObject(transformed_object);
        for (const auto & predicted_path : predicted_paths) {
          predicted_object.kinematics.predicted_paths.push_back(predicted_path);
        }
        output.objects.push_back(predicted_object);
      }
      // For unknown object
    } else {
      auto predicted_object = convertToPredictedObject(transformed_object);
      PredictedPath predicted_path =
        path_generator_->generatePathForNonVehicleObject(transformed_object);
      predicted_path.confidence = 1.0;

      predicted_object.kinematics.predicted_paths.push_back(predicted_path);
      output.objects.push_back(predicted_object);
    }
  }

  // Publish Results
  pub_objects_->publish(output);
  pub_debug_markers_->publish(debug_markers);
}

PredictedObject MapBasedPredictionNode::getPredictedObjectAsCrosswalkUser(
  const TrackedObject & object)
{
  auto predicted_object = convertToPredictedObject(object);
  {
    PredictedPath predicted_path = path_generator_->generatePathForNonVehicleObject(object);
    predicted_path.confidence = 1.0;

    predicted_object.kinematics.predicted_paths.push_back(predicted_path);
  }

  boost::optional<lanelet::ConstLanelet> crossing_crosswalk{boost::none};
  for (const auto & crosswalk : crosswalks_) {
    if (withinLanelet(object, crosswalk)) {
      crossing_crosswalk = crosswalk;
      break;
    }
  }

  if (crossing_crosswalk) {
    const auto entry_point = getCrosswalkEntryPoint(crossing_crosswalk.get());

    if (hasPotentialToReach(
          object, entry_point.first, std::numeric_limits<double>::max(),
          min_crosswalk_user_velocity_)) {
      PredictedPath predicted_path =
        path_generator_->generatePathToTargetPoint(object, entry_point.first);
      predicted_path.confidence = 1.0;
      predicted_object.kinematics.predicted_paths.push_back(predicted_path);
    }

    if (hasPotentialToReach(
          object, entry_point.second, std::numeric_limits<double>::max(),
          min_crosswalk_user_velocity_)) {
      PredictedPath predicted_path =
        path_generator_->generatePathToTargetPoint(object, entry_point.second);
      predicted_path.confidence = 1.0;
      predicted_object.kinematics.predicted_paths.push_back(predicted_path);
    }

  } else if (withinRoadLanelet(object, lanelet_map_ptr_)) {
    lanelet::ConstLanelet closest_crosswalk{};
    const auto & obj_pose = object.kinematics.pose_with_covariance.pose;
    const auto found_closest_crosswalk =
      lanelet::utils::query::getClosestLanelet(crosswalks_, obj_pose, &closest_crosswalk);

    if (found_closest_crosswalk) {
      const auto entry_point = getCrosswalkEntryPoint(closest_crosswalk);

      if (hasPotentialToReach(
            object, entry_point.first, prediction_time_horizon_ * 2.0,
            min_crosswalk_user_velocity_)) {
        PredictedPath predicted_path =
          path_generator_->generatePathToTargetPoint(object, entry_point.first);
        predicted_path.confidence = 1.0;
        predicted_object.kinematics.predicted_paths.push_back(predicted_path);
      }

      if (hasPotentialToReach(
            object, entry_point.second, prediction_time_horizon_ * 2.0,
            min_crosswalk_user_velocity_)) {
        PredictedPath predicted_path =
          path_generator_->generatePathToTargetPoint(object, entry_point.second);
        predicted_path.confidence = 1.0;
        predicted_object.kinematics.predicted_paths.push_back(predicted_path);
      }
    }

  } else {
    for (const auto & crosswalk : crosswalks_) {
      const auto entry_point = getCrosswalkEntryPoint(crosswalk);

      const auto reachable_first = hasPotentialToReach(
        object, entry_point.first, prediction_time_horizon_, min_crosswalk_user_velocity_);
      const auto reachable_second = hasPotentialToReach(
        object, entry_point.second, prediction_time_horizon_, min_crosswalk_user_velocity_);

      if (!reachable_first && !reachable_second) {
        continue;
      }

      const auto reachable_crosswalk = isReachableEntryPoint(
        object, entry_point, lanelet_map_ptr_, prediction_time_horizon_,
        min_crosswalk_user_velocity_);

      if (!reachable_crosswalk) {
        continue;
      }

      PredictedPath predicted_path =
        path_generator_->generatePathForCrosswalkUser(object, reachable_crosswalk.get());
      predicted_path.confidence = 1.0;

      if (predicted_path.path.empty()) {
        continue;
      }

      predicted_object.kinematics.predicted_paths.push_back(predicted_path);
    }
  }

  const auto n_path = predicted_object.kinematics.predicted_paths.size();
  for (auto & predicted_path : predicted_object.kinematics.predicted_paths) {
    predicted_path.confidence = 1.0 / n_path;
  }

  return predicted_object;
}

void MapBasedPredictionNode::updateObjectData(TrackedObject & object)
{
  if (
    object.kinematics.orientation_availability ==
    autoware_auto_perception_msgs::msg::DetectedObjectKinematics::AVAILABLE) {
    return;
  }

  // Compute yaw angle from the velocity and position of the object
  const auto & object_pose = object.kinematics.pose_with_covariance.pose;
  const auto & object_twist = object.kinematics.twist_with_covariance.twist;
  const auto future_object_pose = tier4_autoware_utils::calcOffsetPose(
    object_pose, object_twist.linear.x * 0.1, object_twist.linear.y * 0.1, 0.0);

  if (object.kinematics.twist_with_covariance.twist.linear.x < 0.0) {
    if (
      object.kinematics.orientation_availability ==
      autoware_auto_perception_msgs::msg::DetectedObjectKinematics::SIGN_UNKNOWN) {
      const auto original_yaw =
        tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
      // flip the angle
      object.kinematics.pose_with_covariance.pose.orientation =
        tier4_autoware_utils::createQuaternionFromYaw(tier4_autoware_utils::pi + original_yaw);
    } else {
      const auto updated_object_yaw =
        tier4_autoware_utils::calcAzimuthAngle(object_pose.position, future_object_pose.position);

      object.kinematics.pose_with_covariance.pose.orientation =
        tier4_autoware_utils::createQuaternionFromYaw(updated_object_yaw);
    }

    object.kinematics.twist_with_covariance.twist.linear.x *= -1.0;
  }

  return;
}

void MapBasedPredictionNode::removeOldObjectsHistory(const double current_time)
{
  std::vector<std::string> invalid_object_id;
  for (auto iter = objects_history_.begin(); iter != objects_history_.end(); ++iter) {
    const std::string object_id = iter->first;
    std::deque<ObjectData> & object_data = iter->second;

    // If object data is empty, we are going to delete the buffer for the obstacle
    if (object_data.empty()) {
      invalid_object_id.push_back(object_id);
      continue;
    }

    const double latest_object_time = rclcpp::Time(object_data.back().header.stamp).seconds();

    // Delete Old Objects
    if (current_time - latest_object_time > 2.0) {
      invalid_object_id.push_back(object_id);
      continue;
    }

    // Delete old information
    while (!object_data.empty()) {
      const double post_object_time = rclcpp::Time(object_data.front().header.stamp).seconds();
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
    objects_history_.erase(key);
  }
}

LaneletsData MapBasedPredictionNode::getCurrentLanelets(const TrackedObject & object)
{
  // obstacle point
  lanelet::BasicPoint2d search_point(
    object.kinematics.pose_with_covariance.pose.position.x,
    object.kinematics.pose_with_covariance.pose.position.y);

  // nearest lanelet
  std::vector<std::pair<double, lanelet::Lanelet>> surrounding_lanelets =
    lanelet::geometry::findNearest(lanelet_map_ptr_->laneletLayer, search_point, 10);

  // No Closest Lanelets
  if (surrounding_lanelets.empty()) {
    return {};
  }

  LaneletsData closest_lanelets;
  for (const auto & lanelet : surrounding_lanelets) {
    // Check if the close lanelets meet the necessary condition for start lanelets and
    // Check if similar lanelet is inside the closest lanelet
    if (
      !checkCloseLaneletCondition(lanelet, object, search_point) ||
      isDuplicated(lanelet, closest_lanelets)) {
      continue;
    }

    LaneletData closest_lanelet;
    closest_lanelet.lanelet = lanelet.second;
    closest_lanelet.probability = calculateLocalLikelihood(lanelet.second, object);
    closest_lanelets.push_back(closest_lanelet);
  }

  return closest_lanelets;
}

bool MapBasedPredictionNode::checkCloseLaneletCondition(
  const std::pair<double, lanelet::Lanelet> & lanelet, const TrackedObject & object,
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

  // If the object is in the objects history, we check if the target lanelet is
  // inside the current lanelets id or following lanelets
  const std::string object_id = toHexString(object.object_id);
  if (objects_history_.count(object_id) != 0) {
    const std::vector<lanelet::ConstLanelet> & possible_lanelet =
      objects_history_.at(object_id).back().future_possible_lanelets;

    bool not_in_possible_lanelet =
      std::find(possible_lanelet.begin(), possible_lanelet.end(), lanelet.second) ==
      possible_lanelet.end();
    if (!possible_lanelet.empty() && not_in_possible_lanelet) {
      return false;
    }
  }

  // Step3. Calculate the angle difference between the lane angle and obstacle angle
  const double object_yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
  const double lane_yaw = lanelet::utils::getLaneletAngle(
    lanelet.second, object.kinematics.pose_with_covariance.pose.position);
  const double delta_yaw = object_yaw - lane_yaw;
  const double normalized_delta_yaw = tier4_autoware_utils::normalizeRadian(delta_yaw);
  const double abs_norm_delta = std::fabs(normalized_delta_yaw);

  // Step4. Check if the closest lanelet is valid, and add all
  // of the lanelets that are below max_dist and max_delta_yaw
  const double object_vel = object.kinematics.twist_with_covariance.twist.linear.x;
  const bool is_yaw_reversed =
    M_PI - delta_yaw_threshold_for_searching_lanelet_ < abs_norm_delta && object_vel < 0.0;
  if (
    lanelet.first < dist_threshold_for_searching_lanelet_ &&
    (is_yaw_reversed || abs_norm_delta < delta_yaw_threshold_for_searching_lanelet_)) {
    return true;
  }

  return false;
}

float MapBasedPredictionNode::calculateLocalLikelihood(
  const lanelet::Lanelet & current_lanelet, const TrackedObject & object) const
{
  const auto & obj_point = object.kinematics.pose_with_covariance.pose.position;

  // compute yaw difference between the object and lane
  const double obj_yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
  const double lane_yaw = lanelet::utils::getLaneletAngle(current_lanelet, obj_point);
  const double delta_yaw = obj_yaw - lane_yaw;
  const double abs_norm_delta_yaw = std::fabs(tier4_autoware_utils::normalizeRadian(delta_yaw));

  // compute lateral distance
  const auto centerline = current_lanelet.centerline();
  std::vector<geometry_msgs::msg::Point> converted_centerline;
  for (const auto & p : centerline) {
    const auto converted_p = lanelet::utils::conversion::toGeomMsgPt(p);
    converted_centerline.push_back(converted_p);
  }
  const double lat_dist =
    std::fabs(motion_utils::calcLateralOffset(converted_centerline, obj_point));

  // Compute Chi-squared distributed (Equation (8) in the paper)
  const double sigma_d = sigma_lateral_offset_;  // Standard Deviation for lateral position
  const double sigma_yaw = M_PI * sigma_yaw_angle_deg_ / 180.0;  // Standard Deviation for yaw angle
  const Eigen::Vector2d delta(lat_dist, abs_norm_delta_yaw);
  const Eigen::Matrix2d P_inv =
    (Eigen::Matrix2d() << 1.0 / (sigma_d * sigma_d), 0.0, 0.0, 1.0 / (sigma_yaw * sigma_yaw))
      .finished();
  const double MINIMUM_DISTANCE = 1e-6;
  const double dist = std::max(delta.dot(P_inv * delta), MINIMUM_DISTANCE);

  return static_cast<float>(1.0 / dist);
}

void MapBasedPredictionNode::updateObjectsHistory(
  const std_msgs::msg::Header & header, const TrackedObject & object,
  const LaneletsData & current_lanelets_data)
{
  std::string object_id = toHexString(object.object_id);
  const auto current_lanelets = getLanelets(current_lanelets_data);

  ObjectData single_object_data;
  single_object_data.header = header;
  single_object_data.current_lanelets = current_lanelets;
  single_object_data.future_possible_lanelets = current_lanelets;
  single_object_data.pose = object.kinematics.pose_with_covariance.pose;
  const double object_yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
  single_object_data.pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(object_yaw);
  single_object_data.time_delay = std::fabs((this->get_clock()->now() - header.stamp).seconds());
  single_object_data.twist = object.kinematics.twist_with_covariance.twist;

  if (objects_history_.count(object_id) == 0) {
    // New Object(Create a new object in object histories)
    std::deque<ObjectData> object_data = {single_object_data};
    objects_history_.emplace(object_id, object_data);
  } else {
    // Object that is already in the object buffer
    std::deque<ObjectData> & object_data = objects_history_.at(object_id);
    object_data.push_back(single_object_data);
  }
}

std::vector<PredictedRefPath> MapBasedPredictionNode::getPredictedReferencePath(
  const TrackedObject & object, const LaneletsData & current_lanelets_data,
  const double object_detected_time)
{
  const double obj_vel = std::fabs(object.kinematics.twist_with_covariance.twist.linear.x);

  std::vector<PredictedRefPath> all_ref_paths;
  for (const auto & current_lanelet_data : current_lanelets_data) {
    // parameter for lanelet::routing::PossiblePathsParams
    const double search_dist = prediction_time_horizon_ * obj_vel +
                               lanelet::utils::getLaneletLength3d(current_lanelet_data.lanelet);
    lanelet::routing::PossiblePathsParams possible_params{search_dist, {}, 0, false, true};

    // Step1. Get the path
    // Step1.1 Get the left lanelet
    lanelet::routing::LaneletPaths left_paths;
    auto opt_left = routing_graph_ptr_->left(current_lanelet_data.lanelet);
    if (!!opt_left) {
      left_paths = routing_graph_ptr_->possiblePaths(*opt_left, possible_params);
    }

    // Step1.2 Get the right lanelet
    lanelet::routing::LaneletPaths right_paths;
    auto opt_right = routing_graph_ptr_->right(current_lanelet_data.lanelet);
    if (!!opt_right) {
      right_paths = routing_graph_ptr_->possiblePaths(*opt_right, possible_params);
    }

    // Step1.3 Get the centerline
    lanelet::routing::LaneletPaths center_paths =
      routing_graph_ptr_->possiblePaths(current_lanelet_data.lanelet, possible_params);

    // Skip calculations if all paths are empty
    if (left_paths.empty() && right_paths.empty() && center_paths.empty()) {
      continue;
    }

    // Step2. Predict Object Maneuver
    const Maneuver predicted_maneuver =
      predictObjectManeuver(object, current_lanelet_data, object_detected_time);

    // Step3. Allocate probability for each predicted maneuver
    const auto maneuver_prob =
      calculateManeuverProbability(predicted_maneuver, left_paths, right_paths, center_paths);

    // Step4. add candidate reference paths to the all_ref_paths
    const float path_prob = current_lanelet_data.probability;
    const auto addReferencePathsLocal = [&](const auto & paths, const auto & maneuver) {
      addReferencePaths(object, paths, path_prob, maneuver_prob, maneuver, all_ref_paths);
    };
    addReferencePathsLocal(left_paths, Maneuver::LEFT_LANE_CHANGE);
    addReferencePathsLocal(right_paths, Maneuver::RIGHT_LANE_CHANGE);
    addReferencePathsLocal(center_paths, Maneuver::LANE_FOLLOW);
  }

  return all_ref_paths;
}

Maneuver MapBasedPredictionNode::predictObjectManeuver(
  const TrackedObject & object, const LaneletData & current_lanelet_data,
  const double object_detected_time)
{
  // Step1. Check if we have the object in the buffer
  const std::string object_id = toHexString(object.object_id);
  if (objects_history_.count(object_id) == 0) {
    return Maneuver::LANE_FOLLOW;
  }

  const std::deque<ObjectData> & object_info = objects_history_.at(object_id);
  const double current_time = (this->get_clock()->now()).seconds();

  // Step2. Get the previous id
  int prev_id = static_cast<int>(object_info.size()) - 1;
  while (prev_id >= 0) {
    const double prev_time_delay = object_info.at(prev_id).time_delay;
    const double prev_time =
      rclcpp::Time(object_info.at(prev_id).header.stamp).seconds() + prev_time_delay;
    // if (object_detected_time - prev_time > history_time_length_) {
    if (current_time - prev_time > history_time_length_) {
      break;
    }
    --prev_id;
  }

  if (prev_id < 0) {
    return Maneuver::LANE_FOLLOW;
  }

  // Step3. Get closest previous lanelet ID
  const auto & prev_info = object_info.at(static_cast<size_t>(prev_id));
  const auto prev_pose = compensateTimeDelay(prev_info.pose, prev_info.twist, prev_info.time_delay);
  const lanelet::ConstLanelets prev_lanelets =
    object_info.at(static_cast<size_t>(prev_id)).current_lanelets;
  if (prev_lanelets.empty()) {
    return Maneuver::LANE_FOLLOW;
  }
  lanelet::ConstLanelet prev_lanelet = prev_lanelets.front();
  double closest_prev_yaw = std::numeric_limits<double>::max();
  for (const auto & lanelet : prev_lanelets) {
    const double lane_yaw = lanelet::utils::getLaneletAngle(lanelet, prev_pose.position);
    const double delta_yaw = tf2::getYaw(prev_pose.orientation) - lane_yaw;
    const double normalized_delta_yaw = tier4_autoware_utils::normalizeRadian(delta_yaw);
    if (normalized_delta_yaw < closest_prev_yaw) {
      closest_prev_yaw = normalized_delta_yaw;
      prev_lanelet = lanelet;
    }
  }

  // Step4. Check if the vehicle has changed lane
  const auto current_lanelet = current_lanelet_data.lanelet;
  const double current_time_delay = std::max(current_time - object_detected_time, 0.0);
  const auto current_pose = compensateTimeDelay(
    object.kinematics.pose_with_covariance.pose, object.kinematics.twist_with_covariance.twist,
    current_time_delay);
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
  const double prev_left_dist = calcLeftLateralOffset(prev_left_bound, prev_pose);
  const double prev_right_dist = calcRightLateralOffset(prev_right_bound, prev_pose);
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

geometry_msgs::msg::Pose MapBasedPredictionNode::compensateTimeDelay(
  const geometry_msgs::msg::Pose & delayed_pose, const geometry_msgs::msg::Twist & twist,
  const double dt) const
{
  if (!enable_delay_compensation_) {
    return delayed_pose;
  }

  /*  == Nonlinear model ==
   *
   * x_{k+1}   = x_k + vx_k * cos(yaw_k) * dt
   * y_{k+1}   = y_k + vx_k * sin(yaw_k) * dt
   * yaw_{k+1} = yaw_k + (wz_k) * dt
   *
   */

  const double vx = twist.linear.x;
  const double wz = twist.angular.z;
  const double prev_yaw = tf2::getYaw(delayed_pose.orientation);
  const double prev_x = delayed_pose.position.x;
  const double prev_y = delayed_pose.position.y;
  const double prev_z = delayed_pose.position.z;

  const double curr_x = prev_x + vx * std::cos(prev_yaw) * dt;
  const double curr_y = prev_y + vx * std::sin(prev_yaw) * dt;
  const double curr_z = prev_z;
  const double curr_yaw = prev_yaw + wz * dt;

  geometry_msgs::msg::Pose current_pose;
  current_pose.position = tier4_autoware_utils::createPoint(curr_x, curr_y, curr_z);
  current_pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(curr_yaw);

  return current_pose;
}

double MapBasedPredictionNode::calcRightLateralOffset(
  const lanelet::ConstLineString2d & boundary_line, const geometry_msgs::msg::Pose & search_pose)
{
  std::vector<geometry_msgs::msg::Point> boundary_path(boundary_line.size());
  for (size_t i = 0; i < boundary_path.size(); ++i) {
    const double x = boundary_line[i].x();
    const double y = boundary_line[i].y();
    boundary_path[i] = tier4_autoware_utils::createPoint(x, y, 0.0);
  }

  return std::fabs(motion_utils::calcLateralOffset(boundary_path, search_pose.position));
}

double MapBasedPredictionNode::calcLeftLateralOffset(
  const lanelet::ConstLineString2d & boundary_line, const geometry_msgs::msg::Pose & search_pose)
{
  return -calcRightLateralOffset(boundary_line, search_pose);
}

void MapBasedPredictionNode::updateFuturePossibleLanelets(
  const TrackedObject & object, const lanelet::routing::LaneletPaths & paths)
{
  std::string object_id = toHexString(object.object_id);
  if (objects_history_.count(object_id) == 0) {
    return;
  }

  std::vector<lanelet::ConstLanelet> & possible_lanelets =
    objects_history_.at(object_id).back().future_possible_lanelets;
  for (const auto & path : paths) {
    for (const auto & lanelet : path) {
      bool not_in_buffer = std::find(possible_lanelets.begin(), possible_lanelets.end(), lanelet) ==
                           possible_lanelets.end();
      if (not_in_buffer) {
        possible_lanelets.push_back(lanelet);
      }
    }
  }
}

void MapBasedPredictionNode::addReferencePaths(
  const TrackedObject & object, const lanelet::routing::LaneletPaths & candidate_paths,
  const float path_probability, const ManeuverProbability & maneuver_probability,
  const Maneuver & maneuver, std::vector<PredictedRefPath> & reference_paths)
{
  if (!candidate_paths.empty()) {
    updateFuturePossibleLanelets(object, candidate_paths);
    const auto converted_paths = convertPathType(candidate_paths);
    for (const auto & converted_path : converted_paths) {
      PredictedRefPath predicted_path;
      predicted_path.probability = maneuver_probability.at(maneuver) * path_probability;
      predicted_path.path = converted_path;
      predicted_path.maneuver = maneuver;
      reference_paths.push_back(predicted_path);
    }
  }
}

ManeuverProbability MapBasedPredictionNode::calculateManeuverProbability(
  const Maneuver & predicted_maneuver, const lanelet::routing::LaneletPaths & left_paths,
  const lanelet::routing::LaneletPaths & right_paths,
  const lanelet::routing::LaneletPaths & center_paths)
{
  float left_lane_change_probability = 0.0;
  float right_lane_change_probability = 0.0;
  float lane_follow_probability = 0.0;
  if (!left_paths.empty() && predicted_maneuver == Maneuver::LEFT_LANE_CHANGE) {
    constexpr float LF_PROB_WHEN_LC = 0.9;  // probability for lane follow during lane change
    constexpr float LC_PROB_WHEN_LC = 1.0;  // probability for left lane change
    left_lane_change_probability = LC_PROB_WHEN_LC;
    right_lane_change_probability = 0.0;
    lane_follow_probability = LF_PROB_WHEN_LC;
  } else if (!right_paths.empty() && predicted_maneuver == Maneuver::RIGHT_LANE_CHANGE) {
    constexpr float LF_PROB_WHEN_LC = 0.9;  // probability for lane follow during lane change
    constexpr float RC_PROB_WHEN_LC = 1.0;  // probability for right lane change
    left_lane_change_probability = 0.0;
    right_lane_change_probability = RC_PROB_WHEN_LC;
    lane_follow_probability = LF_PROB_WHEN_LC;
  } else if (!center_paths.empty()) {
    constexpr float LF_PROB = 1.0;  // probability for lane follow
    constexpr float LC_PROB = 0.3;  // probability for left lane change
    constexpr float RC_PROB = 0.3;  // probability for right lane change
    if (predicted_maneuver == Maneuver::LEFT_LANE_CHANGE) {
      // If prediction says left change, but left lane is empty, assume lane follow
      left_lane_change_probability = 0.0;
      right_lane_change_probability = (!right_paths.empty()) ? RC_PROB : 0.0;
    } else if (predicted_maneuver == Maneuver::RIGHT_LANE_CHANGE) {
      // If prediction says right change, but right lane is empty, assume lane follow
      left_lane_change_probability = (!left_paths.empty()) ? LC_PROB : 0.0;
      right_lane_change_probability = 0.0;
    } else {
      // Predicted Maneuver is Lane Follow
      left_lane_change_probability = LC_PROB;
      right_lane_change_probability = RC_PROB;
    }
    lane_follow_probability = LF_PROB;
  } else {
    // Center path is empty
    constexpr float LC_PROB = 1.0;  // probability for left lane change
    constexpr float RC_PROB = 1.0;  // probability for right lane change
    lane_follow_probability = 0.0;
    if (!left_paths.empty() && right_paths.empty()) {
      left_lane_change_probability = LC_PROB;
      right_lane_change_probability = 0.0;
    } else if (left_paths.empty() && !right_paths.empty()) {
      left_lane_change_probability = 0.0;
      right_lane_change_probability = RC_PROB;
    } else {
      left_lane_change_probability = LC_PROB;
      right_lane_change_probability = RC_PROB;
    }
  }

  const float MIN_PROBABILITY = 1e-3;
  const float max_prob = std::max(
    MIN_PROBABILITY, std::max(
                       lane_follow_probability,
                       std::max(left_lane_change_probability, right_lane_change_probability)));

  // Insert Normalized Probability
  ManeuverProbability maneuver_prob;
  maneuver_prob[Maneuver::LEFT_LANE_CHANGE] = left_lane_change_probability / max_prob;
  maneuver_prob[Maneuver::RIGHT_LANE_CHANGE] = right_lane_change_probability / max_prob;
  maneuver_prob[Maneuver::LANE_FOLLOW] = lane_follow_probability / max_prob;

  return maneuver_prob;
}

std::vector<PosePath> MapBasedPredictionNode::convertPathType(
  const lanelet::routing::LaneletPaths & paths)
{
  std::vector<PosePath> converted_paths;
  for (const auto & path : paths) {
    PosePath converted_path;

    // Insert Positions. Note that we start inserting points from previous lanelet
    if (!path.empty()) {
      lanelet::ConstLanelets prev_lanelets = routing_graph_ptr_->previous(path.front());
      if (!prev_lanelets.empty()) {
        lanelet::ConstLanelet prev_lanelet = prev_lanelets.front();
        for (const auto & lanelet_p : prev_lanelet.centerline()) {
          geometry_msgs::msg::Pose current_p;
          current_p.position =
            tier4_autoware_utils::createPoint(lanelet_p.x(), lanelet_p.y(), lanelet_p.z());
          const double lane_yaw = lanelet::utils::getLaneletAngle(prev_lanelet, current_p.position);
          current_p.orientation = tier4_autoware_utils::createQuaternionFromYaw(lane_yaw);
          converted_path.push_back(current_p);
        }
      }
    }

    for (const auto & lanelet : path) {
      for (const auto & lanelet_p : lanelet.centerline()) {
        geometry_msgs::msg::Pose current_p;
        current_p.position =
          tier4_autoware_utils::createPoint(lanelet_p.x(), lanelet_p.y(), lanelet_p.z());
        const double lane_yaw = lanelet::utils::getLaneletAngle(lanelet, current_p.position);
        current_p.orientation = tier4_autoware_utils::createQuaternionFromYaw(lane_yaw);

        // Prevent from inserting same points
        if (!converted_path.empty()) {
          const auto prev_p = converted_path.back();
          const double tmp_dist = tier4_autoware_utils::calcDistance2d(prev_p, current_p);
          if (tmp_dist < 1e-6) {
            continue;
          }
        }

        converted_path.push_back(current_p);
      }
    }

    // Resample Path
    const auto resampled_converted_path = resamplePath(converted_path);
    converted_paths.push_back(resampled_converted_path);
  }

  return converted_paths;
}

PosePath MapBasedPredictionNode::resamplePath(const PosePath & base_path) const
{
  std::vector<double> base_s(base_path.size());
  std::vector<double> base_x(base_path.size());
  std::vector<double> base_y(base_path.size());
  std::vector<double> base_z(base_path.size());
  for (size_t i = 0; i < base_path.size(); ++i) {
    base_x.at(i) = base_path.at(i).position.x;
    base_y.at(i) = base_path.at(i).position.y;
    base_z.at(i) = base_path.at(i).position.z;
    base_s.at(i) = motion_utils::calcSignedArcLength(base_path, 0, i);
  }
  const double base_path_len = base_s.back();

  std::vector<double> resampled_s;
  for (double s = 0.0; s <= base_path_len; s += reference_path_resolution_) {
    resampled_s.push_back(s);
  }

  if (resampled_s.empty()) {
    return base_path;
  }

  // Insert End Point
  const double epsilon = 0.01;
  if (std::fabs(resampled_s.back() - base_path_len) < epsilon) {
    resampled_s.back() = base_path_len;
  } else {
    resampled_s.push_back(base_path_len);
  }

  if (resampled_s.size() < 2) {
    return base_path;
  }

  const auto resampled_x = interpolation::lerp(base_s, base_x, resampled_s);
  const auto resampled_y = interpolation::lerp(base_s, base_y, resampled_s);
  const auto resampled_z = interpolation::lerp(base_s, base_z, resampled_s);

  PosePath resampled_path(resampled_x.size());
  // Position
  for (size_t i = 0; i < resampled_path.size(); ++i) {
    resampled_path.at(i).position =
      tier4_autoware_utils::createPoint(resampled_x.at(i), resampled_y.at(i), resampled_z.at(i));
  }
  // Orientation
  for (size_t i = 0; i < resampled_path.size() - 1; ++i) {
    const auto curr_p = resampled_path.at(i).position;
    const auto next_p = resampled_path.at(i + 1).position;
    const double yaw = std::atan2(next_p.y - curr_p.y, next_p.x - curr_p.x);
    resampled_path.at(i).orientation = tier4_autoware_utils::createQuaternionFromYaw(yaw);
  }
  resampled_path.back().orientation = resampled_path.at(resampled_path.size() - 2).orientation;

  return resampled_path;
}

bool MapBasedPredictionNode::isDuplicated(
  const std::pair<double, lanelet::ConstLanelet> & target_lanelet,
  const LaneletsData & lanelets_data)
{
  const double CLOSE_LANELET_THRESHOLD = 0.1;
  for (const auto & lanelet_data : lanelets_data) {
    const auto target_lanelet_end_p = target_lanelet.second.centerline2d().back();
    const auto lanelet_end_p = lanelet_data.lanelet.centerline2d().back();
    const double dist = std::hypot(
      target_lanelet_end_p.x() - lanelet_end_p.x(), target_lanelet_end_p.y() - lanelet_end_p.y());
    if (dist < CLOSE_LANELET_THRESHOLD) {
      return true;
    }
  }

  return false;
}

bool MapBasedPredictionNode::isDuplicated(
  const PredictedPath & predicted_path, const std::vector<PredictedPath> & predicted_paths)
{
  const double CLOSE_PATH_THRESHOLD = 0.1;
  for (const auto & prev_predicted_path : predicted_paths) {
    const auto prev_path_end = prev_predicted_path.path.back().position;
    const auto current_path_end = predicted_path.path.back().position;
    const double dist = tier4_autoware_utils::calcDistance2d(prev_path_end, current_path_end);
    if (dist < CLOSE_PATH_THRESHOLD) {
      return true;
    }
  }

  return false;
}
}  // namespace map_based_prediction

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(map_based_prediction::MapBasedPredictionNode)
