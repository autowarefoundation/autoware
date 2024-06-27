// Copyright 2023 TIER IV, Inc.
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

#define EIGEN_MPL2_ONLY

#include "radar_object_tracker_node.hpp"

#include "autoware_radar_object_tracker/utils/radar_object_tracker_utils.hpp"
#include "autoware_radar_object_tracker/utils/utils.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <boost/optional.hpp>

#include <glog/logging.h>
#include <tf2_ros/create_timer_interface.h>
#include <tf2_ros/create_timer_ros.h>

#include <iterator>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::radar_object_tracker
{
using Label = autoware_perception_msgs::msg::ObjectClassification;

RadarObjectTrackerNode::RadarObjectTrackerNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("radar_object_tracker", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  // glog for debug
  if (!google::IsGoogleLoggingInitialized()) {
    google::InitGoogleLogging("radar_object_tracker");
    google::InstallFailureSignalHandler();
  }

  // Create publishers and subscribers
  detected_object_sub_ = create_subscription<autoware_perception_msgs::msg::DetectedObjects>(
    "input", rclcpp::QoS{1},
    std::bind(&RadarObjectTrackerNode::onMeasurement, this, std::placeholders::_1));
  tracked_objects_pub_ =
    create_publisher<autoware_perception_msgs::msg::TrackedObjects>("output", rclcpp::QoS{1});
  sub_map_ = this->create_subscription<LaneletMapBin>(
    "/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&RadarObjectTrackerNode::onMap, this, std::placeholders::_1));

  // Parameters
  tracker_lifetime_ = declare_parameter<double>("tracker_lifetime");
  double publish_rate = declare_parameter<double>("publish_rate");
  measurement_count_threshold_ = declare_parameter<int>("measurement_count_threshold");
  world_frame_id_ = declare_parameter<std::string>("world_frame_id");
  bool enable_delay_compensation{declare_parameter<bool>("enable_delay_compensation")};
  tracker_config_directory_ = declare_parameter<std::string>("tracking_config_directory");
  logging_.enable = declare_parameter<bool>("enable_logging");
  logging_.path = declare_parameter<std::string>("logging_file_path");

  // noise filter
  use_distance_based_noise_filtering_ =
    declare_parameter<bool>("use_distance_based_noise_filtering");
  use_map_based_noise_filtering_ = declare_parameter<bool>("use_map_based_noise_filtering");
  minimum_range_threshold_ = declare_parameter<double>("minimum_range_threshold");
  max_distance_from_lane_ = declare_parameter<double>("max_distance_from_lane");
  max_angle_diff_from_lane_ = declare_parameter<double>("max_angle_diff_from_lane");
  max_lateral_velocity_ = declare_parameter<double>("max_lateral_velocity");

  // Load tracking config file
  if (tracker_config_directory_.empty()) {
    tracker_config_directory_ =
      ament_index_cpp::get_package_share_directory("radar_object_tracker") + "/config/tracking/";
  }

  auto cti = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf_buffer_.setCreateTimerInterface(cti);

  // Create ROS time based timer
  if (enable_delay_compensation) {
    const auto period_ns = rclcpp::Rate(publish_rate).period();
    publish_timer_ = rclcpp::create_timer(
      this, get_clock(), period_ns, std::bind(&RadarObjectTrackerNode::onTimer, this));
  }

  const auto tmp = this->declare_parameter<std::vector<int64_t>>("can_assign_matrix");
  const std::vector<int> can_assign_matrix(tmp.begin(), tmp.end());

  // import association matrices
  const auto max_dist_matrix = this->declare_parameter<std::vector<double>>("max_dist_matrix");
  const auto max_area_matrix = this->declare_parameter<std::vector<double>>("max_area_matrix");
  const auto min_area_matrix = this->declare_parameter<std::vector<double>>("min_area_matrix");
  const auto max_rad_matrix = this->declare_parameter<std::vector<double>>("max_rad_matrix");
  const auto min_iou_matrix = this->declare_parameter<std::vector<double>>("min_iou_matrix");
  data_association_ = std::make_unique<DataAssociation>(
    can_assign_matrix, max_dist_matrix, max_area_matrix, min_area_matrix, max_rad_matrix,
    min_iou_matrix);

  // tracker map
  tracker_map_.insert(
    std::make_pair(Label::CAR, this->declare_parameter<std::string>("car_tracker")));
  tracker_map_.insert(
    std::make_pair(Label::TRUCK, this->declare_parameter<std::string>("truck_tracker")));
  tracker_map_.insert(
    std::make_pair(Label::BUS, this->declare_parameter<std::string>("bus_tracker")));
  tracker_map_.insert(
    std::make_pair(Label::TRAILER, this->declare_parameter<std::string>("trailer_tracker")));
  tracker_map_.insert(
    std::make_pair(Label::PEDESTRIAN, this->declare_parameter<std::string>("pedestrian_tracker")));
  tracker_map_.insert(
    std::make_pair(Label::BICYCLE, this->declare_parameter<std::string>("bicycle_tracker")));
  tracker_map_.insert(
    std::make_pair(Label::MOTORCYCLE, this->declare_parameter<std::string>("motorcycle_tracker")));
}

// load map information to node parameter
void RadarObjectTrackerNode::onMap(const LaneletMapBin::ConstSharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "[Radar Object Tracker]: Start loading lanelet");
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
  RCLCPP_INFO(get_logger(), "[Radar Object Tracker]: Map is loaded");
  map_is_loaded_ = true;
}

void RadarObjectTrackerNode::onMeasurement(
  const autoware_perception_msgs::msg::DetectedObjects::ConstSharedPtr input_objects_msg)
{
  const auto self_transform = autoware::radar_object_tracker::utils::getTransformAnonymous(
    tf_buffer_, "base_link", world_frame_id_, input_objects_msg->header.stamp);
  if (!self_transform) {
    return;
  }

  /* transform to world coordinate */
  autoware_perception_msgs::msg::DetectedObjects transformed_objects;
  if (!object_recognition_utils::transformObjects(
        *input_objects_msg, world_frame_id_, tf_buffer_, transformed_objects)) {
    return;
  }
  /* tracker prediction */
  rclcpp::Time measurement_time = input_objects_msg->header.stamp;
  for (auto itr = list_tracker_.begin(); itr != list_tracker_.end(); ++itr) {
    (*itr)->predict(measurement_time);
  }

  /* global nearest neighbor */
  std::unordered_map<int, int> direct_assignment, reverse_assignment;
  Eigen::MatrixXd score_matrix = data_association_->calcScoreMatrix(
    transformed_objects, list_tracker_,  // row : tracker, col : measurement
    logging_.enable, logging_.path);
  data_association_->assign(score_matrix, direct_assignment, reverse_assignment);

  /* tracker measurement update */
  int tracker_idx = 0;
  for (auto tracker_itr = list_tracker_.begin(); tracker_itr != list_tracker_.end();
       ++tracker_itr, ++tracker_idx) {
    if (direct_assignment.find(tracker_idx) != direct_assignment.end()) {  // found
      (*(tracker_itr))
        ->updateWithMeasurement(
          transformed_objects.objects.at(direct_assignment.find(tracker_idx)->second),
          measurement_time, *self_transform);
    } else {  // not found
      (*(tracker_itr))->updateWithoutMeasurement();
    }
  }

  /* life cycle check */
  checkTrackerLifeCycle(list_tracker_, measurement_time, *self_transform);
  /* map based noise filter */
  if (map_is_loaded_ && use_map_based_noise_filtering_) {
    mapBasedNoiseFilter(list_tracker_, measurement_time);
  }
  /* distance based noise filter */
  if (use_distance_based_noise_filtering_) {
    distanceBasedNoiseFilter(list_tracker_, measurement_time, *self_transform);
  }
  /* sanitize trackers */
  sanitizeTracker(list_tracker_, measurement_time);

  /* new tracker */
  for (size_t i = 0; i < transformed_objects.objects.size(); ++i) {
    if (reverse_assignment.find(i) != reverse_assignment.end()) {  // found
      continue;
    }
    std::shared_ptr<Tracker> tracker =
      createNewTracker(transformed_objects.objects.at(i), measurement_time, *self_transform);
    if (tracker) {
      list_tracker_.push_back(tracker);
    }
  }

  if (publish_timer_ == nullptr) {
    publish(measurement_time);
  }
}

std::shared_ptr<Tracker> RadarObjectTrackerNode::createNewTracker(
  const autoware_perception_msgs::msg::DetectedObject & object, const rclcpp::Time & time,
  const geometry_msgs::msg::Transform & /*self_transform*/) const
{
  const std::uint8_t label = object_recognition_utils::getHighestProbLabel(object.classification);
  if (tracker_map_.count(label) != 0) {
    const auto tracker = tracker_map_.at(label);

    if (tracker == "linear_motion_tracker") {
      std::string config_file = tracker_config_directory_ + "linear_motion_tracker.yaml";
      return std::make_shared<LinearMotionTracker>(time, object, config_file, label);
    } else if (tracker == "constant_turn_rate_motion_tracker") {
      std::string config_file =
        tracker_config_directory_ + "constant_turn_rate_motion_tracker.yaml";
      return std::make_shared<ConstantTurnRateMotionTracker>(time, object, config_file, label);
    } else {
      // not implemented yet so put warning
      RCLCPP_WARN(get_logger(), "Tracker %s is not implemented yet", tracker.c_str());
    }
  }
  std::string config_file = tracker_config_directory_ + "linear_motion_tracker.yaml";
  return std::make_shared<LinearMotionTracker>(time, object, config_file, label);
}

void RadarObjectTrackerNode::onTimer()
{
  rclcpp::Time current_time = this->now();
  const auto self_transform = autoware::radar_object_tracker::utils::getTransformAnonymous(
    tf_buffer_, world_frame_id_, "base_link", current_time);
  if (!self_transform) {
    return;
  }

  /* life cycle check */
  checkTrackerLifeCycle(list_tracker_, current_time, *self_transform);

  /* map based noise filter */
  if (map_is_loaded_ && use_map_based_noise_filtering_) {
    mapBasedNoiseFilter(list_tracker_, current_time);
  }
  /* distance based noise filter */
  if (use_distance_based_noise_filtering_) {
    distanceBasedNoiseFilter(list_tracker_, current_time, *self_transform);
  }

  /* sanitize trackers */
  sanitizeTracker(list_tracker_, current_time);

  // Publish
  publish(current_time);
}

void RadarObjectTrackerNode::checkTrackerLifeCycle(
  std::list<std::shared_ptr<Tracker>> & list_tracker, const rclcpp::Time & time,
  [[maybe_unused]] const geometry_msgs::msg::Transform & self_transform)
{
  /* delete tracker */
  for (auto itr = list_tracker.begin(); itr != list_tracker.end(); ++itr) {
    const bool is_old = tracker_lifetime_ < (*itr)->getElapsedTimeFromLastUpdate(time);
    if (is_old) {
      auto erase_itr = itr;
      --itr;
      list_tracker.erase(erase_itr);
    }
  }
}

// remove objects by lanelet information
void RadarObjectTrackerNode::mapBasedNoiseFilter(
  std::list<std::shared_ptr<Tracker>> & list_tracker, const rclcpp::Time & time)
{
  for (auto itr = list_tracker.begin(); itr != list_tracker.end(); ++itr) {
    autoware_perception_msgs::msg::TrackedObject object;
    (*itr)->getTrackedObject(time, object);
    const auto closest_lanelets = autoware::radar_object_tracker::utils::getClosestValidLanelets(
      object, lanelet_map_ptr_, max_distance_from_lane_, max_angle_diff_from_lane_);

    // 1. If the object is not close to any lanelet, delete the tracker
    const bool no_closest_lanelet = closest_lanelets.empty();
    // 2. If the object velocity direction is not close to the lanelet direction, delete the tracker
    const bool is_velocity_direction_close_to_lanelet =
      autoware::radar_object_tracker::utils::hasValidVelocityDirectionToLanelet(
        object, closest_lanelets, max_lateral_velocity_);
    if (no_closest_lanelet || !is_velocity_direction_close_to_lanelet) {
      // std::cout << "object removed due to map based noise filter" << " no close lanelet: " <<
      // no_closest_lanelet << " velocity direction flag:" << is_velocity_direction_close_to_lanelet
      // << std::endl;
      itr = list_tracker.erase(itr);
      --itr;
    }
  }
}

// remove objects by distance
void RadarObjectTrackerNode::distanceBasedNoiseFilter(
  std::list<std::shared_ptr<Tracker>> & list_tracker, const rclcpp::Time & time,
  const geometry_msgs::msg::Transform & self_transform)
{
  // remove objects that are too close
  for (auto itr = list_tracker.begin(); itr != list_tracker.end(); ++itr) {
    autoware_perception_msgs::msg::TrackedObject object;
    (*itr)->getTrackedObject(time, object);
    const double distance = std::hypot(
      object.kinematics.pose_with_covariance.pose.position.x - self_transform.translation.x,
      object.kinematics.pose_with_covariance.pose.position.y - self_transform.translation.y);
    if (distance < minimum_range_threshold_) {
      // std::cout << "object removed due to small distance. distance: " << distance << std::endl;
      itr = list_tracker.erase(itr);
      --itr;
    }
  }
}

void RadarObjectTrackerNode::sanitizeTracker(
  std::list<std::shared_ptr<Tracker>> & list_tracker, const rclcpp::Time & time)
{
  constexpr float min_iou = 0.1;
  constexpr float min_iou_for_unknown_object = 0.001;
  constexpr double distance_threshold = 5.0;
  /* delete collision tracker */
  for (auto itr1 = list_tracker.begin(); itr1 != list_tracker.end(); ++itr1) {
    autoware_perception_msgs::msg::TrackedObject object1;
    (*itr1)->getTrackedObject(time, object1);
    for (auto itr2 = std::next(itr1); itr2 != list_tracker.end(); ++itr2) {
      autoware_perception_msgs::msg::TrackedObject object2;
      (*itr2)->getTrackedObject(time, object2);
      const double distance = std::hypot(
        object1.kinematics.pose_with_covariance.pose.position.x -
          object2.kinematics.pose_with_covariance.pose.position.x,
        object1.kinematics.pose_with_covariance.pose.position.y -
          object2.kinematics.pose_with_covariance.pose.position.y);
      if (distance_threshold < distance) {
        continue;
      }

      const double min_union_iou_area = 1e-2;
      const auto iou = object_recognition_utils::get2dIoU(object1, object2, min_union_iou_area);
      const auto & label1 = (*itr1)->getHighestProbLabel();
      const auto & label2 = (*itr2)->getHighestProbLabel();
      bool should_delete_tracker1 = false;
      bool should_delete_tracker2 = false;

      // If at least one of them is UNKNOWN, delete the one with lower IOU. Because the UNKNOWN
      // objects are not reliable.
      if (label1 == Label::UNKNOWN || label2 == Label::UNKNOWN) {
        if (min_iou_for_unknown_object < iou) {
          if (label1 == Label::UNKNOWN && label2 == Label::UNKNOWN) {
            if ((*itr1)->getTotalMeasurementCount() < (*itr2)->getTotalMeasurementCount()) {
              should_delete_tracker1 = true;
            } else {
              should_delete_tracker2 = true;
            }
          } else if (label1 == Label::UNKNOWN) {
            should_delete_tracker1 = true;
          } else if (label2 == Label::UNKNOWN) {
            should_delete_tracker2 = true;
          }
        }
      } else {  // If neither is UNKNOWN, delete the one with lower IOU.
        if (min_iou < iou) {
          if ((*itr1)->getTotalMeasurementCount() < (*itr2)->getTotalMeasurementCount()) {
            should_delete_tracker1 = true;
          } else {
            should_delete_tracker2 = true;
          }
        }
      }

      if (should_delete_tracker1) {
        itr1 = list_tracker.erase(itr1);
        --itr1;
        break;
      } else if (should_delete_tracker2) {
        itr2 = list_tracker.erase(itr2);
        --itr2;
      }
    }
  }
}

inline bool RadarObjectTrackerNode::shouldTrackerPublish(
  const std::shared_ptr<const Tracker> tracker) const
{
  if (tracker->getTotalMeasurementCount() < measurement_count_threshold_) {
    return false;
  }
  return true;
}

void RadarObjectTrackerNode::publish(const rclcpp::Time & time) const
{
  const auto subscriber_count = tracked_objects_pub_->get_subscription_count() +
                                tracked_objects_pub_->get_intra_process_subscription_count();
  if (subscriber_count < 1) {
    return;
  }
  // Create output msg
  autoware_perception_msgs::msg::TrackedObjects output_msg;
  output_msg.header.frame_id = world_frame_id_;
  output_msg.header.stamp = time;
  for (auto itr = list_tracker_.begin(); itr != list_tracker_.end(); ++itr) {
    if (!shouldTrackerPublish(*itr)) {
      continue;
    }
    autoware_perception_msgs::msg::TrackedObject object;
    (*itr)->getTrackedObject(time, object);
    output_msg.objects.push_back(object);
  }

  // Publish
  tracked_objects_pub_->publish(output_msg);
}
}  // namespace autoware::radar_object_tracker

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::radar_object_tracker::RadarObjectTrackerNode)
