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

#ifndef MAP_BASED_PREDICTION__MAP_BASED_PREDICTION_NODE_HPP_
#define MAP_BASED_PREDICTION__MAP_BASED_PREDICTION_NODE_HPP_

#include "map_based_prediction/path_generator.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <motion_utils/motion_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/ros/transform_listener.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_objects.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace map_based_prediction
{
struct ObjectData
{
  std_msgs::msg::Header header;
  lanelet::ConstLanelets current_lanelets;
  lanelet::ConstLanelets future_possible_lanelets;
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Twist twist;
  double time_delay;
};

enum class Maneuver {
  LANE_FOLLOW = 0,
  LEFT_LANE_CHANGE = 1,
  RIGHT_LANE_CHANGE = 2,
};

struct LaneletData
{
  lanelet::Lanelet lanelet;
  float probability;
};

struct PredictedRefPath
{
  float probability;
  PosePath path;
  Maneuver maneuver;
};

using LaneletsData = std::vector<LaneletData>;
using ManeuverProbability = std::unordered_map<Maneuver, float>;
using autoware_auto_mapping_msgs::msg::HADMapBin;
using autoware_auto_perception_msgs::msg::ObjectClassification;
using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedObjectKinematics;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_perception_msgs::msg::PredictedPath;
using autoware_auto_perception_msgs::msg::TrackedObject;
using autoware_auto_perception_msgs::msg::TrackedObjectKinematics;
using autoware_auto_perception_msgs::msg::TrackedObjects;
using tier4_autoware_utils::StopWatch;

class MapBasedPredictionNode : public rclcpp::Node
{
public:
  explicit MapBasedPredictionNode(const rclcpp::NodeOptions & node_options);

private:
  // ROS Publisher and Subscriber
  rclcpp::Publisher<PredictedObjects>::SharedPtr pub_objects_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_markers_;
  rclcpp::Subscription<TrackedObjects>::SharedPtr sub_objects_;
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_map_;

  // Object History
  std::unordered_map<std::string, std::deque<ObjectData>> objects_history_;

  // Lanelet Map Pointers
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
  std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr_;
  std::shared_ptr<lanelet::traffic_rules::TrafficRules> traffic_rules_ptr_;

  // Pose Transform Listener
  tier4_autoware_utils::TransformListener transform_listener_{this};

  // Path Generator
  std::shared_ptr<PathGenerator> path_generator_;

  // Crosswalk Entry Points
  lanelet::ConstLanelets crosswalks_;

  // Parameters
  bool enable_delay_compensation_;
  double prediction_time_horizon_;
  double prediction_sampling_time_interval_;
  double min_velocity_for_map_based_prediction_;
  double min_crosswalk_user_velocity_;
  double debug_accumulated_time_;
  double dist_threshold_for_searching_lanelet_;
  double delta_yaw_threshold_for_searching_lanelet_;
  double sigma_lateral_offset_;
  double sigma_yaw_angle_deg_;
  double object_buffer_time_length_;
  double history_time_length_;
  double dist_ratio_threshold_to_left_bound_;
  double dist_ratio_threshold_to_right_bound_;
  double diff_dist_threshold_to_left_bound_;
  double diff_dist_threshold_to_right_bound_;
  double reference_path_resolution_;

  // Stop watch
  StopWatch<std::chrono::milliseconds> stop_watch_;

  // Member Functions
  void mapCallback(const HADMapBin::ConstSharedPtr msg);
  void objectsCallback(const TrackedObjects::ConstSharedPtr in_objects);

  PredictedObjectKinematics convertToPredictedKinematics(
    const TrackedObjectKinematics & tracked_object);

  PredictedObject convertToPredictedObject(const TrackedObject & tracked_object);

  PredictedObject getPredictedObjectAsCrosswalkUser(const TrackedObject & object);

  void removeOldObjectsHistory(const double current_time);

  LaneletsData getCurrentLanelets(const TrackedObject & object);
  bool checkCloseLaneletCondition(
    const std::pair<double, lanelet::Lanelet> & lanelet, const TrackedObject & object,
    const lanelet::BasicPoint2d & search_point);
  float calculateLocalLikelihood(
    const lanelet::Lanelet & current_lanelet, const TrackedObject & object) const;
  void updateObjectData(TrackedObject & object);

  void updateObjectsHistory(
    const std_msgs::msg::Header & header, const TrackedObject & object,
    const LaneletsData & current_lanelets_data);

  std::vector<PredictedRefPath> getPredictedReferencePath(
    const TrackedObject & object, const LaneletsData & current_lanelets_data,
    const double object_detected_time);
  Maneuver predictObjectManeuver(
    const TrackedObject & object, const LaneletData & current_lanelet,
    const double object_detected_time);
  geometry_msgs::msg::Pose compensateTimeDelay(
    const geometry_msgs::msg::Pose & delayed_pose, const geometry_msgs::msg::Twist & twist,
    const double dt) const;
  double calcRightLateralOffset(
    const lanelet::ConstLineString2d & boundary_line, const geometry_msgs::msg::Pose & search_pose);
  double calcLeftLateralOffset(
    const lanelet::ConstLineString2d & boundary_line, const geometry_msgs::msg::Pose & search_pose);
  ManeuverProbability calculateManeuverProbability(
    const Maneuver & predicted_maneuver, const lanelet::routing::LaneletPaths & left_paths,
    const lanelet::routing::LaneletPaths & right_paths,
    const lanelet::routing::LaneletPaths & center_paths);

  void addReferencePaths(
    const TrackedObject & object, const lanelet::routing::LaneletPaths & candidate_paths,
    const float path_probability, const ManeuverProbability & maneuver_probability,
    const Maneuver & maneuver, std::vector<PredictedRefPath> & reference_paths);
  std::vector<PosePath> convertPathType(const lanelet::routing::LaneletPaths & paths);
  PosePath resamplePath(const PosePath & base_path) const;

  void updateFuturePossibleLanelets(
    const TrackedObject & object, const lanelet::routing::LaneletPaths & paths);

  bool isDuplicated(
    const std::pair<double, lanelet::ConstLanelet> & target_lanelet,
    const LaneletsData & lanelets_data);
  bool isDuplicated(
    const PredictedPath & predicted_path, const std::vector<PredictedPath> & predicted_paths);

  visualization_msgs::msg::Marker getDebugMarker(
    const TrackedObject & object, const Maneuver & maneuver, const size_t obj_num);
};
}  // namespace map_based_prediction

#endif  // MAP_BASED_PREDICTION__MAP_BASED_PREDICTION_NODE_HPP_
