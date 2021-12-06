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

#ifndef MAP_BASED_PREDICTION_ROS_HPP_
#define MAP_BASED_PREDICTION_ROS_HPP_

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_objects.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>
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

namespace tf2_ros
{
class Buffer;
class TransformListener;
}  // namespace tf2_ros

namespace lanelet
{
class Lanelet;
class LaneletMap;
using LaneletMapPtr = std::shared_ptr<LaneletMap>;
namespace routing
{
class RoutingGraph;
}  // namespace routing
namespace traffic_rules
{
class TrafficRules;
}  // namespace traffic_rules
}  // namespace lanelet

struct ObjectData
{
  lanelet::ConstLanelets current_lanelets;
  lanelet::ConstLanelets future_possible_lanelets;
  geometry_msgs::msg::PoseStamped pose;
};

enum class Maneuver {
  LANE_FOLLOW,
  LEFT_LANE_CHANGE,
  RIGHT_LANE_CHANGE,
};

class MapBasedPrediction;

class MapBasedPredictionROS : public rclcpp::Node
{
private:
  double prediction_time_horizon_;
  double prediction_sampling_delta_time_;
  double min_velocity_for_map_based_prediction_;
  double interpolating_resolution_;
  double debug_accumulated_time_;
  double dist_threshold_for_searching_lanelet_;
  double delta_yaw_threshold_for_searching_lanelet_;
  double sigma_lateral_offset_;
  double sigma_yaw_angle_;
  double history_time_length_;
  double dist_ratio_threshold_to_left_bound_;
  double dist_ratio_threshold_to_right_bound_;
  double diff_dist_threshold_to_left_bound_;
  double diff_dist_threshold_to_right_bound_;

  rclcpp::Subscription<autoware_auto_perception_msgs::msg::TrackedObjects>::SharedPtr sub_objects_;
  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr sub_map_;
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr pub_objects_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

  std::unordered_map<std::string, std::deque<ObjectData>> object_buffer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_ptr_;

  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
  std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr_;
  std::shared_ptr<lanelet::traffic_rules::TrafficRules> traffic_rules_ptr_;
  std::shared_ptr<MapBasedPrediction> map_based_prediction_;

  bool getSelfPose(geometry_msgs::msg::Pose & self_pose, const std_msgs::msg::Header & header);
  bool getSelfPoseInMap(geometry_msgs::msg::Pose & self_pose);

  double getObjectYaw(const autoware_auto_perception_msgs::msg::TrackedObject & object);
  double calculateLikelihood(
    const std::vector<geometry_msgs::msg::Pose> & path,
    const autoware_auto_perception_msgs::msg::TrackedObject & object);

  void addValidPath(
    const lanelet::routing::LaneletPaths & candidate_paths,
    lanelet::routing::LaneletPaths & valid_paths);

  void objectsCallback(
    const autoware_auto_perception_msgs::msg::TrackedObjects::ConstSharedPtr in_objects);
  void mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg);

  bool getClosestLanelets(
    const autoware_auto_perception_msgs::msg::TrackedObject & object,
    const lanelet::LaneletMapPtr & lanelet_map_ptr, lanelet::ConstLanelets & closest_lanelets);

  bool checkCloseLaneletCondition(
    const std::pair<double, lanelet::Lanelet> & lanelet,
    const autoware_auto_perception_msgs::msg::TrackedObject & object,
    const lanelet::BasicPoint2d & search_point);

  void removeInvalidObject(const double current_time);
  bool updateObjectBuffer(
    const std_msgs::msg::Header & header,
    const autoware_auto_perception_msgs::msg::TrackedObject & object,
    lanelet::ConstLanelets & current_lanelets);
  void updatePossibleLanelets(
    const std::string object_id, const lanelet::routing::LaneletPaths & paths);

  double calcRightLateralOffset(
    const lanelet::ConstLineString2d & bound_line, const geometry_msgs::msg::Pose & search_pose);
  double calcLeftLateralOffset(
    const lanelet::ConstLineString2d & bound_line, const geometry_msgs::msg::Pose & search_pose);
  Maneuver detectLaneChange(
    const autoware_auto_perception_msgs::msg::TrackedObject & object,
    const lanelet::ConstLanelet & current_lanelet, const double current_time);

public:
  explicit MapBasedPredictionROS(const rclcpp::NodeOptions & node_options);
};

#endif  // MAP_BASED_PREDICTION_ROS_HPP_
