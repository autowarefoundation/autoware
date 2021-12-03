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
#ifndef OBSTACLE_AVOIDANCE_PLANNER__NODE_HPP_
#define OBSTACLE_AVOIDANCE_PLANNER__NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_point.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <autoware_planning_msgs/msg/enable_avoidance.hpp>
#include <autoware_planning_msgs/msg/is_avoidance_possible.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/optional/optional_fwd.hpp>

#include <memory>
#include <mutex>
#include <vector>

namespace ros
{
class Time;
}

namespace cv
{
class Mat;
}

namespace tf2_ros
{
class Buffer;
class TransformListener;
}  // namespace tf2_ros

class EBPathOptimizer;

struct QPParam;
struct TrajectoryParam;
struct ConstrainParam;
struct VehicleParam;
struct MPTParam;
struct DebugData;
struct Trajectories;

class ObstacleAvoidancePlanner : public rclcpp::Node
{
private:
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  bool is_publishing_area_with_objects_;
  bool is_publishing_clearance_map_;
  bool is_showing_debug_info_;
  bool is_using_vehicle_config_;
  bool is_stopping_if_outside_drivable_area_;
  bool enable_avoidance_;
  const int min_num_points_for_getting_yaw_;
  std::mutex mutex_;

  // params outside logic
  double min_delta_dist_for_replan_;
  double min_delta_time_sec_for_replan_;
  double max_dist_for_extending_end_point_;
  double distance_for_path_shape_change_detection_;

  // logic
  std::unique_ptr<EBPathOptimizer> eb_path_optimizer_ptr_;

  // params
  std::unique_ptr<QPParam> qp_param_;
  std::unique_ptr<TrajectoryParam> traj_param_;
  std::unique_ptr<ConstrainParam> constrain_param_;
  std::unique_ptr<VehicleParam> vehicle_param_;
  std::unique_ptr<MPTParam> mpt_param_;

  std::unique_ptr<geometry_msgs::msg::Pose> current_ego_pose_ptr_;
  std::unique_ptr<geometry_msgs::msg::TwistStamped> current_twist_ptr_;
  std::unique_ptr<geometry_msgs::msg::Pose> prev_ego_pose_ptr_;
  std::unique_ptr<Trajectories> prev_trajectories_ptr_;
  std::unique_ptr<std::vector<autoware_auto_planning_msgs::msg::PathPoint>> prev_path_points_ptr_;
  std::unique_ptr<autoware_auto_perception_msgs::msg::PredictedObjects> in_objects_ptr_;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_ptr_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;
  std::unique_ptr<rclcpp::Time> prev_replanned_time_ptr_;

  // ROS
  rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr avoiding_traj_pub_;
  rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr
    debug_smoothed_points_pub_;
  rclcpp::Publisher<autoware_planning_msgs::msg::IsAvoidancePossible>::SharedPtr
    is_avoidance_possible_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_markers_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr debug_clearance_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr debug_object_clearance_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr debug_area_with_objects_pub_;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr
    objects_sub_;
  rclcpp::Subscription<autoware_planning_msgs::msg::EnableAvoidance>::SharedPtr is_avoidance_sub_;

  // callback functions
  void pathCallback(const autoware_auto_planning_msgs::msg::Path::SharedPtr);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr);
  void objectsCallback(const autoware_auto_perception_msgs::msg::PredictedObjects::SharedPtr);
  void enableAvoidanceCallback(const autoware_planning_msgs::msg::EnableAvoidance::SharedPtr);

  void initialize();

  // generate fine trajectory
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> generatePostProcessedTrajectory(
    const geometry_msgs::msg::Pose & ego_pose,
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
    const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & merged_optimized_points)
    const;

  bool needReplan(
    const geometry_msgs::msg::Pose & ego_pose,
    const std::unique_ptr<geometry_msgs::msg::Pose> & prev_ego_pose,
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
    const std::unique_ptr<rclcpp::Time> & previous_replanned_time,
    const std::unique_ptr<std::vector<autoware_auto_planning_msgs::msg::PathPoint>> &
      prev_path_points,
    std::unique_ptr<Trajectories> & prev_traj_points);

  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> generateOptimizedTrajectory(
    const geometry_msgs::msg::Pose & ego_pose,
    const autoware_auto_planning_msgs::msg::Path & input_path);

  std::unique_ptr<geometry_msgs::msg::Pose> getCurrentEgoPose();

  bool isPathShapeChanged(
    const geometry_msgs::msg::Pose & ego_pose,
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
    const std::unique_ptr<std::vector<autoware_auto_planning_msgs::msg::PathPoint>> &
      prev_path_points);

  autoware_auto_planning_msgs::msg::Trajectory generateTrajectory(
    const autoware_auto_planning_msgs::msg::Path & in_path);

  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> convertPointsToTrajectory(
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
    const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & trajectory_points) const;

  void publishingDebugData(
    const DebugData & debug_data, const autoware_auto_planning_msgs::msg::Path & path,
    const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & traj_points,
    const VehicleParam & vehicle_param);

  int calculateNonDecelerationRange(
    const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & traj_points,
    const geometry_msgs::msg::Pose & ego_pose, const geometry_msgs::msg::Twist & ego_twist) const;

  Trajectories getTrajectoryInsideArea(
    const Trajectories & trajs,
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
    const cv::Mat & road_clearance_map, const nav_msgs::msg::MapMetaData & map_info,
    DebugData * debug_data) const;

  boost::optional<Trajectories> calcTrajectoryInsideArea(
    const Trajectories & trajs,
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
    const cv::Mat & road_clearance_map, const nav_msgs::msg::MapMetaData & map_info,
    DebugData * debug_data, const bool is_prev_traj = false) const;

  Trajectories getPrevTrajs(
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points) const;

  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> getPrevTrajectory(
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points) const;

  Trajectories makePrevTrajectories(
    const geometry_msgs::msg::Pose & ego_pose,
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
    const Trajectories & trajs) const;

  Trajectories getBaseTrajectory(
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
    const Trajectories & current_trajs) const;

  boost::optional<int> getStopIdx(
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
    const Trajectories & trajs, const nav_msgs::msg::MapMetaData & map_info,
    const cv::Mat & road_clearance_map, DebugData * debug_data) const;

  void declareObstacleAvoidancePlannerParameters();

  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);

public:
  explicit ObstacleAvoidancePlanner(const rclcpp::NodeOptions & node_options);
  ~ObstacleAvoidancePlanner();
};

#endif  // OBSTACLE_AVOIDANCE_PLANNER__NODE_HPP_
