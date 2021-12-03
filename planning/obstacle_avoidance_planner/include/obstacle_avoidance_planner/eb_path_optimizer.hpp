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
#ifndef OBSTACLE_AVOIDANCE_PLANNER__EB_PATH_OPTIMIZER_HPP_
#define OBSTACLE_AVOIDANCE_PLANNER__EB_PATH_OPTIMIZER_HPP_

#include <eigen3/Eigen/Core>
#include <opencv2/core.hpp>
#include <rclcpp/clock.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_point.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>

#include <boost/optional/optional_fwd.hpp>

#include <memory>
#include <vector>

struct Bounds;
struct MPTParam;
struct ReferencePoint;

struct Anchor
{
  geometry_msgs::msg::Pose pose;
  double velocity;
};

struct ConstrainLines
{
  double x_coef;
  double y_coef;
  double lower_bound;
  double upper_bound;
};

struct Constrain
{
  ConstrainLines top_and_bottom;
  ConstrainLines left_and_right;
};

struct Rectangle
{
  geometry_msgs::msg::Point top_left;
  geometry_msgs::msg::Point top_right;
  geometry_msgs::msg::Point bottom_left;
  geometry_msgs::msg::Point bottom_right;
};

struct OccupancyMaps
{
  std::vector<std::vector<int>> object_occupancy_map;
  std::vector<std::vector<int>> road_occupancy_map;
};

namespace autoware::common::osqp
{
class OSQPInterface;
}

namespace util
{
struct Rectangle;
struct Footprint;
}  // namespace util

class MPTOptimizer;

struct ConstrainRectangle
{
  geometry_msgs::msg::Point top_left;
  geometry_msgs::msg::Point top_right;
  geometry_msgs::msg::Point bottom_left;
  geometry_msgs::msg::Point bottom_right;
  double velocity;
  bool is_empty_driveable_area = false;
  bool is_including_only_smooth_range = true;
};

struct ConstrainRectangles
{
  ConstrainRectangle object_constrain_rectangle;
  ConstrainRectangle road_constrain_rectangle;
};

struct CandidatePoints
{
  std::vector<geometry_msgs::msg::Pose> fixed_points;
  std::vector<geometry_msgs::msg::Pose> non_fixed_points;
  int begin_path_idx;
  int end_path_idx;
};

struct QPParam
{
  int max_iteration;
  double eps_abs;
  double eps_rel;
  double eps_abs_for_extending;
  double eps_rel_for_extending;
  double eps_abs_for_visualizing;
  double eps_rel_for_visualizing;
};

struct TrajectoryParam
{
  bool is_avoiding_unknown;
  bool is_avoiding_car;
  bool is_avoiding_truck;
  bool is_avoiding_bus;
  bool is_avoiding_bicycle;
  bool is_avoiding_motorbike;
  bool is_avoiding_pedestrian;
  bool is_avoiding_animal;
  int num_sampling_points;
  int num_joint_buffer_points;
  int num_joint_buffer_points_for_extending;
  int num_offset_for_begin_idx;
  int num_fix_points_for_extending;
  double delta_arc_length_for_optimization;
  double delta_arc_length_for_mpt_points;
  double delta_arc_length_for_trajectory;
  double delta_dist_threshold_for_closest_point;
  double delta_yaw_threshold_for_closest_point;
  double delta_yaw_threshold_for_straight;
  double trajectory_length;
  double forward_fixing_distance;
  double forward_fixing_mpt_distance;
  double backward_fixing_distance;
  double max_avoiding_ego_velocity_ms;
  double max_avoiding_objects_velocity_ms;
  double center_line_width;
  double acceleration_for_non_deceleration_range;
  double max_dist_for_extending_end_point;
};

struct Trajectories
{
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> smoothed_trajectory;
  std::vector<ReferencePoint> mpt_ref_points;
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> model_predictive_trajectory;
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> extended_trajectory;
};

struct ConstrainParam
{
  bool is_getting_constraints_close2path_points;
  double clearance_for_fixing;
  double clearance_for_straight_line;
  double clearance_for_joint;
  double range_for_extend_joint;
  double clearance_for_only_smoothing;
  double clearance_from_object_for_straight;
  double min_object_clearance_for_joint;
  double min_object_clearance_for_deceleration;
  double clearance_from_road;
  double clearance_from_object;
  double extra_desired_clearance_from_road;
  double max_x_constrain_search_range;
  double coef_x_constrain_search_resolution;
  double coef_y_constrain_search_resolution;
  double keep_space_shape_x;
  double keep_space_shape_y;
  double max_lon_space_for_driveable_constraint;
};

struct VehicleParam
{
  double wheelbase;
  double length;
  double width;
  double rear_overhang;
  double front_overhang;
  double max_steer_rad;
  double steer_tau;
};

struct FOAData
{
  bool is_avoidance_possible = true;
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> avoiding_traj_points;
  std::vector<ConstrainRectangle> constrain_rectangles;
};

struct DebugData
{
  bool is_expected_to_over_drivable_area = false;
  std::vector<geometry_msgs::msg::Point> interpolated_points;
  std::vector<geometry_msgs::msg::Point> straight_points;
  std::vector<geometry_msgs::msg::Pose> fixed_points;
  std::vector<geometry_msgs::msg::Pose> non_fixed_points;
  std::vector<ConstrainRectangle> constrain_rectangles;
  std::vector<geometry_msgs::msg::Pose> fixed_points_for_extending;
  std::vector<geometry_msgs::msg::Pose> non_fixed_points_for_extending;
  std::vector<ConstrainRectangle> constrain_rectangles_for_extending;
  std::vector<geometry_msgs::msg::Point> interpolated_points_for_extending;
  std::vector<util::Footprint> vehicle_footprints;
  std::vector<util::Footprint> current_vehicle_footprints;
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> avoiding_traj_points;
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> smoothed_points;
  std::vector<autoware_auto_perception_msgs::msg::PredictedObject> avoiding_objects;
  std::vector<Bounds> bounds;
  std::vector<geometry_msgs::msg::Pose> bounds_candidate_for_base_points;
  std::vector<geometry_msgs::msg::Pose> bounds_candidate_for_top_points;
  std::vector<geometry_msgs::msg::Pose> bounds_candidate_for_mid_points;
  std::vector<geometry_msgs::msg::Pose> fixed_mpt_points;
  cv::Mat clearance_map;
  cv::Mat only_object_clearance_map;
  cv::Mat area_with_objects_map;
  FOAData foa_data;
};

enum class OptMode : int {
  Normal = 0,
  Extending = 1,
  Visualizing = 2,
};

class EBPathOptimizer
{
private:
  rclcpp::Clock logger_ros_clock_;

  const bool is_showing_debug_info_;
  const double epsilon_;

  const QPParam qp_param_;
  const TrajectoryParam traj_param_;
  const ConstrainParam constrain_param_;
  const VehicleParam vehicle_param_;

  Eigen::MatrixXd default_a_matrix_;
  std::unique_ptr<geometry_msgs::msg::Vector3> keep_space_shape_ptr_;
  std::unique_ptr<autoware::common::osqp::OSQPInterface> osqp_solver_ptr_;
  std::unique_ptr<autoware::common::osqp::OSQPInterface> ex_osqp_solver_ptr_;
  std::unique_ptr<autoware::common::osqp::OSQPInterface> vis_osqp_solver_ptr_;

  std::unique_ptr<MPTOptimizer> mpt_optimizer_ptr_;

  void initializeSolver();

  Eigen::MatrixXd makePMatrix();

  Eigen::MatrixXd makeAMatrix();

  geometry_msgs::msg::Pose getOriginPose(
    const std::vector<geometry_msgs::msg::Point> & interpolated_points, const int interpolated_idx,
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points);

  Anchor getAnchor(
    const std::vector<geometry_msgs::msg::Point> & interpolated_points, const int interpolated_idx,
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points) const;

  boost::optional<std::vector<std::vector<geometry_msgs::msg::Point>>> getOccupancyPoints(
    const geometry_msgs::msg::Pose & origin, const cv::Mat & clearance_map,
    const nav_msgs::msg::MapMetaData & map_info) const;

  Constrain getConstrainFromConstrainRectangle(
    const geometry_msgs::msg::Point & interpolated_point,
    const ConstrainRectangle & constrain_range);

  ConstrainLines getConstrainLines(
    const double dx, const double dy, const geometry_msgs::msg::Point & point,
    const geometry_msgs::msg::Point & opposite_point);

  ConstrainRectangles getConstrainRectangles(
    const Anchor & anchor, const cv::Mat & clearance_map,
    const cv::Mat & only_objects_clearance_map, const nav_msgs::msg::MapMetaData & map_info) const;

  ConstrainRectangle getConstrainRectangle(const Anchor & anchor, const double clearance) const;

  ConstrainRectangle getConstrainRectangle(
    const Anchor & anchor, const int & nearest_idx,
    const std::vector<geometry_msgs::msg::Point> & interpolated_points,
    const cv::Mat & clearance_map, const nav_msgs::msg::MapMetaData & map_info) const;

  ConstrainRectangle getConstrainRectangle(
    const std::vector<std::vector<int>> & occupancy_map,
    const std::vector<std::vector<geometry_msgs::msg::Point>> & occupancy_points,
    const Anchor & anchor, const nav_msgs::msg::MapMetaData & map_info,
    const cv::Mat & only_objects_clearance_map) const;

  ConstrainRectangle getConstrainRectangle(
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
    const Anchor & anchor, const cv::Mat & clearance_map,
    const nav_msgs::msg::MapMetaData & map_info) const;

  ConstrainRectangle getConstrainRectangle(
    const std::vector<std::vector<geometry_msgs::msg::Point>> & occupancy_points,
    const util::Rectangle & util_rect, const Anchor & anchor) const;

  ConstrainRectangle getUpdatedConstrainRectangle(
    const ConstrainRectangle & rectangle, const geometry_msgs::msg::Point & candidate_point,
    const nav_msgs::msg::MapMetaData & map_info, const cv::Mat & only_objects_clearance_map) const;

  OccupancyMaps getOccupancyMaps(
    const std::vector<std::vector<geometry_msgs::msg::Point>> & occupancy_points,
    const geometry_msgs::msg::Pose & origin_pose,
    const geometry_msgs::msg::Point & origin_point_in_image, const cv::Mat & clearance_map,
    const cv::Mat & only_objects_clearance_map, const nav_msgs::msg::MapMetaData & map_info) const;

  int getStraightLineIdx(
    const std::vector<geometry_msgs::msg::Point> & interpolated_points,
    const int farthest_point_idx, const cv::Mat & only_objects_clearance,
    const nav_msgs::msg::MapMetaData & map_info,
    std::vector<geometry_msgs::msg::Point> & debug_detected_straight_points);

  int getEndPathIdx(
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
    const int begin_path_idx, const double required_trajectory_length);

  int getEndPathIdxInsideArea(
    const geometry_msgs::msg::Pose & ego_pose,
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
    const int begin_path_idx, const cv::Mat & drivable_area,
    const nav_msgs::msg::MapMetaData & map_info);

  boost::optional<std::vector<ConstrainRectangle>> getPostProcessedConstrainRectangles(
    const bool enable_avoidance, const std::vector<ConstrainRectangle> & object_constrains,
    const std::vector<ConstrainRectangle> & road_constrains,
    const std::vector<ConstrainRectangle> & only_smooth_constrains,
    const std::vector<geometry_msgs::msg::Point> & interpolated_points,
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
    const int farthest_point_idx, const int num_fixed_points, const int straight_idx,
    DebugData * debug_data) const;

  boost::optional<std::vector<ConstrainRectangle>> getValidConstrainRectangles(
    const std::vector<ConstrainRectangle> & constrains,
    const std::vector<ConstrainRectangle> & only_smooth_constrains, DebugData * debug_data) const;

  boost::optional<std::vector<ConstrainRectangle>> getConstrainRectanglesClose2PathPoints(
    const bool is_using_only_smooth_constrain, const bool is_using_road_constrain,
    const std::vector<ConstrainRectangle> & object_constrains,
    const std::vector<ConstrainRectangle> & road_constrains,
    const std::vector<ConstrainRectangle> & only_smooth_constrains, DebugData * debug_data) const;

  boost::optional<std::vector<ConstrainRectangle>> getConstrainRectanglesWithinArea(
    const bool is_using_only_smooth_constrain, const bool is_using_road_constrain,
    const int farthest_point_idx, const int num_fixed_points, const int straight_idx,
    const std::vector<ConstrainRectangle> & object_constrains,
    const std::vector<ConstrainRectangle> & road_constrains,
    const std::vector<ConstrainRectangle> & only_smooth_constrains,
    const std::vector<geometry_msgs::msg::Point> & interpolated_points,
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
    DebugData * debug_data) const;

  bool isPreFixIdx(
    const int target_idx, const int farthest_point_idx, const int num_fixed,
    const int straight_idx) const;

  bool isClose2Object(
    const geometry_msgs::msg::Point & point, const nav_msgs::msg::MapMetaData & map_info,
    const cv::Mat & only_objects_clearance_map, const double distance_threshold) const;

  boost::optional<std::vector<ConstrainRectangle>> getConstrainRectangleVec(
    const bool enable_avoidance, const autoware_auto_planning_msgs::msg::Path & input_path,
    const std::vector<geometry_msgs::msg::Point> & interpolated_points, const int num_fixed_points,
    const int farthest_point_idx, const int straight_idx, const cv::Mat & clearance_map,
    const cv::Mat & only_objects_clearance_map, DebugData * debug_data);

  boost::optional<std::vector<ConstrainRectangle>> getConstrainRectangleVec(
    const autoware_auto_planning_msgs::msg::Path & path,
    const std::vector<geometry_msgs::msg::Point> & interpolated_points, const int num_fixed_points,
    const int farthest_point_idx, const int straight_idx, const cv::Mat & clearance_map,
    const cv::Mat & only_objects_clearance_map);

  std::vector<ConstrainRectangle> getConstrainRectangleVec(
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & input_path,
    const std::vector<geometry_msgs::msg::Point> & interpolated_points, const int num_fixed_points,
    const int farthest_point_idx);

  Rectangle getRelShapeRectangle(
    const geometry_msgs::msg::Vector3 & vehicle_shape,
    const geometry_msgs::msg::Pose & origin) const;

  Rectangle getAbsShapeRectangle(
    const Rectangle & rel_shape_rectangle_points, const geometry_msgs::msg::Point & offset_point,
    const geometry_msgs::msg::Pose & origin) const;

  std::vector<geometry_msgs::msg::Point> getPaddedInterpolatedPoints(
    const std::vector<geometry_msgs::msg::Point> & interpolated_points, const int farthest_idx);

  int getNumFixedPoints(
    const std::vector<geometry_msgs::msg::Pose> & fixed_points,
    const std::vector<geometry_msgs::msg::Point> & interpolated_points, const int farthest_idx);

  boost::optional<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>
  getOptimizedTrajectory(
    const bool enable_avoidance, const autoware_auto_planning_msgs::msg::Path & path,
    const CandidatePoints & candidate_points, const cv::Mat & clearance_map,
    const cv::Mat & only_objects_clearance_map, DebugData * debug_data);

  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> getExtendedOptimizedTrajectory(
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
    const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & optimized_points,
    DebugData * debug_data);

  void updateConstrain(
    const std::vector<geometry_msgs::msg::Point> & interpolated_points,
    const std::vector<ConstrainRectangle> & rectangle_points, const OptMode & opt_mode);

  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> convertOptimizedPointsToTrajectory(
    const std::vector<double> optimized_points, const std::vector<ConstrainRectangle> & constraint,
    const int farthest_idx);

  std::vector<geometry_msgs::msg::Pose> getFixedPoints(
    const geometry_msgs::msg::Pose & ego_pose,
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
    const std::unique_ptr<Trajectories> & prev_optimized_points, const cv::Mat & drivable_area,
    const nav_msgs::msg::MapMetaData & map_info);

  CandidatePoints getCandidatePoints(
    const geometry_msgs::msg::Pose & ego_pose,
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
    const std::unique_ptr<Trajectories> & prev_trajs, const cv::Mat & drivable_area,
    const nav_msgs::msg::MapMetaData & map_info, DebugData * debug_data);

  bool isPointInsideDrivableArea(
    const geometry_msgs::msg::Point & point, const cv::Mat & drivable_area,
    const nav_msgs::msg::MapMetaData & map_info);

  CandidatePoints getDefaultCandidatePoints(
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points);

  std::vector<double> solveQP(const OptMode & opt_mode);

  bool isFixingPathPoint(
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points) const;

  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> calculateTrajectory(
    const std::vector<geometry_msgs::msg::Point> & padded_interpolated_points,
    const std::vector<ConstrainRectangle> & constrain_rectangles, const int farthest_idx,
    const OptMode & opt_mode);

  FOAData getFOAData(
    const std::vector<ConstrainRectangle> & rectangles,
    const std::vector<geometry_msgs::msg::Point> & interpolated_points, const int farthest_idx);

public:
  EBPathOptimizer(
    const bool is_showing_debug_info, const QPParam qp_param, const TrajectoryParam traj_param,
    const ConstrainParam constrain_param, const VehicleParam & vehicle_param,
    const MPTParam & mpt_param);

  ~EBPathOptimizer();

  boost::optional<Trajectories> generateOptimizedTrajectory(
    const bool enable_avoidance, const geometry_msgs::msg::Pose & ego_pose,
    const autoware_auto_planning_msgs::msg::Path & path,
    const std::unique_ptr<Trajectories> & prev_trajs,
    const std::vector<autoware_auto_perception_msgs::msg::PredictedObject> & objects,
    DebugData * debug_data);
};

#endif  // OBSTACLE_AVOIDANCE_PLANNER__EB_PATH_OPTIMIZER_HPP_
