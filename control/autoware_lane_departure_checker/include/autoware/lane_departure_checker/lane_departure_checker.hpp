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

#ifndef AUTOWARE__LANE_DEPARTURE_CHECKER__LANE_DEPARTURE_CHECKER_HPP_
#define AUTOWARE__LANE_DEPARTURE_CHECKER__LANE_DEPARTURE_CHECKER_HPP_

#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <autoware/universe_utils/geometry/pose_deviation.hpp>
#include <autoware/universe_utils/system/time_keeper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>

#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <boost/geometry/algorithms/envelope.hpp>
#include <boost/geometry/algorithms/union.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::lane_departure_checker
{
using autoware::universe_utils::LinearRing2d;
using autoware::universe_utils::PoseDeviation;
using autoware::universe_utils::Segment2d;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using tier4_planning_msgs::msg::PathWithLaneId;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
typedef boost::geometry::index::rtree<Segment2d, boost::geometry::index::rstar<16>> SegmentRtree;

struct Param
{
  double footprint_margin_scale{0.0};
  double footprint_extra_margin{0.0};
  double resample_interval{0.0};
  double max_deceleration{0.0};
  double delay_time{0.0};
  double max_lateral_deviation{0.0};
  double max_longitudinal_deviation{0.0};
  double max_yaw_deviation_deg{0.0};
  double min_braking_distance{0.0};
  // nearest search to ego
  double ego_nearest_dist_threshold{0.0};
  double ego_nearest_yaw_threshold{0.0};
};

struct Input
{
  nav_msgs::msg::Odometry::ConstSharedPtr current_odom{};
  lanelet::LaneletMapPtr lanelet_map{};
  LaneletRoute::ConstSharedPtr route{};
  lanelet::ConstLanelets route_lanelets{};
  lanelet::ConstLanelets shoulder_lanelets{};
  Trajectory::ConstSharedPtr reference_trajectory{};
  Trajectory::ConstSharedPtr predicted_trajectory{};
  std::vector<std::string> boundary_types_to_detect{};
};

struct Output
{
  std::map<std::string, double> processing_time_map{};
  bool will_leave_lane{};
  bool is_out_of_lane{};
  bool will_cross_boundary{};
  PoseDeviation trajectory_deviation{};
  lanelet::ConstLanelets candidate_lanelets{};
  TrajectoryPoints resampled_trajectory{};
  std::vector<LinearRing2d> vehicle_footprints{};
  std::vector<LinearRing2d> vehicle_passing_areas{};
};

class LaneDepartureChecker
{
public:
  LaneDepartureChecker(
    std::shared_ptr<universe_utils::TimeKeeper> time_keeper =
      std::make_shared<universe_utils::TimeKeeper>())
  : time_keeper_(time_keeper)
  {
  }
  Output update(const Input & input);

  void setParam(const Param & param, const autoware::vehicle_info_utils::VehicleInfo vehicle_info)
  {
    param_ = param;
    vehicle_info_ptr_ = std::make_shared<autoware::vehicle_info_utils::VehicleInfo>(vehicle_info);
  }

  void setParam(const Param & param) { param_ = param; }

  void setVehicleInfo(const autoware::vehicle_info_utils::VehicleInfo vehicle_info)
  {
    vehicle_info_ptr_ = std::make_shared<autoware::vehicle_info_utils::VehicleInfo>(vehicle_info);
  }

  bool checkPathWillLeaveLane(
    const lanelet::ConstLanelets & lanelets, const PathWithLaneId & path) const;

  std::vector<std::pair<double, lanelet::Lanelet>> getLaneletsFromPath(
    const lanelet::LaneletMapPtr lanelet_map_ptr, const PathWithLaneId & path) const;

  std::optional<autoware::universe_utils::Polygon2d> getFusedLaneletPolygonForPath(
    const lanelet::LaneletMapPtr lanelet_map_ptr, const PathWithLaneId & path) const;

  bool updateFusedLaneletPolygonForPath(
    const lanelet::LaneletMapPtr lanelet_map_ptr, const PathWithLaneId & path,
    std::vector<lanelet::Id> & fused_lanelets_id,
    std::optional<autoware::universe_utils::Polygon2d> & fused_lanelets_polygon) const;

  bool checkPathWillLeaveLane(
    const lanelet::LaneletMapPtr lanelet_map_ptr, const PathWithLaneId & path) const;

  bool checkPathWillLeaveLane(
    const lanelet::LaneletMapPtr lanelet_map_ptr, const PathWithLaneId & path,
    std::vector<lanelet::Id> & fused_lanelets_id,
    std::optional<autoware::universe_utils::Polygon2d> & fused_lanelets_polygon) const;

  PathWithLaneId cropPointsOutsideOfLanes(
    const lanelet::LaneletMapPtr lanelet_map_ptr, const PathWithLaneId & path,
    const size_t end_index);

  static bool isOutOfLane(
    const lanelet::ConstLanelets & candidate_lanelets, const LinearRing2d & vehicle_footprint);

private:
  Param param_;
  std::shared_ptr<autoware::vehicle_info_utils::VehicleInfo> vehicle_info_ptr_;

  static PoseDeviation calcTrajectoryDeviation(
    const Trajectory & trajectory, const geometry_msgs::msg::Pose & pose,
    const double dist_threshold, const double yaw_threshold);

  //! This function assumes the input trajectory is sampled dense enough
  static TrajectoryPoints resampleTrajectory(const Trajectory & trajectory, const double interval);

  static TrajectoryPoints cutTrajectory(const TrajectoryPoints & trajectory, const double length);

  std::vector<LinearRing2d> createVehicleFootprints(
    const geometry_msgs::msg::PoseWithCovariance & covariance, const TrajectoryPoints & trajectory,
    const Param & param);
  std::vector<LinearRing2d> createVehicleFootprints(const PathWithLaneId & path) const;

  static std::vector<LinearRing2d> createVehiclePassingAreas(
    const std::vector<LinearRing2d> & vehicle_footprints);

  bool willLeaveLane(
    const lanelet::ConstLanelets & candidate_lanelets,
    const std::vector<LinearRing2d> & vehicle_footprints) const;

  double calcMaxSearchLengthForBoundaries(const Trajectory & trajectory) const;

  static SegmentRtree extractUncrossableBoundaries(
    const lanelet::LaneletMap & lanelet_map, const geometry_msgs::msg::Point & ego_point,
    const double max_search_length, const std::vector<std::string> & boundary_types_to_detect);

  bool willCrossBoundary(
    const std::vector<LinearRing2d> & vehicle_footprints,
    const SegmentRtree & uncrossable_segments) const;

  lanelet::BasicPolygon2d toBasicPolygon2D(const LinearRing2d & footprint_hull) const;
  autoware::universe_utils::Polygon2d toPolygon2D(const lanelet::BasicPolygon2d & poly) const;

  mutable std::shared_ptr<universe_utils::TimeKeeper> time_keeper_;
};
}  // namespace autoware::lane_departure_checker

#endif  // AUTOWARE__LANE_DEPARTURE_CHECKER__LANE_DEPARTURE_CHECKER_HPP_
