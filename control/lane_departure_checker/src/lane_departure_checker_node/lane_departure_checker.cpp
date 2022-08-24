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

#include "lane_departure_checker/lane_departure_checker.hpp"

#include "lane_departure_checker/util/create_vehicle_footprint.hpp"

#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/math/normalization.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>
#include <tier4_autoware_utils/system/stop_watch.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/Polygon.h>
#include <tf2/utils.h>

#include <algorithm>
#include <vector>

using tier4_autoware_utils::LinearRing2d;
using tier4_autoware_utils::MultiPoint2d;
using tier4_autoware_utils::Point2d;

namespace
{
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

double calcBrakingDistance(
  const double abs_velocity, const double max_deceleration, const double delay_time)
{
  return (abs_velocity * abs_velocity) / (2.0 * max_deceleration) + delay_time * abs_velocity;
}

bool isInAnyLane(const lanelet::ConstLanelets & candidate_lanelets, const Point2d & point)
{
  for (const auto & ll : candidate_lanelets) {
    if (boost::geometry::within(point, ll.polygon2d().basicPolygon())) {
      return true;
    }
  }

  return false;
}

LinearRing2d createHullFromFootprints(const std::vector<LinearRing2d> & footprints)
{
  MultiPoint2d combined;
  for (const auto & footprint : footprints) {
    for (const auto & p : footprint) {
      combined.push_back(p);
    }
  }

  LinearRing2d hull;
  boost::geometry::convex_hull(combined, hull);

  return hull;
}

lanelet::ConstLanelets getCandidateLanelets(
  const lanelet::ConstLanelets & route_lanelets,
  const std::vector<LinearRing2d> & vehicle_footprints)
{
  lanelet::ConstLanelets candidate_lanelets;

  // Find lanes within the convex hull of footprints
  const auto footprint_hull = createHullFromFootprints(vehicle_footprints);
  for (const auto & route_lanelet : route_lanelets) {
    const auto poly = route_lanelet.polygon2d().basicPolygon();
    if (!boost::geometry::disjoint(poly, footprint_hull)) {
      candidate_lanelets.push_back(route_lanelet);
    }
  }

  return candidate_lanelets;
}
}  // namespace

namespace lane_departure_checker
{
Output LaneDepartureChecker::update(const Input & input)
{
  Output output{};

  tier4_autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;

  output.trajectory_deviation = calcTrajectoryDeviation(
    *input.reference_trajectory, input.current_pose->pose, param_.ego_nearest_dist_threshold,
    param_.ego_nearest_yaw_threshold);
  output.processing_time_map["calcTrajectoryDeviation"] = stop_watch.toc(true);

  {
    constexpr double min_velocity = 0.01;
    const auto & raw_abs_velocity = std::abs(input.current_odom->twist.twist.linear.x);
    const auto abs_velocity = raw_abs_velocity < min_velocity ? 0.0 : raw_abs_velocity;

    const auto braking_distance =
      calcBrakingDistance(abs_velocity, param_.max_deceleration, param_.delay_time);

    output.resampled_trajectory = cutTrajectory(
      resampleTrajectory(*input.predicted_trajectory, param_.resample_interval), braking_distance);
    output.processing_time_map["resampleTrajectory"] = stop_watch.toc(true);
  }
  output.vehicle_footprints =
    createVehicleFootprints(input.current_odom->pose, output.resampled_trajectory, param_);
  output.processing_time_map["createVehicleFootprints"] = stop_watch.toc(true);

  output.vehicle_passing_areas = createVehiclePassingAreas(output.vehicle_footprints);
  output.processing_time_map["createVehiclePassingAreas"] = stop_watch.toc(true);

  output.candidate_lanelets = getCandidateLanelets(input.route_lanelets, output.vehicle_footprints);
  output.processing_time_map["getCandidateLanelets"] = stop_watch.toc(true);

  output.will_leave_lane = willLeaveLane(output.candidate_lanelets, output.vehicle_footprints);
  output.processing_time_map["willLeaveLane"] = stop_watch.toc(true);

  output.is_out_of_lane = isOutOfLane(output.candidate_lanelets, output.vehicle_footprints.front());
  output.processing_time_map["isOutOfLane"] = stop_watch.toc(true);

  return output;
}

bool LaneDepartureChecker::checkPathWillLeaveLane(
  const lanelet::ConstLanelets & lanelets, const PathWithLaneId & path) const
{
  std::vector<LinearRing2d> vehicle_footprints = createVehicleFootprints(path);
  lanelet::ConstLanelets candidate_lanelets = getCandidateLanelets(lanelets, vehicle_footprints);
  return willLeaveLane(candidate_lanelets, vehicle_footprints);
}

PoseDeviation LaneDepartureChecker::calcTrajectoryDeviation(
  const Trajectory & trajectory, const geometry_msgs::msg::Pose & pose, const double dist_threshold,
  const double yaw_threshold)
{
  const auto nearest_idx = motion_utils::findFirstNearestIndexWithSoftConstraints(
    trajectory.points, pose, dist_threshold, yaw_threshold);
  return tier4_autoware_utils::calcPoseDeviation(trajectory.points.at(nearest_idx).pose, pose);
}

TrajectoryPoints LaneDepartureChecker::resampleTrajectory(
  const Trajectory & trajectory, const double interval)
{
  TrajectoryPoints resampled;

  resampled.push_back(trajectory.points.front());
  for (size_t i = 1; i < trajectory.points.size() - 1; ++i) {
    const auto & point = trajectory.points.at(i);

    const auto p1 = tier4_autoware_utils::fromMsg(resampled.back().pose.position);
    const auto p2 = tier4_autoware_utils::fromMsg(point.pose.position);

    if (boost::geometry::distance(p1.to_2d(), p2.to_2d()) > interval) {
      resampled.push_back(point);
    }
  }
  resampled.push_back(trajectory.points.back());

  return resampled;
}

TrajectoryPoints LaneDepartureChecker::cutTrajectory(
  const TrajectoryPoints & trajectory, const double length)
{
  TrajectoryPoints cut;

  double total_length = 0.0;
  cut.push_back(trajectory.front());
  for (size_t i = 1; i < trajectory.size(); ++i) {
    const auto & point = trajectory.at(i);

    const auto p1 = tier4_autoware_utils::fromMsg(cut.back().pose.position);
    const auto p2 = tier4_autoware_utils::fromMsg(point.pose.position);
    const auto points_distance = boost::geometry::distance(p1.to_2d(), p2.to_2d());

    const auto remain_distance = length - total_length;

    // Over length
    if (remain_distance <= 0) {
      break;
    }

    // Require interpolation
    if (remain_distance <= points_distance) {
      const Eigen::Vector3d p_interpolated = p1 + remain_distance * (p2 - p1).normalized();

      TrajectoryPoint p;
      p.pose.position.x = p_interpolated.x();
      p.pose.position.y = p_interpolated.y();
      p.pose.position.z = p_interpolated.z();
      p.pose.orientation = point.pose.orientation;

      cut.push_back(p);
      break;
    }

    cut.push_back(point);
    total_length += points_distance;
  }

  return cut;
}

std::vector<LinearRing2d> LaneDepartureChecker::createVehicleFootprints(
  const geometry_msgs::msg::PoseWithCovariance & covariance, const TrajectoryPoints & trajectory,
  const Param & param)
{
  // Calculate longitudinal and lateral margin based on covariance
  const auto margin = calcFootprintMargin(covariance, param.footprint_margin_scale);

  // Create vehicle footprint in base_link coordinate
  const auto local_vehicle_footprint = createVehicleFootprint(*vehicle_info_ptr_, margin);

  // Create vehicle footprint on each TrajectoryPoint
  std::vector<LinearRing2d> vehicle_footprints;
  for (const auto & p : trajectory) {
    vehicle_footprints.push_back(
      transformVector(local_vehicle_footprint, tier4_autoware_utils::pose2transform(p.pose)));
  }

  return vehicle_footprints;
}

std::vector<LinearRing2d> LaneDepartureChecker::createVehicleFootprints(
  const PathWithLaneId & path) const
{
  // Create vehicle footprint in base_link coordinate
  const auto local_vehicle_footprint = createVehicleFootprint(*vehicle_info_ptr_);

  // Create vehicle footprint on each Path point
  std::vector<LinearRing2d> vehicle_footprints;
  for (const auto & p : path.points) {
    vehicle_footprints.push_back(
      transformVector(local_vehicle_footprint, tier4_autoware_utils::pose2transform(p.point.pose)));
  }

  return vehicle_footprints;
}

std::vector<LinearRing2d> LaneDepartureChecker::createVehiclePassingAreas(
  const std::vector<LinearRing2d> & vehicle_footprints)
{
  // Create hull from two adjacent vehicle footprints
  std::vector<LinearRing2d> areas;
  for (size_t i = 0; i < vehicle_footprints.size() - 1; ++i) {
    const auto & footprint1 = vehicle_footprints.at(i);
    const auto & footprint2 = vehicle_footprints.at(i + 1);
    areas.push_back(createHullFromFootprints({footprint1, footprint2}));
  }

  return areas;
}

bool LaneDepartureChecker::willLeaveLane(
  const lanelet::ConstLanelets & candidate_lanelets,
  const std::vector<LinearRing2d> & vehicle_footprints)
{
  for (const auto & vehicle_footprint : vehicle_footprints) {
    if (isOutOfLane(candidate_lanelets, vehicle_footprint)) {
      return true;
    }
  }

  return false;
}

bool LaneDepartureChecker::isOutOfLane(
  const lanelet::ConstLanelets & candidate_lanelets, const LinearRing2d & vehicle_footprint)
{
  for (const auto & point : vehicle_footprint) {
    if (!isInAnyLane(candidate_lanelets, point)) {
      return true;
    }
  }

  return false;
}
}  // namespace lane_departure_checker
