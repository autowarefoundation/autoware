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

#include <rclcpp/rclcpp.hpp>
#include <scene_module/crosswalk/scene_crosswalk.hpp>
#include <utilization/util.hpp>

#include <cmath>
#include <vector>

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point>;
using Line = bg::model::linestring<Point>;

CrosswalkModule::CrosswalkModule(
  const int64_t module_id, const lanelet::ConstLanelet & crosswalk,
  const PlannerParam & planner_param, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock),
  module_id_(module_id),
  crosswalk_(crosswalk),
  state_(State::APPROACH)
{
  planner_param_ = planner_param;
}

bool CrosswalkModule::modifyPathVelocity(
  autoware_auto_planning_msgs::msg::PathWithLaneId * path,
  tier4_planning_msgs::msg::StopReason * stop_reason)
{
  debug_data_ = DebugData();
  debug_data_.base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;
  first_stop_path_point_index_ = static_cast<int>(path->points.size()) - 1;
  *stop_reason =
    planning_utils::initializeStopReason(tier4_planning_msgs::msg::StopReason::CROSSWALK);

  const auto input = *path;

  // create polygon
  lanelet::CompoundPolygon3d lanelet_polygon = crosswalk_.polygon3d();
  Polygon polygon;
  for (const auto & lanelet_point : lanelet_polygon) {
    polygon.outer().push_back(bg::make<Point>(lanelet_point.x(), lanelet_point.y()));
  }
  polygon.outer().push_back(polygon.outer().front());
  polygon = isClockWise(polygon) ? polygon : inverseClockWise(polygon);

  // check state
  geometry_msgs::msg::PoseStamped self_pose = planner_data_->current_pose;
  if (bg::within(Point(self_pose.pose.position.x, self_pose.pose.position.y), polygon)) {
    state_ = State::INSIDE;
  } else if (state_ == State::INSIDE) {
    state_ = State::GO_OUT;
  }

  if (state_ == State::APPROACH) {
    // check person in polygon
    const auto objects_ptr = planner_data_->predicted_objects;
    const auto no_ground_pointcloud_ptr = planner_data_->no_ground_pointcloud;

    autoware_auto_planning_msgs::msg::PathWithLaneId slow_path, stop_path;
    if (!checkSlowArea(input, polygon, objects_ptr, no_ground_pointcloud_ptr, slow_path)) {
      return false;
    }

    bool insert_stop;
    if (!checkStopArea(
          slow_path, polygon, objects_ptr, no_ground_pointcloud_ptr, stop_path, &insert_stop)) {
      return false;
    }
    // stop_path = slow_path;
    *path = stop_path;

    if (insert_stop) {
      /* get stop point and stop factor */
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = debug_data_.first_stop_pose;
      stop_factor.stop_factor_points = debug_data_.stop_factor_points;
      planning_utils::appendStopReason(stop_factor, stop_reason);
    }
  }
  return true;
}

bool CrosswalkModule::checkStopArea(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input, const Polygon & crosswalk_polygon,
  const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr & objects_ptr,
  [[maybe_unused]] const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & no_ground_pointcloud_ptr,
  autoware_auto_planning_msgs::msg::PathWithLaneId & output, bool * insert_stop)
{
  output = input;
  *insert_stop = false;
  bool pedestrian_found = false;
  bool object_found = false;
  const bool external_slowdown =
    isTargetExternalInputStatus(tier4_api_msgs::msg::CrosswalkStatus::SLOWDOWN);
  const bool external_stop =
    isTargetExternalInputStatus(tier4_api_msgs::msg::CrosswalkStatus::STOP);
  const bool external_go = isTargetExternalInputStatus(tier4_api_msgs::msg::CrosswalkStatus::GO);
  rclcpp::Time current_time = clock_->now();

  // create stop area
  Polygon stop_polygon;
  if (!createVehiclePathPolygonInCrosswalk(output, crosswalk_polygon, 1.0, stop_polygon)) {
    return false;
  }

  // -- debug code --
  std::vector<Eigen::Vector3d> points;
  for (size_t i = 0; i < stop_polygon.outer().size(); ++i) {
    const auto p = stop_polygon.outer().at(i);
    points.push_back(Eigen::Vector3d(p.x(), p.y(), planner_data_->current_pose.pose.position.z));
  }
  debug_data_.stop_polygons.push_back(points);
  // ----------------

  // check object pointcloud
  // for (size_t i = 0; i < no_ground_pointcloud_ptr->size(); ++i) {
  //   Point point(no_ground_pointcloud_ptr->at(i).x, no_ground_pointcloud_ptr->at(i).y);
  //   if (!bg::within(point, crosswalk_polygon)) continue;
  //   if (bg::within(point, stop_polygon)) {
  //     object_found = true;
  //     debug_data_.stop_factor_points.emplace_back(
  //       planning_utils::toRosPoint(no_ground_pointcloud_ptr->at(i)));
  //     break;
  //   }
  // }

  // check pedestrian
  for (const auto & object : objects_ptr->objects) {
    if (object_found) {
      break;
    }

    if (isTargetType(object)) {
      Point point(
        object.kinematics.initial_pose_with_covariance.pose.position.x,
        object.kinematics.initial_pose_with_covariance.pose.position.y);
      if (!bg::within(point, crosswalk_polygon)) {
        continue;
      }
      if (bg::within(point, stop_polygon)) {
        pedestrian_found = true;
        debug_data_.stop_factor_points.emplace_back(
          object.kinematics.initial_pose_with_covariance.pose.position);
        break;
      }
      for (const auto & object_path : object.kinematics.predicted_paths) {
        for (size_t k = 0; k < object_path.path.size() - 1; ++k) {
          const auto predicted_time =
            rclcpp::Time(objects_ptr->header.stamp) +
            rclcpp::Duration(object_path.time_step) * static_cast<double>(k);
          if (
            (predicted_time - current_time).seconds() <
            planner_param_.stop_predicted_object_prediction_time_margin) {
            const auto op0 = object_path.path.at(k).position;
            const auto op1 = object_path.path.at(k + 1).position;
            const Line line{{op0.x, op0.y}, {op1.x, op1.y}};
            std::vector<Point> line_collision_points;
            bg::intersection(stop_polygon, line, line_collision_points);
            if (!line_collision_points.empty()) {
              pedestrian_found = true;
            }
            if (pedestrian_found) {
              debug_data_.stop_factor_points.emplace_back(
                object.kinematics.initial_pose_with_covariance.pose.position);
              break;
            }
          }
        }
      }
    }
  }

  bool stop = false;

  if (pedestrian_found || object_found) {
    stop = true;
  }
  if (external_go || external_slowdown) {
    stop = false;
  } else if (external_stop) {
    stop = true;
  }

  // insert stop point
  if (stop) {
    lanelet::Optional<lanelet::ConstLineString3d> stop_line_opt =
      getStopLineFromMap(module_id_, planner_data_, "crosswalk_id");
    if (!!stop_line_opt) {
      if (!insertTargetVelocityPoint(
            input, stop_line_opt.get(), planner_param_.stop_margin, 0.0, *planner_data_, output,
            debug_data_, first_stop_path_point_index_)) {
        return false;
      }
    } else {
      if (!insertTargetVelocityPoint(
            input, crosswalk_polygon,
            planner_param_.stop_line_distance + planner_param_.stop_margin, 0.0, *planner_data_,
            output, debug_data_, first_stop_path_point_index_)) {
        return false;
      }
      *insert_stop = stop;
    }
  }
  return true;
}

bool CrosswalkModule::checkSlowArea(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input, const Polygon & crosswalk_polygon,
  const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr & objects_ptr,
  [[maybe_unused]] const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & no_ground_pointcloud_ptr,
  autoware_auto_planning_msgs::msg::PathWithLaneId & output)
{
  output = input;
  bool pedestrian_found = false;
  const bool external_slowdown =
    isTargetExternalInputStatus(tier4_api_msgs::msg::CrosswalkStatus::SLOWDOWN);
  const bool external_stop =
    isTargetExternalInputStatus(tier4_api_msgs::msg::CrosswalkStatus::STOP);
  const bool external_go = isTargetExternalInputStatus(tier4_api_msgs::msg::CrosswalkStatus::GO);

  Polygon slowdown_polygon;
  if (!createVehiclePathPolygonInCrosswalk(output, crosswalk_polygon, 4.0, slowdown_polygon)) {
    return false;
  }

  // -- debug code --
  std::vector<Eigen::Vector3d> points;
  for (size_t i = 0; i < slowdown_polygon.outer().size(); ++i) {
    Eigen::Vector3d point;
    point << slowdown_polygon.outer().at(i).x(), slowdown_polygon.outer().at(i).y(),
      planner_data_->current_pose.pose.position.z;
    points.push_back(point);
  }
  debug_data_.slow_polygons.push_back(points);
  // ----------------

  // check pedestrian
  for (const auto & object : objects_ptr->objects) {
    if (isTargetType(object)) {
      Point point(
        object.kinematics.initial_pose_with_covariance.pose.position.x,
        object.kinematics.initial_pose_with_covariance.pose.position.y);
      if (!bg::within(point, crosswalk_polygon)) {
        continue;
      }
      if (bg::within(point, slowdown_polygon)) {
        pedestrian_found = true;
      }
    }
  }

  bool slowdown = false;

  if (pedestrian_found) {
    slowdown = true;
  }
  if (external_go || external_stop) {
    slowdown = false;
  } else if (external_slowdown) {
    slowdown = true;
  }

  // insert slow point
  if (slowdown) {
    lanelet::Optional<lanelet::ConstLineString3d> stop_line_opt =
      getStopLineFromMap(module_id_, planner_data_, "crosswalk_id");
    if (!!stop_line_opt) {
      if (!insertTargetVelocityPoint(
            input, stop_line_opt.get(), planner_param_.slow_margin, planner_param_.slow_velocity,
            *planner_data_, output, debug_data_, first_stop_path_point_index_)) {
        return false;
      }
    } else {
      if (!insertTargetVelocityPoint(
            input, slowdown_polygon, planner_param_.stop_line_distance + planner_param_.slow_margin,
            planner_param_.slow_velocity, *planner_data_, output, debug_data_,
            first_stop_path_point_index_)) {
        return false;
      }
    }
  }
  return true;
}
bool CrosswalkModule::createVehiclePathPolygonInCrosswalk(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input, const Polygon & crosswalk_polygon,
  const float extended_width, Polygon & path_polygon)
{
  std::vector<Point> path_collision_points;
  for (size_t i = 0; i < input.points.size() - 1; ++i) {
    const auto p0 = input.points.at(i).point.pose.position;
    const auto p1 = input.points.at(i + 1).point.pose.position;
    const Line line{{p0.x, p0.y}, {p1.x, p1.y}};
    std::vector<Point> line_collision_points;
    bg::intersection(crosswalk_polygon, line, line_collision_points);
    if (line_collision_points.empty()) {
      continue;
    }
    for (size_t j = 0; j < line_collision_points.size(); ++j) {
      path_collision_points.push_back(line_collision_points.at(j));
    }
  }
  if (path_collision_points.size() != 2) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 5000,
      "There must be two points of conflict between the crosswalk polygon and the path. points is "
      "%d",
      (int)path_collision_points.size());
    return false;
  }

  Polygon candidate_path_polygon;
  {
    const auto cp0 = path_collision_points.at(0);
    const auto cp1 = path_collision_points.at(1);

    const double vehicle_width = planner_data_->vehicle_info_.vehicle_width_m;
    const double polygon_width = (vehicle_width / 2.0) + extended_width;
    const double polygon_extend_length = std::hypot(cp0.x() - cp1.x(), cp0.y() - cp1.y());

    const double yaw = std::atan2(cp1.y() - cp0.y(), cp1.x() - cp0.x());
    const double w_cos = polygon_width * std::cos(yaw);
    const double w_sin = polygon_width * std::sin(yaw);
    const double l_cos = polygon_extend_length * std::cos(yaw);
    const double l_sin = polygon_extend_length * std::sin(yaw);

    // The candidate_path_polygon is generated to be
    // a constant width(polygon_width) from the centerline of the path
    // across the crosswalk. The length of the candidate_path_polygon
    // is also extended so that the four corners of the polygon
    // are not inside the crosswalk polygon.
    candidate_path_polygon.outer().push_back(
      bg::make<Point>(cp0.x() + w_sin - l_cos, cp0.y() - w_cos - l_sin));
    candidate_path_polygon.outer().push_back(
      bg::make<Point>(cp0.x() - w_sin - l_cos, cp0.y() + w_cos - l_sin));
    candidate_path_polygon.outer().push_back(
      bg::make<Point>(cp1.x() - w_sin + l_cos, cp1.y() + w_cos + l_sin));
    candidate_path_polygon.outer().push_back(
      bg::make<Point>(cp1.x() + w_sin + l_cos, cp1.y() - w_cos + l_sin));
    candidate_path_polygon.outer().push_back(candidate_path_polygon.outer().front());
  }
  candidate_path_polygon = isClockWise(candidate_path_polygon)
                             ? candidate_path_polygon
                             : inverseClockWise(candidate_path_polygon);

  std::vector<Polygon> path_polygons;
  bg::intersection(crosswalk_polygon, candidate_path_polygon, path_polygons);

  if (path_polygons.size() != 1) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 5000, "Number of polygon is %d. Must be 1", (int)path_polygons.size());
    return false;
  }
  path_polygon = path_polygons.at(0);
  return true;
}

bool CrosswalkModule::isTargetType(const autoware_auto_perception_msgs::msg::PredictedObject & obj)
{
  if (
    obj.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::PEDESTRIAN ||
    obj.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::BICYCLE) {
    return true;
  }
  return false;
}

bool CrosswalkModule::isTargetExternalInputStatus(const int target_status)
{
  return planner_data_->external_crosswalk_status_input &&
         planner_data_->external_crosswalk_status_input.get().status == target_status &&
         (clock_->now() - planner_data_->external_crosswalk_status_input.get().header.stamp)
             .seconds() < planner_param_.external_input_timeout;
}
}  // namespace behavior_velocity_planner
