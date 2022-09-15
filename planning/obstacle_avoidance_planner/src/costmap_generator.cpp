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

#include "obstacle_avoidance_planner/costmap_generator.hpp"

#include "obstacle_avoidance_planner/utils/cv_utils.hpp"
#include "obstacle_avoidance_planner/utils/utils.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

namespace
{
cv::Point toCVPoint(const geometry_msgs::msg::Point & p)
{
  cv::Point cv_point;
  cv_point.x = p.x;
  cv_point.y = p.y;
  return cv_point;
}

bool isAvoidingObjectType(
  const autoware_auto_perception_msgs::msg::PredictedObject & object,
  const TrajectoryParam & traj_param)
{
  if (
    (object.classification.at(0).label == object.classification.at(0).UNKNOWN &&
     traj_param.is_avoiding_unknown) ||
    (object.classification.at(0).label == object.classification.at(0).CAR &&
     traj_param.is_avoiding_car) ||
    (object.classification.at(0).label == object.classification.at(0).TRUCK &&
     traj_param.is_avoiding_truck) ||
    (object.classification.at(0).label == object.classification.at(0).BUS &&
     traj_param.is_avoiding_bus) ||
    (object.classification.at(0).label == object.classification.at(0).BICYCLE &&
     traj_param.is_avoiding_bicycle) ||
    (object.classification.at(0).label == object.classification.at(0).MOTORCYCLE &&
     traj_param.is_avoiding_motorbike) ||
    (object.classification.at(0).label == object.classification.at(0).PEDESTRIAN &&
     traj_param.is_avoiding_pedestrian)) {
    return true;
  }
  return false;
}

bool arePointsInsideDriveableArea(
  const std::vector<geometry_msgs::msg::Point> & image_points, const cv::Mat & clearance_map)
{
  bool points_inside_area = false;
  for (const auto & image_point : image_points) {
    const float clearance =
      clearance_map.ptr<float>(static_cast<int>(image_point.y))[static_cast<int>(image_point.x)];
    if (clearance > 0) {
      points_inside_area = true;
    }
  }
  return points_inside_area;
}

bool isAvoidingObject(
  const PolygonPoints & polygon_points,
  const autoware_auto_perception_msgs::msg::PredictedObject & object, const cv::Mat & clearance_map,
  const nav_msgs::msg::MapMetaData & map_info,
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const TrajectoryParam & traj_param)
{
  if (path_points.empty()) {
    return false;
  }
  if (!isAvoidingObjectType(object, traj_param)) {
    return false;
  }
  const auto image_point = geometry_utils::transformMapToOptionalImage(
    object.kinematics.initial_pose_with_covariance.pose.position, map_info);
  if (!image_point) {
    return false;
  }

  // TODO(murooka) remove findNearestIndex without any constraints
  const int nearest_idx = motion_utils::findNearestIndex(
    path_points, object.kinematics.initial_pose_with_covariance.pose.position);
  const auto nearest_path_point = path_points[nearest_idx];
  const auto rel_p = geometry_utils::transformToRelativeCoordinate2D(
    object.kinematics.initial_pose_with_covariance.pose.position, nearest_path_point.pose);
  // skip object located back the beginning of path points
  if (nearest_idx == 0 && rel_p.x < 0) {
    return false;
  }

  /*
  const float object_clearance_from_road =
    clearance_map.ptr<float>(
      static_cast<int>(image_point.get().y))[static_cast<int>(image_point.get().x)] *
    map_info.resolution;
    */
  const geometry_msgs::msg::Vector3 twist =
    object.kinematics.initial_twist_with_covariance.twist.linear;
  const double vel = std::sqrt(twist.x * twist.x + twist.y * twist.y + twist.z * twist.z);
  /*
  const auto nearest_path_point_image =
    geometry_utils::transformMapToOptionalImage(nearest_path_point.pose.position, map_info);
  if (!nearest_path_point_image) {
    return false;
  }
  const float nearest_path_point_clearance =
    clearance_map.ptr<float>(static_cast<int>(
      nearest_path_point_image.get().y))[static_cast<int>(nearest_path_point_image.get().x)] *
    map_info.resolution;
  */
  const double lateral_offset_to_path = motion_utils::calcLateralOffset(
    path_points, object.kinematics.initial_pose_with_covariance.pose.position);
  if (
    // nearest_path_point_clearance - traj_param.center_line_width * 0.5 <
    // object_clearance_from_road ||
    std::abs(lateral_offset_to_path) < traj_param.center_line_width * 0.5 ||
    vel > traj_param.max_avoiding_objects_velocity_ms ||
    !arePointsInsideDriveableArea(polygon_points.points_in_image, clearance_map)) {
    return false;
  }
  return true;
}
}  // namespace

namespace tier4_autoware_utils
{
template <>
geometry_msgs::msg::Point getPoint(const cv::Point & p)
{
  geometry_msgs::msg::Point geom_p;
  geom_p.x = p.x;
  geom_p.y = p.y;

  return geom_p;
}
}  // namespace tier4_autoware_utils

CVMaps CostmapGenerator::getMaps(
  const bool enable_avoidance, const autoware_auto_planning_msgs::msg::Path & path,
  const std::vector<autoware_auto_perception_msgs::msg::PredictedObject> & objects,
  const TrajectoryParam & traj_param, DebugData & debug_data)
{
  stop_watch_.tic(__func__);

  // make cv_maps
  CVMaps cv_maps;

  cv_maps.drivable_area = getDrivableAreaInCV(path.drivable_area, debug_data);
  cv_maps.clearance_map = getClearanceMap(cv_maps.drivable_area, debug_data);

  std::vector<autoware_auto_perception_msgs::msg::PredictedObject> debug_avoiding_objects;
  cv::Mat objects_image = drawObstaclesOnImage(
    enable_avoidance, objects, path.points, path.drivable_area.info, cv_maps.drivable_area,
    cv_maps.clearance_map, traj_param, &debug_avoiding_objects, debug_data);

  cv_maps.area_with_objects_map =
    getAreaWithObjects(cv_maps.drivable_area, objects_image, debug_data);
  cv_maps.only_objects_clearance_map = getClearanceMap(objects_image, debug_data);
  cv_maps.map_info = path.drivable_area.info;

  // debug data
  debug_data.clearance_map = cv_maps.clearance_map;
  debug_data.only_object_clearance_map = cv_maps.only_objects_clearance_map;
  debug_data.area_with_objects_map = cv_maps.area_with_objects_map;
  debug_data.avoiding_objects = debug_avoiding_objects;
  debug_data.msg_stream << "    " << __func__ << ":= " << stop_watch_.toc(__func__) << " [ms]\n";
  return cv_maps;
}

cv::Mat CostmapGenerator::getDrivableAreaInCV(
  const nav_msgs::msg::OccupancyGrid & occupancy_grid, DebugData & debug_data) const
{
  stop_watch_.tic(__func__);

  cv::Mat drivable_area = cv::Mat(occupancy_grid.info.width, occupancy_grid.info.height, CV_8UC1);

  drivable_area.forEach<unsigned char>([&](unsigned char & value, const int * position) -> void {
    cv_utils::getOccupancyGridValue(occupancy_grid, position[0], position[1], value);
  });

  debug_data.msg_stream << "      " << __func__ << ":= " << stop_watch_.toc(__func__) << " [ms]\n";
  return drivable_area;
}

cv::Mat CostmapGenerator::getClearanceMap(
  const cv::Mat & drivable_area, DebugData & debug_data) const
{
  stop_watch_.tic(__func__);

  cv::Mat clearance_map;
  cv::distanceTransform(drivable_area, clearance_map, cv::DIST_L2, 5);

  debug_data.msg_stream << "      " << __func__ << ":= " << stop_watch_.toc(__func__) << " [ms]\n";
  return clearance_map;
}

cv::Mat CostmapGenerator::drawObstaclesOnImage(
  const bool enable_avoidance,
  const std::vector<autoware_auto_perception_msgs::msg::PredictedObject> & objects,
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const nav_msgs::msg::MapMetaData & map_info, [[maybe_unused]] const cv::Mat & drivable_area,
  const cv::Mat & clearance_map, const TrajectoryParam & traj_param,
  std::vector<autoware_auto_perception_msgs::msg::PredictedObject> * debug_avoiding_objects,
  DebugData & debug_data) const
{
  stop_watch_.tic(__func__);

  std::vector<autoware_auto_planning_msgs::msg::PathPoint> path_points_inside_area;
  for (const auto & point : path_points) {
    std::vector<geometry_msgs::msg::Point> points;
    geometry_msgs::msg::Point image_point;
    if (!geometry_utils::transformMapToImage(point.pose.position, map_info, image_point)) {
      continue;
    }
    const float clearance =
      clearance_map.ptr<float>(static_cast<int>(image_point.y))[static_cast<int>(image_point.x)];
    if (clearance < 1e-5) {
      continue;
    }
    path_points_inside_area.push_back(point);
  }

  // NOTE: objects image is too sparse so that creating cost map is heavy.
  //       Then, objects image is created by filling dilated drivable area,
  //       instead of "cv::Mat objects_image = cv::Mat::ones(clearance_map.rows, clearance_map.cols,
  //       CV_8UC1) * 255;".
  constexpr double dilate_margin = 1.0;
  cv::Mat objects_image;
  const int dilate_size = static_cast<int>(
    (1.8 + dilate_margin) /
    (std::sqrt(2) * map_info.resolution));  // TODO(murooka) use clearance_from_object
  cv::dilate(drivable_area, objects_image, cv::Mat(), cv::Point(-1, -1), dilate_size);

  if (!enable_avoidance) {
    return objects_image;
  }

  // fill object
  std::vector<std::vector<cv::Point>> cv_polygons;
  std::vector<std::array<double, 2>> obj_cog_info;
  std::vector<geometry_msgs::msg::Point> obj_positions;
  for (const auto & object : objects) {
    const PolygonPoints polygon_points = cv_polygon_utils::getPolygonPoints(object, map_info);
    if (isAvoidingObject(
          polygon_points, object, clearance_map, map_info, path_points_inside_area, traj_param)) {
      const double lon_dist_to_path = motion_utils::calcSignedArcLength(
        path_points, 0, object.kinematics.initial_pose_with_covariance.pose.position);
      const double lat_dist_to_path = motion_utils::calcLateralOffset(
        path_points, object.kinematics.initial_pose_with_covariance.pose.position);
      obj_cog_info.push_back({lon_dist_to_path, lat_dist_to_path});
      obj_positions.push_back(object.kinematics.initial_pose_with_covariance.pose.position);

      cv_polygons.push_back(cv_polygon_utils::getCVPolygon(
        object, polygon_points, path_points_inside_area, clearance_map, map_info));
      debug_avoiding_objects->push_back(object);
    }
  }
  for (const auto & polygon : cv_polygons) {
    cv::fillConvexPoly(objects_image, polygon, cv::Scalar(0));
  }

  // fill between objects in the same side
  const auto get_closest_obj_point = [&](size_t idx) {
    // TODO(murooka) remove findNearestIndex without any constraints
    const auto & path_point =
      path_points.at(motion_utils::findNearestIndex(path_points, obj_positions.at(idx)));
    double min_dist = std::numeric_limits<double>::min();
    size_t min_idx = 0;
    for (size_t p_idx = 0; p_idx < cv_polygons.at(idx).size(); ++p_idx) {
      const double dist =
        tier4_autoware_utils::calcDistance2d(cv_polygons.at(idx).at(p_idx), path_point);
      if (dist < min_dist) {
        min_dist = dist;
        min_idx = p_idx;
      }
    }

    geometry_msgs::msg::Point geom_point;
    geom_point.x = cv_polygons.at(idx).at(min_idx).x;
    geom_point.y = cv_polygons.at(idx).at(min_idx).y;
    return geom_point;
  };

  std::vector<std::vector<cv::Point>> cv_between_polygons;
  for (size_t i = 0; i < obj_positions.size(); ++i) {
    for (size_t j = i + 1; j < obj_positions.size(); ++j) {
      const auto & obj_info1 = obj_cog_info.at(i);
      const auto & obj_info2 = obj_cog_info.at(j);

      // RCLCPP_ERROR_STREAM(rclcpp::get_logger("lat"), obj_info1.at(1) << " " << obj_info2.at(1));
      // RCLCPP_ERROR_STREAM(rclcpp::get_logger("lon"), obj_info1.at(0) << " " << obj_info2.at(0));

      constexpr double max_lon_dist_to_convex_obstacles = 30.0;
      if (
        obj_info1.at(1) * obj_info2.at(1) < 0 ||
        std::abs(obj_info1.at(0) - obj_info2.at(0)) > max_lon_dist_to_convex_obstacles) {
        continue;
      }

      std::vector<cv::Point> cv_points;
      cv_points.push_back(toCVPoint(obj_positions.at(i)));
      cv_points.push_back(toCVPoint(get_closest_obj_point(i)));
      cv_points.push_back(toCVPoint(get_closest_obj_point(j)));
      cv_points.push_back(toCVPoint(obj_positions.at(j)));

      cv_between_polygons.push_back(cv_points);
    }
  }
  /*
  for (const auto & polygon : cv_between_polygons) {
    cv::fillConvexPoly(objects_image, polygon, cv::Scalar(0));
  }
  */

  debug_data.msg_stream << "      " << __func__ << ":= " << stop_watch_.toc(__func__) << " [ms]\n";

  return objects_image;
}

cv::Mat CostmapGenerator::getAreaWithObjects(
  const cv::Mat & drivable_area, const cv::Mat & objects_image, DebugData & debug_data) const
{
  stop_watch_.tic(__func__);

  cv::Mat area_with_objects = cv::min(objects_image, drivable_area);

  debug_data.msg_stream << "      " << __func__ << ":= " << stop_watch_.toc(__func__) << " [ms]\n";
  return area_with_objects;
}
