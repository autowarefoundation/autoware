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
#include "obstacle_avoidance_planner/process_cv.hpp"

#include "obstacle_avoidance_planner/eb_path_optimizer.hpp"
#include "obstacle_avoidance_planner/util.hpp"

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <boost/optional.hpp>

#include <opencv2/imgproc/imgproc_c.h>
#include <tf2/utils.h>

#include <algorithm>
#include <limits>
#include <vector>

namespace process_cv
{
void getOccupancyGridValue(
  const nav_msgs::msg::OccupancyGrid & og, const int i, const int j, unsigned char & value)
{
  int i_flip = og.info.width - i - 1;
  int j_flip = og.info.height - j - 1;
  if (og.data[i_flip + j_flip * og.info.width] > 0) {
    value = 0;
  } else {
    value = 255;
  }
}

void putOccupancyGridValue(
  nav_msgs::msg::OccupancyGrid & og, const int i, const int j, const unsigned char value)
{
  int i_flip = og.info.width - i - 1;
  int j_flip = og.info.height - j - 1;
  og.data[i_flip + j_flip * og.info.width] = value;
}

cv::Mat drawObstaclesOnImage(
  const bool enable_avoidance,
  const std::vector<autoware_auto_perception_msgs::msg::PredictedObject> & objects,
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const nav_msgs::msg::MapMetaData & map_info, const cv::Mat & clearance_map,
  const TrajectoryParam & traj_param,
  std::vector<autoware_auto_perception_msgs::msg::PredictedObject> * debug_avoiding_objects)
{
  std::vector<autoware_auto_planning_msgs::msg::PathPoint> path_points_inside_area;
  for (const auto & point : path_points) {
    std::vector<geometry_msgs::msg::Point> points;
    geometry_msgs::msg::Point image_point;
    if (!util::transformMapToImage(point.pose.position, map_info, image_point)) {
      continue;
    }
    const float clearance =
      clearance_map.ptr<float>(static_cast<int>(image_point.y))[static_cast<int>(image_point.x)];
    if (clearance < 1e-5) {
      continue;
    }
    path_points_inside_area.push_back(point);
  }
  cv::Mat objects_image = cv::Mat::ones(clearance_map.rows, clearance_map.cols, CV_8UC1) * 255;
  if (!enable_avoidance) {
    return objects_image;
  }

  std::vector<std::vector<cv::Point>> cv_polygons;
  for (const auto & object : objects) {
    const PolygonPoints polygon_points = getPolygonPoints(object, map_info);
    if (isAvoidingObject(
          polygon_points, object, clearance_map, map_info, path_points_inside_area, traj_param)) {
      cv_polygons.push_back(
        getCVPolygon(object, polygon_points, path_points_inside_area, clearance_map, map_info));
      debug_avoiding_objects->push_back(object);
    }
  }
  for (const auto & polygon : cv_polygons) {
    cv::fillConvexPoly(objects_image, polygon, cv::Scalar(0));
  }
  return objects_image;
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
  const auto image_point = util::transformMapToOptionalImage(
    object.kinematics.initial_pose_with_covariance.pose.position, map_info);
  if (!image_point) {
    return false;
  }

  const int nearest_idx =
    util::getNearestIdx(path_points, object.kinematics.initial_pose_with_covariance.pose.position);
  const auto nearest_path_point = path_points[nearest_idx];
  const auto rel_p = util::transformToRelativeCoordinate2D(
    object.kinematics.initial_pose_with_covariance.pose.position, nearest_path_point.pose);
  // skip object located back the beginning of path points
  if (nearest_idx == 0 && rel_p.x < 0) {
    return false;
  }

  const float object_clearance_from_road =
    clearance_map.ptr<float>(
      static_cast<int>(image_point.get().y))[static_cast<int>(image_point.get().x)] *
    map_info.resolution;
  const geometry_msgs::msg::Vector3 twist =
    object.kinematics.initial_twist_with_covariance.twist.linear;
  const double vel = std::sqrt(twist.x * twist.x + twist.y * twist.y + twist.z * twist.z);
  const auto nearest_path_point_image =
    util::transformMapToOptionalImage(nearest_path_point.pose.position, map_info);
  if (!nearest_path_point_image) {
    return false;
  }
  const float nearest_path_point_clearance =
    clearance_map.ptr<float>(static_cast<int>(
      nearest_path_point_image.get().y))[static_cast<int>(nearest_path_point_image.get().x)] *
    map_info.resolution;
  if (
    nearest_path_point_clearance - traj_param.center_line_width * 0.5 <
      object_clearance_from_road ||
    vel > traj_param.max_avoiding_objects_velocity_ms ||
    !arePointsInsideDriveableArea(polygon_points.points_in_image, clearance_map)) {
    return false;
  }
  return true;
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

PolygonPoints getPolygonPoints(
  const autoware_auto_perception_msgs::msg::PredictedObject & object,
  const nav_msgs::msg::MapMetaData & map_info)
{
  std::vector<geometry_msgs::msg::Point> points_in_image;
  std::vector<geometry_msgs::msg::Point> points_in_map;
  PolygonPoints polygon_points;
  if (object.shape.type == object.shape.BOUNDING_BOX) {
    polygon_points = getPolygonPointsFromBB(object, map_info);
  } else if (object.shape.type == object.shape.CYLINDER) {
    polygon_points = getPolygonPointsFromCircle(object, map_info);
  } else if (object.shape.type == object.shape.POLYGON) {
    polygon_points = getPolygonPointsFromPolygon(object, map_info);
  }
  return polygon_points;
}

PolygonPoints getPolygonPointsFromBB(
  const autoware_auto_perception_msgs::msg::PredictedObject & object,
  const nav_msgs::msg::MapMetaData & map_info)
{
  std::vector<geometry_msgs::msg::Point> points_in_image;
  std::vector<geometry_msgs::msg::Point> points_in_map;
  const double dim_x = object.shape.dimensions.x;
  const double dim_y = object.shape.dimensions.y;
  const std::vector<double> rel_x = {0.5 * dim_x, 0.5 * dim_x, -0.5 * dim_x, -0.5 * dim_x};
  const std::vector<double> rel_y = {0.5 * dim_y, -0.5 * dim_y, -0.5 * dim_y, 0.5 * dim_y};
  const geometry_msgs::msg::Pose object_pose = object.kinematics.initial_pose_with_covariance.pose;
  for (std::size_t i = 0; i < rel_x.size(); i++) {
    geometry_msgs::msg::Point rel_point;
    rel_point.x = rel_x[i];
    rel_point.y = rel_y[i];
    auto abs_point = util::transformToAbsoluteCoordinate2D(rel_point, object_pose);
    geometry_msgs::msg::Point image_point;
    if (util::transformMapToImage(abs_point, map_info, image_point)) {
      points_in_image.push_back(image_point);
      points_in_map.push_back(abs_point);
    }
  }
  PolygonPoints polygon_points;
  polygon_points.points_in_image = points_in_image;
  polygon_points.points_in_map = points_in_map;
  return polygon_points;
}

PolygonPoints getPolygonPointsFromCircle(
  const autoware_auto_perception_msgs::msg::PredictedObject & object,
  const nav_msgs::msg::MapMetaData & map_info)
{
  std::vector<geometry_msgs::msg::Point> points_in_image;
  std::vector<geometry_msgs::msg::Point> points_in_map;
  const double radius = object.shape.dimensions.x;
  const geometry_msgs::msg::Point center =
    object.kinematics.initial_pose_with_covariance.pose.position;
  constexpr int num_sampling_points = 5;
  for (int i = 0; i < num_sampling_points; ++i) {
    std::vector<double> deltas = {0, 1.0};
    for (const auto & delta : deltas) {
      geometry_msgs::msg::Point point;
      point.x = std::cos(
                  ((i + delta) / static_cast<double>(num_sampling_points)) * 2.0 * M_PI +
                  M_PI / static_cast<double>(num_sampling_points)) *
                  (radius / 2.0) +
                center.x;
      point.y = std::sin(
                  ((i + delta) / static_cast<double>(num_sampling_points)) * 2.0 * M_PI +
                  M_PI / static_cast<double>(num_sampling_points)) *
                  (radius / 2.0) +
                center.y;
      point.z = center.z;
      geometry_msgs::msg::Point image_point;
      if (util::transformMapToImage(point, map_info, image_point)) {
        points_in_image.push_back(image_point);
        points_in_map.push_back(point);
      }
    }
  }
  PolygonPoints polygon_points;
  polygon_points.points_in_image = points_in_image;
  polygon_points.points_in_map = points_in_map;
  return polygon_points;
}

PolygonPoints getPolygonPointsFromPolygon(
  const autoware_auto_perception_msgs::msg::PredictedObject & object,
  const nav_msgs::msg::MapMetaData & map_info)
{
  std::vector<geometry_msgs::msg::Point> points_in_image;
  std::vector<geometry_msgs::msg::Point> points_in_map;
  for (const auto & polygon_p : object.shape.footprint.points) {
    geometry_msgs::msg::Point rel_point;
    rel_point.x = polygon_p.x;
    rel_point.y = polygon_p.y;
    geometry_msgs::msg::Point point = util::transformToAbsoluteCoordinate2D(
      rel_point, object.kinematics.initial_pose_with_covariance.pose);
    const auto image_point = util::transformMapToOptionalImage(point, map_info);
    if (image_point) {
      points_in_image.push_back(image_point.get());
      points_in_map.push_back(point);
    }
  }
  PolygonPoints polygon_points;
  polygon_points.points_in_image = points_in_image;
  polygon_points.points_in_map = points_in_map;
  return polygon_points;
}

std::vector<cv::Point> getCVPolygon(
  const autoware_auto_perception_msgs::msg::PredictedObject & object,
  const PolygonPoints & polygon_points,
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const cv::Mat & clearance_map, const nav_msgs::msg::MapMetaData & map_info)
{
  const int nearest_idx =
    util::getNearestIdx(path_points, object.kinematics.initial_pose_with_covariance.pose.position);
  const auto nearest_path_point = path_points[nearest_idx];
  if (path_points.empty()) {
    return getDefaultCVPolygon(polygon_points.points_in_image);
  }
  return getExtendedCVPolygon(
    polygon_points.points_in_image, polygon_points.points_in_map, nearest_path_point.pose, object,
    clearance_map, map_info);
}

std::vector<cv::Point> getDefaultCVPolygon(
  const std::vector<geometry_msgs::msg::Point> & points_in_image)
{
  std::vector<cv::Point> cv_polygon;
  for (const auto & point : points_in_image) {
    cv::Point image_point = cv::Point(point.x, point.y);
    cv_polygon.push_back(image_point);
  }
  return cv_polygon;
}

std::vector<cv::Point> getExtendedCVPolygon(
  const std::vector<geometry_msgs::msg::Point> & points_in_image,
  const std::vector<geometry_msgs::msg::Point> & points_in_map,
  const geometry_msgs::msg::Pose & nearest_path_point_pose,
  const autoware_auto_perception_msgs::msg::PredictedObject & object, const cv::Mat & clearance_map,
  const nav_msgs::msg::MapMetaData & map_info)
{
  const boost::optional<Edges> optional_edges = getEdges(
    points_in_image, points_in_map, nearest_path_point_pose, object, clearance_map, map_info);
  if (!optional_edges) {
    return getDefaultCVPolygon(points_in_image);
  }
  const Edges edges = optional_edges.get();

  const int nearest_polygon_idx = util::getNearestIdx(points_in_image, edges.origin);
  std::vector<cv::Point> cv_polygon;
  if (edges.back_idx == nearest_polygon_idx || edges.front_idx == nearest_polygon_idx) {
    // make polygon only with edges and extended edges
  } else if (edges.back_idx < nearest_polygon_idx) {
    // back_idx -> nearest_idx -> frond_idx
    if (edges.back_idx < edges.front_idx && nearest_polygon_idx < edges.front_idx) {
      for (int i = edges.back_idx + 1; i < edges.front_idx; i++) {
        cv_polygon.push_back(cv::Point(points_in_image[i].x, points_in_image[i].y));
      }
      // back_idx -> vector_front -> vector_back -> nearest_idx -> frond_idx
    } else if (edges.back_idx < edges.front_idx && nearest_polygon_idx > edges.front_idx) {
      for (int i = edges.back_idx - 1; i >= 0; i--) {
        cv_polygon.push_back(cv::Point(points_in_image[i].x, points_in_image[i].y));
      }
      for (int i = points_in_image.size() - 1; i > edges.front_idx; i--) {
        cv_polygon.push_back(cv::Point(points_in_image[i].x, points_in_image[i].y));
      }
      // back_idx -> vector_back -> vector_front -> nearest_idx -> front_idx
    } else {
      for (std::size_t i = edges.back_idx + 1; i < points_in_image.size(); i++) {
        cv_polygon.push_back(cv::Point(points_in_image[i].x, points_in_image[i].y));
      }
      for (int i = 0; i < edges.front_idx; i++) {
        cv_polygon.push_back(cv::Point(points_in_image[i].x, points_in_image[i].y));
      }
    }
  } else {
    // back_idx -> nearest_idx -> front_idx
    if (edges.back_idx >= edges.front_idx && nearest_polygon_idx > edges.front_idx) {
      for (int i = edges.back_idx - 1; i > edges.front_idx; i--) {
        cv_polygon.push_back(cv::Point(points_in_image[i].x, points_in_image[i].y));
      }
      // back_idx -> vector_back -> vector_front -> nearest_idx -> front_idx
    } else {
      if (edges.back_idx >= edges.front_idx && nearest_polygon_idx < edges.front_idx) {
        for (std::size_t i = edges.back_idx + 1; i < points_in_image.size(); i++) {
          cv_polygon.push_back(cv::Point(points_in_image[i].x, points_in_image[i].y));
        }
        for (int i = 0; i < edges.front_idx; i++) {
          cv_polygon.push_back(cv::Point(points_in_image[i].x, points_in_image[i].y));
        }
      } else {  // back_idx -> vector_front -> vector_back -> nearest_idx -> front_idx
        for (int i = edges.back_idx - 1; i >= 0; i--) {
          cv_polygon.push_back(cv::Point(points_in_image[i].x, points_in_image[i].y));
        }
        for (int i = points_in_image.size() - 1; i > edges.front_idx; i--) {
          cv_polygon.push_back(cv::Point(points_in_image[i].x, points_in_image[i].y));
        }
      }
    }
  }
  cv_polygon.push_back(
    cv::Point(points_in_image[edges.front_idx].x, points_in_image[edges.front_idx].y));
  cv_polygon.push_back(cv::Point(edges.extended_front.x, edges.extended_front.y));
  cv_polygon.push_back(cv::Point(edges.extended_back.x, edges.extended_back.y));
  cv_polygon.push_back(
    cv::Point(points_in_image[edges.back_idx].x, points_in_image[edges.back_idx].y));
  return cv_polygon;
}

boost::optional<Edges> getEdges(
  const std::vector<geometry_msgs::msg::Point> & points_in_image,
  const std::vector<geometry_msgs::msg::Point> & points_in_map,
  const geometry_msgs::msg::Pose & nearest_path_point_pose,
  const autoware_auto_perception_msgs::msg::PredictedObject & object, const cv::Mat & clearance_map,
  const nav_msgs::msg::MapMetaData & map_info)
{
  // calculate perpendicular point to object along with path point orientation
  const double yaw = tf2::getYaw(nearest_path_point_pose.orientation);
  const Eigen::Vector2d rel_path_vec(std::cos(yaw), std::sin(yaw));
  const Eigen::Vector2d obj_vec(
    object.kinematics.initial_pose_with_covariance.pose.position.x,
    object.kinematics.initial_pose_with_covariance.pose.position.y);
  const double inner_product = rel_path_vec[0] * (obj_vec[0] - nearest_path_point_pose.position.x) +
                               rel_path_vec[1] * (obj_vec[1] - nearest_path_point_pose.position.y);
  geometry_msgs::msg::Point origin;
  origin.x = nearest_path_point_pose.position.x + rel_path_vec[0] * inner_product;
  origin.y = nearest_path_point_pose.position.y + rel_path_vec[1] * inner_product;
  const Eigen::Vector2d obj2origin(origin.x - obj_vec[0], origin.y - obj_vec[1]);

  // calculate origin for casting ray to edges
  const auto path_point_image =
    util::transformMapToImage(nearest_path_point_pose.position, map_info);
  constexpr double ray_origin_dist_scale = 1.0;
  const float clearance = clearance_map.ptr<float>(static_cast<int>(
                            path_point_image.y))[static_cast<int>(path_point_image.x)] *
                          map_info.resolution * ray_origin_dist_scale;
  const Eigen::Vector2d obj2ray_origin = obj2origin.normalized() * (obj2origin.norm() + clearance);
  geometry_msgs::msg::Point ray_origin;
  ray_origin.x = obj_vec[0] + obj2ray_origin[0];
  ray_origin.y = obj_vec[1] + obj2ray_origin[1];
  geometry_msgs::msg::Point ray_origin_image;
  ray_origin_image = util::transformMapToImage(ray_origin, map_info);

  double min_cos = std::numeric_limits<double>::max();
  double max_cos = std::numeric_limits<double>::lowest();
  const double path_yaw = tf2::getYaw(nearest_path_point_pose.orientation);
  const double dx1 = std::cos(path_yaw);
  const double dy1 = std::sin(path_yaw);
  const Eigen::Vector2d path_point_vec(dx1, dy1);
  const double path_point_vec_norm = path_point_vec.norm();
  Edges edges;
  for (std::size_t i = 0; i < points_in_image.size(); i++) {
    const double dx2 = points_in_map[i].x - ray_origin.x;
    const double dy2 = points_in_map[i].y - ray_origin.y;
    const Eigen::Vector2d path_point2point(dx2, dy2);
    const double inner_product = path_point_vec.dot(path_point2point);
    const double cos = inner_product / (path_point_vec_norm * path_point2point.norm());
    if (cos > max_cos) {
      max_cos = cos;
      edges.front_idx = i;
    }
    if (cos < min_cos) {
      min_cos = cos;
      edges.back_idx = i;
    }
  }

  const double max_sin = std::sqrt(1 - max_cos * max_cos);
  const double min_sin = std::sqrt(1 - min_cos * min_cos);
  const Eigen::Vector2d point2front_edge(
    points_in_image[edges.front_idx].x - ray_origin_image.x,
    points_in_image[edges.front_idx].y - ray_origin_image.y);
  const Eigen::Vector2d point2back_edge(
    points_in_image[edges.back_idx].x - ray_origin_image.x,
    points_in_image[edges.back_idx].y - ray_origin_image.y);
  const Eigen::Vector2d point2extended_front_edge =
    point2front_edge.normalized() * (clearance * 2 / max_sin) * (1 / map_info.resolution);
  const Eigen::Vector2d point2extended_back_edge =
    point2back_edge.normalized() * (clearance * 2 / min_sin) * (1 / map_info.resolution);

  const double dist2extended_front_edge = point2extended_front_edge.norm() * map_info.resolution;
  const double dist2front_edge = point2front_edge.norm() * map_info.resolution;
  const double dist2extended_back_edge = point2extended_back_edge.norm() * map_info.resolution;
  const double dist2back_edge = point2back_edge.norm() * map_info.resolution;
  if (
    dist2extended_front_edge < clearance * 2 || dist2extended_back_edge < clearance * 2 ||
    dist2front_edge > dist2extended_front_edge || dist2back_edge > dist2extended_back_edge) {
    return boost::none;
  }
  geometry_msgs::msg::Point extended_front;
  geometry_msgs::msg::Point extended_back;
  extended_front.x = point2extended_front_edge(0) + ray_origin_image.x;
  extended_front.y = point2extended_front_edge(1) + ray_origin_image.y;
  extended_back.x = point2extended_back_edge(0) + ray_origin_image.x;
  extended_back.y = point2extended_back_edge(1) + ray_origin_image.y;
  edges.extended_front = extended_front;
  edges.extended_back = extended_back;
  edges.origin = ray_origin_image;
  return edges;
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

cv::Mat getDrivableAreaInCV(const nav_msgs::msg::OccupancyGrid & occupancy_grid)
{
  cv::Mat drivable_area = cv::Mat(occupancy_grid.info.width, occupancy_grid.info.height, CV_8UC1);

  drivable_area.forEach<unsigned char>([&](unsigned char & value, const int * position) -> void {
    getOccupancyGridValue(occupancy_grid, position[0], position[1], value);
  });
  return drivable_area;
}

cv::Mat getClearanceMap(const cv::Mat & drivable_area)
{
  cv::Mat clearance_map;
  cv::distanceTransform(drivable_area, clearance_map, cv::DIST_L2, 5);
  return clearance_map;
}

cv::Mat getAreaWithObjects(const cv::Mat & drivable_area, const cv::Mat & objects_image)
{
  cv::Mat area_with_objects = cv::min(objects_image, drivable_area);
  return area_with_objects;
}

boost::optional<int> getStopIdx(
  const std::vector<util::Footprint> & footprints, const geometry_msgs::msg::Pose & ego_pose,
  const cv::Mat & road_clearance_map, const nav_msgs::msg::MapMetaData & map_info)
{
  const int nearest_idx = util::getNearestPointIdx(footprints, ego_pose.position);
  for (std::size_t i = nearest_idx; i < footprints.size(); i++) {
    const auto top_left = getDistance(road_clearance_map, footprints[i].top_left, map_info);
    const auto top_right = getDistance(road_clearance_map, footprints[i].top_right, map_info);
    const auto bottom_right = getDistance(road_clearance_map, footprints[i].bottom_right, map_info);
    const auto bottom_left = getDistance(road_clearance_map, footprints[i].bottom_left, map_info);
    const double epsilon = 1e-8;
    if (!top_left || !top_right || !bottom_left || !bottom_right) {
      continue;
    } else if (  // NOLINT
      top_left.get() < epsilon || top_right.get() < epsilon || bottom_left.get() < epsilon ||
      bottom_right.get() < epsilon) {
      return std::max(static_cast<int>(i - 1), 0);
    }
  }
  return boost::none;
}

boost::optional<double> getDistance(
  const cv::Mat & clearance_map, const geometry_msgs::msg::Point & map_point,
  const nav_msgs::msg::MapMetaData & map_info)
{
  const auto image_point = util::transformMapToOptionalImage(map_point, map_info);
  if (!image_point) {
    return boost::none;
  }
  const float clearance = clearance_map.ptr<float>(static_cast<int>(
                            image_point.get().y))[static_cast<int>(image_point.get().x)] *
                          map_info.resolution;
  return clearance;
}

CVMaps getMaps(
  const bool enable_avoidance, const autoware_auto_planning_msgs::msg::Path & path,
  const std::vector<autoware_auto_perception_msgs::msg::PredictedObject> & objects,
  const TrajectoryParam & traj_param, DebugData * debug_data)
{
  CVMaps cv_maps;
  cv_maps.drivable_area = getDrivableAreaInCV(path.drivable_area);
  cv_maps.clearance_map = getClearanceMap(cv_maps.drivable_area);

  std::vector<autoware_auto_perception_msgs::msg::PredictedObject> debug_avoiding_objects;
  cv::Mat objects_image = drawObstaclesOnImage(
    enable_avoidance, objects, path.points, path.drivable_area.info, cv_maps.clearance_map,
    traj_param, &debug_avoiding_objects);
  cv_maps.area_with_objects_map = getAreaWithObjects(cv_maps.drivable_area, objects_image);
  cv_maps.only_objects_clearance_map = getClearanceMap(objects_image);
  cv_maps.map_info = path.drivable_area.info;

  debug_data->clearance_map = cv_maps.clearance_map;
  debug_data->only_object_clearance_map = cv_maps.only_objects_clearance_map;
  debug_data->area_with_objects_map = cv_maps.area_with_objects_map;
  debug_data->avoiding_objects = debug_avoiding_objects;
  return cv_maps;
}
}  // namespace process_cv
