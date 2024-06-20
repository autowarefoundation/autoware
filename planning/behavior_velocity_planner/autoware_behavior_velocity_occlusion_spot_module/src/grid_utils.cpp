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

#include "grid_utils.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner
{
namespace grid_utils
{

Polygon2d pointsToPoly(const Point2d p0, const Point2d p1, const double radius)
{
  LineString2d line = {p0, p1};
  const double angle = atan2(p0.y() - p1.y(), p0.x() - p1.x());
  const double r = radius;
  Polygon2d line_poly;
  // add polygon counter clockwise
  line_poly.outer().emplace_back(p0.x() + r * sin(angle), p0.y() - r * cos(angle));
  line_poly.outer().emplace_back(p1.x() + r * sin(angle), p1.y() - r * cos(angle));
  line_poly.outer().emplace_back(p1.x() - r * sin(angle), p1.y() + r * cos(angle));
  line_poly.outer().emplace_back(p0.x() - r * sin(angle), p0.y() + r * cos(angle));
  // std::cout << boost::geometry::wkt(line_poly) << std::endl;
  // std::cout << boost::geometry::wkt(line) << std::endl;

  boost::geometry::correct(line_poly);
  return line_poly;
}

void findOcclusionSpots(
  std::vector<grid_map::Position> & occlusion_spot_positions, const grid_map::GridMap & grid,
  const Polygon2d & polygon, [[maybe_unused]] double min_size)
{
  const grid_map::Matrix & grid_data = grid["layer"];
  grid_map::Polygon grid_polygon;
  for (const auto & point : polygon.outer()) {
    grid_polygon.addVertex({point.x(), point.y()});
  }
  for (autoware::grid_map_utils::PolygonIterator iterator(grid, grid_polygon);
       !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index & index = *iterator;
    if (grid_data(index.x(), index.y()) == grid_utils::occlusion_cost_value::UNKNOWN) {
      grid_map::Position occlusion_spot_position;
      if (grid.getPosition(index, occlusion_spot_position)) {
        occlusion_spot_positions.emplace_back(occlusion_spot_position);
      }
    }
  }
}

bool isCollisionFree(
  const grid_map::GridMap & grid, const grid_map::Position & p1, const grid_map::Position & p2,
  const double radius)
{
  const grid_map::Matrix & grid_data = grid["layer"];
  try {
    Point2d occlusion_p = {p1.x(), p1.y()};
    Point2d collision_p = {p2.x(), p2.y()};
    Polygon2d polygon = pointsToPoly(occlusion_p, collision_p, radius);
    grid_map::Polygon grid_polygon;
    for (const auto & point : polygon.outer()) {
      grid_polygon.addVertex({point.x(), point.y()});
    }
    for (autoware::grid_map_utils::PolygonIterator iterator(grid, grid_polygon);
         !iterator.isPastEnd(); ++iterator) {
      const grid_map::Index & index = *iterator;
      if (grid_data(index.x(), index.y()) == grid_utils::occlusion_cost_value::OCCUPIED) {
        return false;
      }
    }
  } catch (const std::invalid_argument & e) {
    std::cerr << e.what() << std::endl;
    return false;
  }
  return true;
}

std::optional<Polygon2d> generateOcclusionPolygon(
  const Polygon2d & occupancy_poly, const Point2d & origin, const Point2d & min_theta_pos,
  const Point2d & max_theta_pos, const double ray_max_length = 100.0)
{
  using autoware::universe_utils::normalizeRadian;
  const double origin_x = origin.x();
  const double origin_y = origin.y();
  const double min_theta =
    normalizeRadian(std::atan2(min_theta_pos.y() - origin_y, min_theta_pos.x() - origin_x), 0.0);
  const double max_theta =
    normalizeRadian(std::atan2(max_theta_pos.y() - origin_y, max_theta_pos.x() - origin_x), 0.0);
  LineString2d theta_min_ray = {
    origin,
    {origin_x + ray_max_length * std::cos(min_theta),
     origin_y + ray_max_length * std::sin(min_theta)}};
  LineString2d theta_max_ray = {
    origin,
    {origin_x + ray_max_length * std::cos(max_theta),
     origin_y + ray_max_length * std::sin(max_theta)}};
  Polygon2d occlusion_poly;
  occlusion_poly.outer() = {min_theta_pos, max_theta_pos};
  std::vector<Point2d> min_intersections;
  std::vector<Point2d> max_intersections;
  boost::geometry::intersection(occupancy_poly, theta_min_ray, min_intersections);
  boost::geometry::intersection(occupancy_poly, theta_max_ray, max_intersections);
  if (!min_intersections.empty()) {
    // has min theta intersection
    occlusion_poly.outer().emplace_back(min_intersections.front());
  }
  if (!max_intersections.empty()) {
    // has max theta intersection
    occlusion_poly.outer().emplace_back(max_intersections.front());
  }
  //! case outside detection area
  if (occlusion_poly.outer().size() == 2) return std::nullopt;
  boost::geometry::correct(occlusion_poly);
  Polygon2d hull_poly;
  boost::geometry::convex_hull(occlusion_poly, hull_poly);
  return hull_poly;
}

Polygon2d generateOccupancyPolygon(const nav_msgs::msg::MapMetaData & info, const double r = 100)
{
  using autoware::universe_utils::calcOffsetPose;
  // generate occupancy polygon from grid origin
  Polygon2d poly;  // create counter clockwise poly
  poly.outer().emplace_back(to_bg2d(calcOffsetPose(info.origin, 0, 0, 0).position));
  poly.outer().emplace_back(to_bg2d(calcOffsetPose(info.origin, r, 0, 0).position));
  poly.outer().emplace_back(to_bg2d(calcOffsetPose(info.origin, r, r, 0).position));
  poly.outer().emplace_back(to_bg2d(calcOffsetPose(info.origin, 0, r, 0).position));

  boost::geometry::correct(poly);
  return poly;
}

std::pair<size_t, size_t> calcEdgePoint(const Polygon2d & foot_print, const Point2d & origin)
{
  size_t min_idx = 0;
  size_t max_idx = 0;
  double min_theta = std::numeric_limits<double>::max();
  double max_theta = -std::numeric_limits<double>::max();
  for (size_t i = 0; i < foot_print.outer().size(); i++) {
    const auto & f = foot_print.outer().at(i);
    PolarCoordinates polar = toPolarCoordinates(origin, f);
    const double theta_norm = autoware::universe_utils::normalizeRadian(polar.theta, 0.0);
    if (theta_norm < min_theta) {
      min_theta = theta_norm;
      min_idx = i;
    }
    if (theta_norm > max_theta) {
      max_theta = theta_norm;
      max_idx = i;
    }
  }
  return std::make_pair(min_idx, max_idx);
}

std::optional<Polygon2d> generateOccupiedPolygon(
  const Polygon2d & occupancy_poly, const Polygon2d & foot_print, const Point & position)
{
  Point2d origin = {position.x, position.y};
  const auto & edge_pair = calcEdgePoint(foot_print, origin);
  const size_t min_idx = edge_pair.first;
  const size_t max_idx = edge_pair.second;
  Polygon2d occupied_polygon;
  const auto & poly = generateOcclusionPolygon(
    occupancy_poly, origin, foot_print.outer().at(min_idx), foot_print.outer().at(max_idx));
  return poly;
}

Point transformFromMap2Grid(const TransformStamped & geom_tf_map2grid, const Point2d & p)
{
  Point geom_pt = autoware::universe_utils::createPoint(p.x(), p.y(), 0);
  Point transformed_geom_pt;
  // from map coordinate to occupancy grid coordinate
  tf2::doTransform(geom_pt, transformed_geom_pt, geom_tf_map2grid);
  return transformed_geom_pt;
}

void generateOccupiedImage(
  const OccupancyGrid & occ_grid, cv::Mat & inout_image,
  const Polygons2d & stuck_vehicle_foot_prints, const Polygons2d & moving_vehicle_foot_prints,
  const bool use_object_foot_print, const bool use_object_raycast)
{
  const auto & occ = occ_grid;
  OccupancyGrid occupancy_grid;
  PoseStamped grid_origin;
  const double width = occ.info.width * occ.info.resolution;
  const double height = occ.info.height * occ.info.resolution;
  Point scan_origin = occ.info.origin.position;
  scan_origin.x += 0.5 * width;
  scan_origin.y += 0.5 * height;

  // calculate grid origin
  {
    grid_origin.header = occ.header;
    grid_origin.pose.position.x = occ.info.origin.position.x;
    grid_origin.pose.position.y = occ.info.origin.position.y;
    grid_origin.pose.position.z = 0.0;  // same z as foot print polygon
  }

  // header
  {
    occupancy_grid.header.stamp = occ.header.stamp;
    occupancy_grid.header.frame_id = "map";
  }

  // info
  {
    occupancy_grid.info.map_load_time = occ.header.stamp;
    occupancy_grid.info.resolution = occ.info.resolution;
    occupancy_grid.info.width = occ.info.width;
    occupancy_grid.info.height = occ.info.height;
    occupancy_grid.info.origin = grid_origin.pose;
  }

  constexpr uint8_t occupied_space = occlusion_cost_value::OCCUPIED_IMAGE;
  // get transform
  tf2::Stamped<tf2::Transform> tf_grid2map;
  tf2::Stamped<tf2::Transform> tf_map2grid;
  tf2::fromMsg(grid_origin, tf_grid2map);
  tf_map2grid.setData(tf_grid2map.inverse());
  const auto geom_tf_map2grid = tf2::toMsg(tf_map2grid);

  // create not Detection Area using opencv
  std::vector<std::vector<cv::Point>> cv_polygons;
  std::vector<cv::Point> cv_polygon;
  Polygon2d occupancy_poly = generateOccupancyPolygon(occupancy_grid.info);
  if (use_object_raycast) {
    for (const auto & foot_print : moving_vehicle_foot_prints) {
      // calculate occlusion polygon from moving vehicle
      const auto polys = generateOccupiedPolygon(occupancy_poly, foot_print, scan_origin);
      if (polys == std::nullopt) continue;
      // transform to cv point and stuck it to cv polygon
      for (const auto & p : polys.value().outer()) {
        const Point transformed_geom_pt = transformFromMap2Grid(geom_tf_map2grid, p);
        cv_polygon.emplace_back(
          toCVPoint(transformed_geom_pt, width, height, occupancy_grid.info.resolution));
      }
      cv_polygons.push_back(cv_polygon);
      // clear previously added points
      cv_polygon.clear();
    }
  }
  if (use_object_foot_print) {
    for (const auto & foot_print : stuck_vehicle_foot_prints) {
      for (const auto & p : foot_print.outer()) {
        const Point transformed_geom_pt = transformFromMap2Grid(geom_tf_map2grid, p);
        cv_polygon.emplace_back(
          toCVPoint(transformed_geom_pt, width, height, occupancy_grid.info.resolution));
      }
      cv_polygons.push_back(cv_polygon);
      // clear previously added points
      cv_polygon.clear();
    }
  }
  for (const auto & p : cv_polygons) {
    // fill in occlusion area and copy to occupancy grid
    cv::fillConvexPoly(inout_image, p, cv::Scalar(occupied_space));
  }
}

cv::Point toCVPoint(
  const Point & geom_point, const double width_m, const double height_m, const double resolution)
{
  return cv::Point(
    static_cast<int>((height_m - geom_point.y) / resolution),
    static_cast<int>((width_m - geom_point.x) / resolution));
}

void imageToOccupancyGrid(const cv::Mat & cv_image, nav_msgs::msg::OccupancyGrid * occupancy_grid)
{
  const int width = cv_image.cols;
  const int height = cv_image.rows;
  occupancy_grid->data.clear();
  occupancy_grid->data.resize(width * height);
  for (int x = width - 1; x >= 0; x--) {
    for (int y = height - 1; y >= 0; y--) {
      const int idx = (height - 1 - y) + (width - 1 - x) * height;
      unsigned char intensity = cv_image.at<unsigned char>(y, x);
      if (intensity == grid_utils::occlusion_cost_value::FREE_SPACE) {
        // do nothing
      } else if (intensity == grid_utils::occlusion_cost_value::UNKNOWN_IMAGE) {
        intensity = grid_utils::occlusion_cost_value::UNKNOWN;
      } else if (intensity == grid_utils::occlusion_cost_value::OCCUPIED_IMAGE) {
        intensity = grid_utils::occlusion_cost_value::OCCUPIED;
      } else {
        throw std::logic_error("behavior_velocity[occlusion_spot_grid]: invalid if clause");
      }
      occupancy_grid->data.at(idx) = intensity;
    }
  }
}
void toQuantizedImage(
  const nav_msgs::msg::OccupancyGrid & occupancy_grid, cv::Mat * border_image,
  cv::Mat * occlusion_image, const GridParam & param)
{
  const int width = border_image->cols;
  const int height = border_image->rows;
  for (int x = width - 1; x >= 0; x--) {
    for (int y = height - 1; y >= 0; y--) {
      const int idx = (height - 1 - y) + (width - 1 - x) * height;
      unsigned char intensity = occupancy_grid.data.at(idx);
      if (intensity <= param.free_space_max) {
        continue;
      } else if (intensity < param.occupied_min) {
        intensity = grid_utils::occlusion_cost_value::UNKNOWN_IMAGE;
        occlusion_image->at<unsigned char>(y, x) = intensity;
      } else {
        intensity = grid_utils::occlusion_cost_value::OCCUPIED_IMAGE;
        border_image->at<unsigned char>(y, x) = intensity;
      }
    }
  }
}

void denoiseOccupancyGridCV(
  const OccupancyGrid::ConstSharedPtr occupancy_grid_ptr,
  const Polygons2d & stuck_vehicle_foot_prints, const Polygons2d & moving_vehicle_foot_prints,
  grid_map::GridMap & grid_map, const GridParam & param, const bool is_show_debug_window,
  const int num_iter, const bool use_object_footprints, const bool use_object_ray_casts)
{
  OccupancyGrid occupancy_grid = *occupancy_grid_ptr;
  cv::Mat border_image(
    occupancy_grid.info.width, occupancy_grid.info.height, CV_8UC1,
    cv::Scalar(grid_utils::occlusion_cost_value::FREE_SPACE));
  cv::Mat occlusion_image(
    occupancy_grid.info.width, occupancy_grid.info.height, CV_8UC1,
    cv::Scalar(grid_utils::occlusion_cost_value::FREE_SPACE));
  toQuantizedImage(occupancy_grid, &border_image, &occlusion_image, param);

  //! show original occupancy grid to compare difference
  if (is_show_debug_window) {
    cv::namedWindow("occlusion_image", cv::WINDOW_NORMAL);
    cv::imshow("occlusion_image", occlusion_image);
    cv::moveWindow("occlusion_image", 0, 0);
  }

  //! raycast object shadow using vehicle
  if (use_object_footprints || use_object_ray_casts) {
    generateOccupiedImage(
      occupancy_grid, border_image, stuck_vehicle_foot_prints, moving_vehicle_foot_prints,
      use_object_footprints, use_object_ray_casts);
    if (is_show_debug_window) {
      cv::namedWindow("object ray shadow", cv::WINDOW_NORMAL);
      cv::imshow("object ray shadow", border_image);
      cv::moveWindow("object ray shadow", 300, 0);
    }
  }

  //!< @brief erode occlusion to make sure occlusion candidates are big enough
  cv::Mat kernel(2, 2, CV_8UC1, cv::Scalar(1));
  cv::erode(occlusion_image, occlusion_image, kernel, cv::Point(-1, -1), num_iter);
  if (is_show_debug_window) {
    cv::namedWindow("morph", cv::WINDOW_NORMAL);
    cv::imshow("morph", occlusion_image);
    cv::moveWindow("morph", 0, 300);
  }

  border_image += occlusion_image;
  if (is_show_debug_window) {
    cv::namedWindow("merge", cv::WINDOW_NORMAL);
    cv::imshow("merge", border_image);
    cv::moveWindow("merge", 300, 300);
    cv::waitKey(1);
  }
  imageToOccupancyGrid(border_image, &occupancy_grid);
  grid_map::GridMapRosConverter::fromOccupancyGrid(occupancy_grid, "layer", grid_map);
}
}  // namespace grid_utils
}  // namespace autoware::behavior_velocity_planner
