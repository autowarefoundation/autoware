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

#include <scene_module/occlusion_spot/grid_utils.hpp>

#include <algorithm>
#include <stdexcept>
#include <vector>

namespace behavior_velocity_planner
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
  return line_poly;
}

std::vector<std::pair<grid_map::Position, grid_map::Position>> pointsToRays(
  const grid_map::Position p0, const grid_map::Position p1, const double radius)
{
  using grid_map::Position;
  std::vector<std::pair<Position, Position>> lines;
  const double angle = atan2(p0.y() - p1.y(), p0.x() - p1.x());
  const double r = radius;
  lines.emplace_back(std::make_pair(p0, p1));
  lines.emplace_back(std::make_pair(
    Position(p0.x() + r * sin(angle), p0.y() - r * cos(angle)),
    Position(p1.x() + r * sin(angle), p1.y() - r * cos(angle))));
  lines.emplace_back(std::make_pair(
    Position(p1.x() - r * sin(angle), p1.y() + r * cos(angle)),
    Position(p0.x() - r * sin(angle), p0.y() + r * cos(angle))));
  return lines;
}

void addObjectsToGridMap(const std::vector<PredictedObject> & objs, grid_map::GridMap & grid)
{
  auto & grid_data = grid["layer"];
  for (const auto & obj : objs) {
    Polygon2d foot_print_polygon = planning_utils::toFootprintPolygon(obj);
    grid_map::Polygon grid_polygon;
    const auto & pos = obj.kinematics.initial_pose_with_covariance.pose.position;
    if (grid.isInside(grid_map::Position(pos.x, pos.y))) continue;
    try {
      for (const auto & point : foot_print_polygon.outer()) {
        grid_polygon.addVertex({point.x(), point.y()});
      }
      for (grid_map::PolygonIterator iterator(grid, grid_polygon); !iterator.isPastEnd();
           ++iterator) {
        const grid_map::Index & index = *iterator;
        if (!grid.isValid(index)) continue;
        grid_data(index.x(), index.y()) = grid_utils::occlusion_cost_value::OCCUPIED;
      }
    } catch (const std::invalid_argument & e) {
      std::cerr << e.what() << std::endl;
    }
  }
}

bool isOcclusionSpotSquare(
  OcclusionSpotSquare & occlusion_spot, const grid_map::Matrix & grid_data,
  const grid_map::Index & cell, int min_occlusion_size, const grid_map::Size & grid_size)
{
  const int offset = (min_occlusion_size != 1) ? (min_occlusion_size - 1) : min_occlusion_size;
  const int cell_max_x = grid_size.x() - 1;
  const int cell_max_y = grid_size.y() - 1;
  // Calculate ranges to check
  int min_x = cell.x() - offset;
  int max_x = cell.x() + offset;
  int min_y = cell.y() - offset;
  int max_y = cell.y() + offset;
  if (min_x < 0) max_x += std::abs(min_x);
  if (max_x > cell_max_x) min_x -= std::abs(max_x - cell_max_x);
  if (min_y < 0) max_y += std::abs(min_y);
  if (max_y > cell_max_y) min_y -= std::abs(max_y - cell_max_y);
  // No occlusion_spot with size 0
  if (min_occlusion_size == 0) {
    return false;
  }
  /**
   * @brief
   *   (min_x,min_y)...(max_x,min_y)
   *        .               .
   *   (min_x,max_y)...(max_x,max_y)
   */
  // Ensure we stay inside the grid
  min_x = std::max(0, min_x);
  max_x = std::min(cell_max_x, max_x);
  min_y = std::max(0, min_y);
  max_y = std::min(cell_max_y, max_y);
  int not_unknown_count = 0;
  if (grid_data(cell.x(), cell.y()) != grid_utils::occlusion_cost_value::UNKNOWN) {
    return false;
  }
  for (int x = min_x; x <= max_x; ++x) {
    for (int y = min_y; y <= max_y; ++y) {
      // if the value is not unknown value return false
      if (grid_data(x, y) != grid_utils::occlusion_cost_value::UNKNOWN) {
        not_unknown_count++;
      }
      /**
       * @brief case pass o: unknown x: freespace or occupied
       *   oxx oxo oox xxx oxo oxo
       *   oox oxx oox ooo oox oxo ... etc
       *   ooo ooo oox ooo xoo oxo
       */
      if (not_unknown_count > min_occlusion_size + 1) return false;
    }
  }
  occlusion_spot.min_occlusion_size = min_occlusion_size;
  occlusion_spot.index = cell;
  return true;
}

void findOcclusionSpots(
  std::vector<grid_map::Position> & occlusion_spot_positions, const grid_map::GridMap & grid,
  const Polygon2d & polygon, double min_size)
{
  const grid_map::Matrix & grid_data = grid["layer"];
  const int min_occlusion_spot_size = std::max(0.0, std::floor(min_size / grid.getResolution()));
  grid_map::Polygon grid_polygon;
  for (const auto & point : polygon.outer()) {
    grid_polygon.addVertex({point.x(), point.y()});
  }
  for (grid_map::PolygonIterator iterator(grid, grid_polygon); !iterator.isPastEnd(); ++iterator) {
    OcclusionSpotSquare occlusion_spot_square;
    if (isOcclusionSpotSquare(
          occlusion_spot_square, grid_data, *iterator, min_occlusion_spot_size, grid.getSize())) {
      if (grid.getPosition(occlusion_spot_square.index, occlusion_spot_square.position)) {
        occlusion_spot_positions.emplace_back(occlusion_spot_square.position);
      }
    }
  }
}

bool isCollisionFree(
  const grid_map::GridMap & grid, const grid_map::Position & p1, const grid_map::Position & p2,
  const double radius)
{
  const grid_map::Matrix & grid_data = grid["layer"];
  bool polys = true;
  try {
    if (polys) {
      Point2d occlusion_p = {p1.x(), p1.y()};
      Point2d collision_p = {p2.x(), p2.y()};
      Polygon2d polygon = pointsToPoly(occlusion_p, collision_p, radius);
      grid_map::Polygon grid_polygon;
      for (const auto & point : polygon.outer()) {
        grid_polygon.addVertex({point.x(), point.y()});
      }
      for (grid_map::PolygonIterator iterator(grid, grid_polygon); !iterator.isPastEnd();
           ++iterator) {
        const grid_map::Index & index = *iterator;
        if (grid_data(index.x(), index.y()) == grid_utils::occlusion_cost_value::OCCUPIED) {
          return false;
        }
      }
    } else {
      std::vector<std::pair<grid_map::Position, grid_map::Position>> lines =
        pointsToRays(p1, p2, radius);
      for (const auto & p : lines) {
        for (grid_map::LineIterator iterator(grid, p.first, p.second); !iterator.isPastEnd();
             ++iterator) {
          const grid_map::Index & index = *iterator;
          if (grid_data(index.x(), index.y()) == grid_utils::occlusion_cost_value::OCCUPIED) {
            return false;
          }
        }
      }
    }  // polys or not
  } catch (const std::invalid_argument & e) {
    std::cerr << e.what() << std::endl;
    return false;
  }
  return true;
}

void getCornerPositions(
  std::vector<grid_map::Position> & corner_positions, const grid_map::GridMap & grid,
  const OcclusionSpotSquare & occlusion_spot_square)
{
  // Special case with size = 1: only one cell
  if (occlusion_spot_square.min_occlusion_size == 1) {
    corner_positions.emplace_back(occlusion_spot_square.position);
    return;
  }
  std::vector<grid_map::Index> corner_indexes;
  const int offset = (occlusion_spot_square.min_occlusion_size - 1) / 2;
  /**
   * @brief relation of each grid position
   *    bl br
   *    tl tr
   */
  corner_indexes = {// bl
                    grid_map::Index(
                      std::max(0, occlusion_spot_square.index.x() - offset),
                      std::max(0, occlusion_spot_square.index.y() - offset)),
                    // br
                    grid_map::Index(
                      std::min(grid.getSize().x() - 1, occlusion_spot_square.index.x() + offset),
                      std::max(0, occlusion_spot_square.index.y() - offset)),
                    // tl
                    grid_map::Index(
                      std::max(0, occlusion_spot_square.index.x() - offset),
                      std::min(grid.getSize().y() - 1, occlusion_spot_square.index.y() + offset)),
                    // tr
                    grid_map::Index(
                      std::min(grid.getSize().x() - 1, occlusion_spot_square.index.x() + offset),
                      std::min(grid.getSize().y() - 1, occlusion_spot_square.index.y() + offset))};
  for (const grid_map::Index & corner_index : corner_indexes) {
    grid_map::Position corner_position;
    grid.getPosition(corner_index, corner_position);
    corner_positions.emplace_back(corner_position);
  }
}

boost::optional<Polygon2d> generateOcclusionPolygon(
  const Polygon2d & occupancy_poly, const Point2d & origin, const Point2d & min_theta_pos,
  const Point2d & max_theta_pos, const double ray_max_length = 100.0)
{
  using tier4_autoware_utils::normalizeRadian;
  const double origin_x = origin.x();
  const double origin_y = origin.y();
  // TODO(tanaka): consider this later
  const double delay_angle = M_PI / 6.0;
  const double min_theta =
    normalizeRadian(std::atan2(min_theta_pos.y() - origin_y, min_theta_pos.x() - origin_x), 0.0) -
    delay_angle;
  const double max_theta =
    normalizeRadian(std::atan2(max_theta_pos.y() - origin_y, max_theta_pos.x() - origin_x), 0.0) +
    delay_angle;
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
  if (occlusion_poly.outer().size() == 2) return boost::none;
  boost::geometry::correct(occlusion_poly);
  Polygon2d hull_poly;
  boost::geometry::convex_hull(occlusion_poly, hull_poly);
  return hull_poly;
}

Polygon2d generateOccupancyPolygon(const nav_msgs::msg::MapMetaData & info, const double r = 100)
{
  using tier4_autoware_utils::calcOffsetPose;
  // generate occupancy polygon from grid origin
  Polygon2d poly;  // create counter clockwise poly
  poly.outer().emplace_back(to_bg2d(calcOffsetPose(info.origin, 0, 0, 0).position));
  poly.outer().emplace_back(to_bg2d(calcOffsetPose(info.origin, r, 0, 0).position));
  poly.outer().emplace_back(to_bg2d(calcOffsetPose(info.origin, r, r, 0).position));
  poly.outer().emplace_back(to_bg2d(calcOffsetPose(info.origin, 0, r, 0).position));
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
    const double theta_norm = tier4_autoware_utils::normalizeRadian(polar.theta, 0.0);
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

boost::optional<Polygon2d> generateOccupiedPolygon(
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
  Point geom_pt = tier4_autoware_utils::createPoint(p.x(), p.y(), 0);
  Point transformed_geom_pt;
  // from map coordinate to occupancy grid corrdinate
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
      if (polys == boost::none) continue;
      // transform to cv point and stuck it to cv polygon
      for (const auto & p : polys.get().outer()) {
        const Point transformed_geom_pt = transformFromMap2Grid(geom_tf_map2grid, p);
        cv_polygon.emplace_back(
          toCVPoint(transformed_geom_pt, width, height, occupancy_grid.info.resolution));
      }
      cv_polygons.push_back(cv_polygon);
      // clear previously addeed points
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
      // clear previously addeed points
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
        intensity = grid_utils::occlusion_cost_value::FREE_SPACE;
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
  const nav_msgs::msg::OccupancyGrid & occupancy_grid, cv::Mat * cv_image, const GridParam & param)
{
  const int width = cv_image->cols;
  const int height = cv_image->rows;
  for (int x = width - 1; x >= 0; x--) {
    for (int y = height - 1; y >= 0; y--) {
      const int idx = (height - 1 - y) + (width - 1 - x) * height;
      unsigned char intensity = occupancy_grid.data.at(idx);
      if (intensity <= param.free_space_max) {
        intensity = grid_utils::occlusion_cost_value::FREE_SPACE;
      } else if (param.free_space_max < intensity && intensity < param.occupied_min) {
        intensity = grid_utils::occlusion_cost_value::UNKNOWN_IMAGE;
      } else if (param.occupied_min <= intensity) {
        intensity = grid_utils::occlusion_cost_value::OCCUPIED_IMAGE;
      } else {
        throw std::logic_error("behavior_velocity[occlusion_spot_grid]: invalid if clause");
      }
      cv_image->at<unsigned char>(y, x) = intensity;
    }
  }
}

void denoiseOccupancyGridCV(
  const OccupancyGrid::ConstSharedPtr occupancy_grid_ptr,
  const Polygons2d & stuck_vehicle_foot_prints, const Polygons2d & moving_vehicle_foot_prints,
  grid_map::GridMap & grid_map, const GridParam & param, const bool is_show_debug_window,
  const bool filter_occupancy_grid, const bool use_object_footprints,
  const bool use_object_ray_casts)
{
  OccupancyGrid occupancy_grid = *occupancy_grid_ptr;
  cv::Mat cv_image(
    occupancy_grid.info.width, occupancy_grid.info.height, CV_8UC1,
    cv::Scalar(grid_utils::occlusion_cost_value::OCCUPIED));
  toQuantizedImage(occupancy_grid, &cv_image, param);

  //! show orignal occupancy grid to compare difference
  if (is_show_debug_window) {
    cv::namedWindow("original", cv::WINDOW_NORMAL);
    cv::imshow("original", cv_image);
    cv::waitKey(1);
  }

  //! raycast object shadow using vehicle
  if (use_object_footprints || use_object_ray_casts) {
    generateOccupiedImage(
      occupancy_grid, cv_image, stuck_vehicle_foot_prints, moving_vehicle_foot_prints,
      use_object_footprints, use_object_ray_casts);
    if (is_show_debug_window) {
      cv::namedWindow("object ray shadow", cv::WINDOW_NORMAL);
      cv::imshow("object ray shadow", cv_image);
      cv::waitKey(1);
    }
  }

  //!< @brief opening & closing to remove noise in occupancy grid
  if (filter_occupancy_grid) {
    constexpr int num_iter = 2;
    cv::morphologyEx(cv_image, cv_image, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1, -1), num_iter);
    if (is_show_debug_window) {
      cv::namedWindow("morph", cv::WINDOW_NORMAL);
      cv::imshow("morph", cv_image);
      cv::waitKey(1);
    }
  }
  imageToOccupancyGrid(cv_image, &occupancy_grid);
  grid_map::GridMapRosConverter::fromOccupancyGrid(occupancy_grid, "layer", grid_map);
}
}  // namespace grid_utils
}  // namespace behavior_velocity_planner
