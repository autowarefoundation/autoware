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

#include "behavior_velocity_crosswalk_module/util.hpp"

#include <behavior_velocity_planner_common/utilization/util.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <motion_utils/trajectory/path_with_lane_id.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/geometry/boost_geometry.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>

#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <lanelet2_routing/RoutingGraphContainer.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <lanelet2_extension/regulatory_elements/road_marking.hpp>

#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;
using motion_utils::calcSignedArcLength;
using tier4_autoware_utils::createPoint;
using tier4_autoware_utils::Line2d;
using tier4_autoware_utils::Point2d;

std::vector<std::pair<int64_t, lanelet::ConstLanelet>> getCrosswalksOnPath(
  const geometry_msgs::msg::Pose & current_pose,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const lanelet::LaneletMapPtr lanelet_map,
  const std::shared_ptr<const lanelet::routing::RoutingGraphContainer> & overall_graphs)
{
  std::vector<std::pair<lanelet::Id, lanelet::ConstLanelet>> crosswalks;

  // Add current lane id
  const auto nearest_lane_id =
    behavior_velocity_planner::planning_utils::getNearestLaneId(path, lanelet_map, current_pose);

  std::vector<lanelet::Id> unique_lane_ids;
  if (nearest_lane_id) {
    // Add subsequent lane_ids from nearest lane_id
    unique_lane_ids = behavior_velocity_planner::planning_utils::getSubsequentLaneIdsSetOnPath(
      path, *nearest_lane_id);
  } else {
    // Add all lane_ids in path
    unique_lane_ids = behavior_velocity_planner::planning_utils::getSortedLaneIdsFromPath(path);
  }

  for (const auto lane_id : unique_lane_ids) {
    const auto ll = lanelet_map->laneletLayer.get(lane_id);

    constexpr int PEDESTRIAN_GRAPH_ID = 1;
    const auto conflicting_crosswalks = overall_graphs->conflictingInGraph(ll, PEDESTRIAN_GRAPH_ID);
    for (const auto & crosswalk : conflicting_crosswalks) {
      crosswalks.emplace_back(lane_id, crosswalk);
    }
  }

  return crosswalks;
}

std::set<lanelet::Id> getCrosswalkIdSetOnPath(
  const geometry_msgs::msg::Pose & current_pose,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const lanelet::LaneletMapPtr lanelet_map,
  const std::shared_ptr<const lanelet::routing::RoutingGraphContainer> & overall_graphs)
{
  std::set<lanelet::Id> crosswalk_id_set;

  for (const auto & crosswalk :
       getCrosswalksOnPath(current_pose, path, lanelet_map, overall_graphs)) {
    crosswalk_id_set.insert(crosswalk.second.id());
  }

  return crosswalk_id_set;
}

bool checkRegulatoryElementExistence(const lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  const auto all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr);
  return !lanelet::utils::query::crosswalks(all_lanelets).empty();
}

std::vector<geometry_msgs::msg::Point> getPolygonIntersects(
  const PathWithLaneId & ego_path, const lanelet::BasicPolygon2d & polygon,
  const geometry_msgs::msg::Point & ego_pos,
  const size_t max_num = std::numeric_limits<size_t>::max())
{
  std::vector<Point2d> intersects{};

  bool found_max_num = false;
  for (size_t i = 0; i < ego_path.points.size() - 1; ++i) {
    const auto & p_back = ego_path.points.at(i).point.pose.position;
    const auto & p_front = ego_path.points.at(i + 1).point.pose.position;
    const Line2d segment{{p_back.x, p_back.y}, {p_front.x, p_front.y}};

    std::vector<Point2d> tmp_intersects{};
    bg::intersection(segment, polygon, tmp_intersects);

    for (const auto & p : tmp_intersects) {
      intersects.push_back(p);
      if (intersects.size() == max_num) {
        found_max_num = true;
        break;
      }
    }

    if (found_max_num) {
      break;
    }
  }

  const auto compare = [&](const Point2d & p1, const Point2d & p2) {
    const auto dist_l1 =
      calcSignedArcLength(ego_path.points, size_t(0), createPoint(p1.x(), p1.y(), ego_pos.z));

    const auto dist_l2 =
      calcSignedArcLength(ego_path.points, size_t(0), createPoint(p2.x(), p2.y(), ego_pos.z));

    return dist_l1 < dist_l2;
  };

  std::sort(intersects.begin(), intersects.end(), compare);

  // convert tier4_autoware_utils::Point2d to geometry::msg::Point
  std::vector<geometry_msgs::msg::Point> geometry_points;
  for (const auto & p : intersects) {
    geometry_points.push_back(createPoint(p.x(), p.y(), ego_pos.z));
  }
  return geometry_points;
}

std::vector<geometry_msgs::msg::Point> getLinestringIntersects(
  const PathWithLaneId & ego_path, const lanelet::BasicLineString2d & linestring,
  const geometry_msgs::msg::Point & ego_pos,
  const size_t max_num = std::numeric_limits<size_t>::max())
{
  std::vector<Point2d> intersects{};

  bool found_max_num = false;
  for (size_t i = 0; i < ego_path.points.size() - 1; ++i) {
    const auto & p_back = ego_path.points.at(i).point.pose.position;
    const auto & p_front = ego_path.points.at(i + 1).point.pose.position;
    const Line2d segment{{p_back.x, p_back.y}, {p_front.x, p_front.y}};

    std::vector<Point2d> tmp_intersects{};
    bg::intersection(segment, linestring, tmp_intersects);

    for (const auto & p : tmp_intersects) {
      intersects.push_back(p);
      if (intersects.size() == max_num) {
        found_max_num = true;
        break;
      }
    }

    if (found_max_num) {
      break;
    }
  }

  const auto compare = [&](const Point2d & p1, const Point2d & p2) {
    const auto dist_l1 =
      calcSignedArcLength(ego_path.points, size_t(0), createPoint(p1.x(), p1.y(), ego_pos.z));

    const auto dist_l2 =
      calcSignedArcLength(ego_path.points, size_t(0), createPoint(p2.x(), p2.y(), ego_pos.z));

    return dist_l1 < dist_l2;
  };

  std::sort(intersects.begin(), intersects.end(), compare);

  // convert tier4_autoware_utils::Point2d to geometry::msg::Point
  std::vector<geometry_msgs::msg::Point> geometry_points;
  for (const auto & p : intersects) {
    geometry_points.push_back(createPoint(p.x(), p.y(), ego_pos.z));
  }
  return geometry_points;
}

std::optional<lanelet::ConstLineString3d> getStopLineFromMap(
  const lanelet::Id lane_id, const lanelet::LaneletMapPtr & lanelet_map_ptr,
  const std::string & attribute_name)
{
  lanelet::ConstLanelet lanelet = lanelet_map_ptr->laneletLayer.get(lane_id);
  const auto road_markings = lanelet.regulatoryElementsAs<lanelet::autoware::RoadMarking>();
  lanelet::ConstLineStrings3d stop_line;
  for (const auto & road_marking : road_markings) {
    // TODO(someone): Create regulatory element for crosswalk
    const std::string type =
      road_marking->roadMarking().attributeOr(lanelet::AttributeName::Type, "none");
    const int target_id = road_marking->roadMarking().attributeOr(attribute_name, 0);
    if (type == lanelet::AttributeValueString::StopLine && target_id == lane_id) {
      stop_line.push_back(road_marking->roadMarking());
      break;  // only one stop_line exists.
    }
  }
  if (stop_line.empty()) {
    return {};
  }

  return stop_line.front();
}
}  // namespace behavior_velocity_planner
