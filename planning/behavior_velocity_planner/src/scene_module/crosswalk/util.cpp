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

#include <scene_module/crosswalk/util.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>
#include <utilization/util.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>

#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <lanelet2_extension/regulatory_elements/road_marking.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

namespace behavior_velocity_planner
{

namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point>;
using Line = bg::model::linestring<Point>;
using motion_utils::calcSignedArcLength;
using tier4_autoware_utils::createPoint;

std::vector<Point> getPolygonIntersects(
  const PathWithLaneId & ego_path, const lanelet::BasicPolygon2d & polygon,
  const geometry_msgs::msg::Point & ego_pos,
  const size_t max_num = std::numeric_limits<size_t>::max())
{
  std::vector<Point> intersects{};

  bool found_max_num = false;
  for (size_t i = 0; i < ego_path.points.size() - 1; ++i) {
    const auto & p_back = ego_path.points.at(i).point.pose.position;
    const auto & p_front = ego_path.points.at(i + 1).point.pose.position;
    const Line segment{{p_back.x, p_back.y}, {p_front.x, p_front.y}};

    std::vector<Point> tmp_intersects{};
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

  const auto compare = [&](const Point & p1, const Point & p2) {
    const auto dist_l1 =
      calcSignedArcLength(ego_path.points, size_t(0), createPoint(p1.x(), p1.y(), ego_pos.z));

    const auto dist_l2 =
      calcSignedArcLength(ego_path.points, size_t(0), createPoint(p2.x(), p2.y(), ego_pos.z));

    return dist_l1 < dist_l2;
  };

  std::sort(intersects.begin(), intersects.end(), compare);

  return intersects;
}

std::vector<Point> getLinestringIntersects(
  const PathWithLaneId & ego_path, const lanelet::BasicLineString2d & linestring,
  const geometry_msgs::msg::Point & ego_pos,
  const size_t max_num = std::numeric_limits<size_t>::max())
{
  std::vector<Point> intersects{};

  bool found_max_num = false;
  for (size_t i = 0; i < ego_path.points.size() - 1; ++i) {
    const auto & p_back = ego_path.points.at(i).point.pose.position;
    const auto & p_front = ego_path.points.at(i + 1).point.pose.position;
    const Line segment{{p_back.x, p_back.y}, {p_front.x, p_front.y}};

    std::vector<Point> tmp_intersects{};
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

  const auto compare = [&](const Point & p1, const Point & p2) {
    const auto dist_l1 =
      calcSignedArcLength(ego_path.points, size_t(0), createPoint(p1.x(), p1.y(), ego_pos.z));

    const auto dist_l2 =
      calcSignedArcLength(ego_path.points, size_t(0), createPoint(p2.x(), p2.y(), ego_pos.z));

    return dist_l1 < dist_l2;
  };

  std::sort(intersects.begin(), intersects.end(), compare);

  return intersects;
}

lanelet::Optional<lanelet::ConstLineString3d> getStopLineFromMap(
  const int lane_id, const std::shared_ptr<const PlannerData> & planner_data,
  const std::string & attribute_name)
{
  lanelet::ConstLanelet lanelet =
    planner_data->route_handler_->getLaneletMapPtr()->laneletLayer.get(lane_id);
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
