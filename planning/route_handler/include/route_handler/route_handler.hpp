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

#ifndef ROUTE_HANDLER__ROUTE_HANDLER_HPP_
#define ROUTE_HANDLER__ROUTE_HANDLER_HPP_

#include <lanelet2_extension/utility/query.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_mapping_msgs/msg/had_map_segment.hpp>
#include <autoware_auto_mapping_msgs/msg/map_primitive.hpp>
#include <autoware_auto_planning_msgs/msg/had_map_route.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <limits>
#include <memory>
#include <vector>

namespace route_handler
{
using autoware_auto_mapping_msgs::msg::HADMapBin;
using autoware_auto_mapping_msgs::msg::HADMapSegment;
using autoware_auto_planning_msgs::msg::HADMapRoute;
using autoware_auto_planning_msgs::msg::Path;
using autoware_auto_planning_msgs::msg::PathPointWithLaneId;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using std_msgs::msg::Header;

enum class LaneChangeDirection { NONE, LEFT, RIGHT };
enum class PullOverDirection { NONE, LEFT, RIGHT };
enum class PullOutDirection { NONE, LEFT, RIGHT };

class RouteHandler
{
public:
  RouteHandler() = default;
  explicit RouteHandler(const HADMapBin & map_msg);

  // non-const methods
  void setMap(const HADMapBin & map_msg);
  void setRoute(const HADMapRoute & route_msg);
  void setRouteLanelets(const lanelet::ConstLanelets & path_lanelets);
  void setPullOverGoalPose(
    const lanelet::ConstLanelet target_lane, const double vehicle_width, const double margin);

  // const methods

  // for route handler status
  bool isHandlerReady() const;
  lanelet::ConstPolygon3d getExtraDrivableAreaById(const lanelet::Id id) const;
  Header getRouteHeader() const;
  lanelet::routing::RoutingGraphContainer getOverallGraph() const;

  // for routing
  bool planPathLaneletsBetweenCheckpoints(
    const Pose & start_checkpoint, const Pose & goal_checkpoint,
    lanelet::ConstLanelets * path_lanelets) const;
  std::vector<HADMapSegment> createMapSegments(const lanelet::ConstLanelets & path_lanelets) const;

  // for goal
  bool isInGoalRouteSection(const lanelet::ConstLanelet & lanelet) const;
  Pose getGoalPose() const;
  Pose getPullOverGoalPose() const;
  lanelet::Id getGoalLaneId() const;
  bool getGoalLanelet(lanelet::ConstLanelet * goal_lanelet) const;
  std::vector<lanelet::ConstLanelet> getLanesAfterGoal(const double vehicle_length) const;

  // for lanelet
  bool getPreviousLaneletWithinRoute(
    const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * prev_lanelet) const;
  bool isDeadEndLanelet(const lanelet::ConstLanelet & lanelet) const;
  lanelet::ConstLanelets getLaneletsFromPoint(const lanelet::ConstPoint3d & point) const;

  /**
   * @brief Check if same-direction lane is available at the right side of the lanelet
   * Searches for any lanes regardless of whether it is lane-changeable or not.
   * Required the linestring to be shared(same line ID) between the lanelets.
   * @param the lanelet of interest
   * @return vector of lanelet having same direction if true
   */
  boost::optional<lanelet::ConstLanelet> getRightLanelet(
    const lanelet::ConstLanelet & lanelet) const;

  /**
   * @brief Check if same-direction lane is available at the left side of the lanelet
   * Searches for any lanes regardless of whether it is lane-changeable or not.
   * Required the linestring to be shared(same line ID) between the lanelets.
   * @param the lanelet of interest
   * @return vector of lanelet having same direction if true
   */
  boost::optional<lanelet::ConstLanelet> getLeftLanelet(
    const lanelet::ConstLanelet & lanelet) const;
  lanelet::ConstLanelets getNextLanelets(const lanelet::ConstLanelet & lanelet) const;

  /**
   * @brief Check if opposite-direction lane is available at the right side of the lanelet
   * Required the linestring to be shared(same line ID) between the lanelets.
   * @param the lanelet of interest
   * @return vector of lanelet with opposite direction if true
   */
  lanelet::Lanelets getRightOppositeLanelets(const lanelet::ConstLanelet & lanelet) const;

  /**
   * @brief Check if opposite-direction lane is available at the left side of the lanelet
   * Required the linestring to be shared between(same line ID) the lanelets.
   * @param the lanelet of interest
   * @return vector of lanelet with opposite direction if true
   */
  lanelet::Lanelets getLeftOppositeLanelets(const lanelet::ConstLanelet & lanelet) const;

  /**
   * @brief Searches the furthest linestring to the right side of the lanelet
   * Only lanelet with same direction is considered
   * @param the lanelet of interest
   * @return right most linestring of the lane with same direction
   */
  lanelet::ConstLineString3d getRightMostSameDirectionLinestring(
    const lanelet::ConstLanelet & lanelet) const noexcept;

  /**
   * @brief Searches the furthest linestring to the right side of the lanelet
   * Used to search for road shoulders. Lane direction is ignored
   * @param the lanelet of interest
   * @return right most linestring
   */
  lanelet::ConstLineString3d getRightMostLinestring(
    const lanelet::ConstLanelet & lanelet) const noexcept;

  /**
   * @brief Searches the furthest linestring to the left side of the lanelet
   * Only lanelet with same direction is considered
   * @param the lanelet of interest
   * @return left most linestring of the lane with same direction
   */
  lanelet::ConstLineString3d getLeftMostSameDirectionLinestring(
    const lanelet::ConstLanelet & lanelet) const noexcept;

  /**
   * @brief Searches the furthest linestring to the left side of the lanelet
   * Used to search for road shoulders. Lane direction is ignored
   * @param the lanelet of interest
   * @return left most linestring
   */
  lanelet::ConstLineString3d getLeftMostLinestring(
    const lanelet::ConstLanelet & lanelet) const noexcept;
  int getNumLaneToPreferredLane(const lanelet::ConstLanelet & lanelet) const;
  bool getClosestLaneletWithinRoute(
    const Pose & search_pose, lanelet::ConstLanelet * closest_lanelet) const;
  lanelet::ConstLanelet getLaneletsFromId(const lanelet::Id id) const;
  lanelet::ConstLanelets getLaneletsFromIds(const lanelet::Ids ids) const;
  lanelet::ConstLanelets getLaneletSequence(
    const lanelet::ConstLanelet & lanelet, const Pose & current_pose,
    const double backward_distance, const double forward_distance) const;
  lanelet::ConstLanelets getLaneletSequence(
    const lanelet::ConstLanelet & lanelet,
    const double backward_distance = std::numeric_limits<double>::max(),
    const double forward_distance = std::numeric_limits<double>::max()) const;
  lanelet::ConstLanelets getShoulderLaneletSequence(
    const lanelet::ConstLanelet & lanelet, const Pose & current_pose,
    const double backward_distance = std::numeric_limits<double>::max(),
    const double forward_distance = std::numeric_limits<double>::max()) const;
  lanelet::ConstLanelets getCheckTargetLanesFromPath(
    const PathWithLaneId & path, const lanelet::ConstLanelets & target_lanes,
    const double check_length) const;
  lanelet::routing::RelationType getRelation(
    const lanelet::ConstLanelet & prev_lane, const lanelet::ConstLanelet & next_lane) const;
  lanelet::ConstLanelets getShoulderLanelets() const;

  // for path
  PathWithLaneId getCenterLinePath(
    const lanelet::ConstLanelets & lanelet_sequence, const double s_start, const double s_end,
    bool use_exact = true) const;
  bool getLaneChangeTarget(
    const lanelet::ConstLanelets & lanelets, lanelet::ConstLanelet * target_lanelet) const;
  bool getPullOverTarget(
    const lanelet::ConstLanelets & lanelets, lanelet::ConstLanelet * target_lanelet) const;
  bool getPullOutStart(
    const lanelet::ConstLanelets & lanelets, lanelet::ConstLanelet * target_lanelet,
    const Pose & pose, const double vehicle_width) const;
  double getLaneChangeableDistance(
    const Pose & current_pose, const LaneChangeDirection & direction) const;
  lanelet::ConstPolygon3d getIntersectionAreaById(const lanelet::Id id) const;

private:
  // MUST
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;
  std::shared_ptr<const lanelet::routing::RoutingGraphContainer> overall_graphs_ptr_;
  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::ConstLanelets road_lanelets_;
  lanelet::ConstLanelets route_lanelets_;
  lanelet::ConstLanelets preferred_lanelets_;
  lanelet::ConstLanelets start_lanelets_;
  lanelet::ConstLanelets goal_lanelets_;
  lanelet::ConstLanelets shoulder_lanelets_;
  Pose pull_over_goal_pose_;
  HADMapRoute route_msg_;

  rclcpp::Logger logger_{rclcpp::get_logger("route_handler")};

  bool is_route_msg_ready_{false};
  bool is_map_msg_ready_{false};
  bool is_handler_ready_{false};

  // non-const methods
  void setLaneletsFromRouteMsg();

  // const methods
  // for routing
  lanelet::ConstLanelets getMainLanelets(const lanelet::ConstLanelets & path_lanelets) const;

  // for lanelet
  bool isInTargetLane(const PoseStamped & pose, const lanelet::ConstLanelets & target) const;
  bool isInPreferredLane(const PoseStamped & pose) const;
  bool isBijectiveConnection(
    const lanelet::ConstLanelets & lanelet_section1,
    const lanelet::ConstLanelets & lanelet_section2) const;
  bool getNextLaneletWithinRoute(
    const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * next_lanelet) const;
  bool getPreviousLaneletWithinRouteExceptGoal(
    const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * prev_lanelet) const;
  bool getNextLaneletWithinRouteExceptStart(
    const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * next_lanelet) const;
  bool getRightLaneletWithinRoute(
    const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * right_lanelet) const;
  bool getLeftLaneletWithinRoute(
    const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * left_lanelet) const;
  lanelet::ConstLanelets getRouteLanelets() const;
  lanelet::ConstLanelets getLaneletSequenceUpTo(
    const lanelet::ConstLanelet & lanelet,
    const double min_length = std::numeric_limits<double>::max()) const;
  lanelet::ConstLanelets getLaneletSequenceAfter(
    const lanelet::ConstLanelet & lanelet,
    const double min_length = std::numeric_limits<double>::max()) const;
  bool getFollowingShoulderLanelet(
    const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * following_lanelet) const;
  lanelet::ConstLanelets getShoulderLaneletSequenceAfter(
    const lanelet::ConstLanelet & lanelet,
    const double min_length = std::numeric_limits<double>::max()) const;
  bool getPreviousShoulderLanelet(
    const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * prev_lanelet) const;
  lanelet::ConstLanelets getShoulderLaneletSequenceUpTo(
    const lanelet::ConstLanelet & lanelet,
    const double min_length = std::numeric_limits<double>::max()) const;
  lanelet::ConstLanelets getPreviousLaneletSequence(
    const lanelet::ConstLanelets & lanelet_sequence) const;
  lanelet::ConstLanelets getClosestLaneletSequence(const Pose & pose) const;
  lanelet::ConstLanelets getLaneChangeTargetLanes(const Pose & pose) const;
  lanelet::ConstLanelets getLaneSequenceUpTo(const lanelet::ConstLanelet & lanelet) const;
  lanelet::ConstLanelets getLaneSequenceAfter(const lanelet::ConstLanelet & lanelet) const;
  lanelet::ConstLanelets getLaneSequence(const lanelet::ConstLanelet & lanelet) const;
  lanelet::ConstLanelets getNeighborsWithinRoute(const lanelet::ConstLanelet & lanelet) const;
  std::vector<lanelet::ConstLanelets> getLaneSection(const lanelet::ConstLanelet & lanelet) const;
  lanelet::ConstLanelets getNextLaneSequence(const lanelet::ConstLanelets & lane_sequence) const;

  // for path

  PathWithLaneId updatePathTwist(const PathWithLaneId & path) const;
};
}  // namespace route_handler
#endif  // ROUTE_HANDLER__ROUTE_HANDLER_HPP_
