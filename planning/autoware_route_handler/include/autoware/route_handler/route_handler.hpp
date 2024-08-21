// Copyright 2021-2024 TIER IV, Inc.
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

#ifndef AUTOWARE__ROUTE_HANDLER__ROUTE_HANDLER_HPP_
#define AUTOWARE__ROUTE_HANDLER__ROUTE_HANDLER_HPP_

#include <rclcpp/logger.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/lanelet_segment.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_routing/Forward.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include <limits>
#include <memory>
#include <optional>
#include <vector>

namespace autoware::route_handler
{
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::LaneletSegment;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using std_msgs::msg::Header;
using tier4_planning_msgs::msg::PathWithLaneId;
using unique_identifier_msgs::msg::UUID;
using RouteSections = std::vector<autoware_planning_msgs::msg::LaneletSegment>;

enum class Direction { NONE, LEFT, RIGHT };
enum class PullOverDirection { NONE, LEFT, RIGHT };
enum class PullOutDirection { NONE, LEFT, RIGHT };

struct ReferencePoint
{
  bool is_waypoint{false};
  geometry_msgs::msg::Point point;
};
using PiecewiseReferencePoints = std::vector<ReferencePoint>;

struct PiecewiseWaypoints
{
  lanelet::Id lanelet_id;
  std::vector<geometry_msgs::msg::Point> piecewise_waypoints;
};
using Waypoints = std::vector<PiecewiseWaypoints>;

class RouteHandler
{
public:
  RouteHandler() = default;
  explicit RouteHandler(const LaneletMapBin & map_msg);

  // non-const methods
  void setMap(const LaneletMapBin & map_msg);
  void setRoute(const LaneletRoute & route_msg);
  void setRouteLanelets(const lanelet::ConstLanelets & path_lanelets);
  void clearRoute();

  // const methods

  // for route handler status
  bool isHandlerReady() const;
  lanelet::ConstPolygon3d getExtraDrivableAreaById(const lanelet::Id id) const;
  Header getRouteHeader() const;
  UUID getRouteUuid() const;
  bool isAllowedGoalModification() const;

  // for routing graph
  bool isMapMsgReady() const;
  lanelet::routing::RoutingGraphPtr getRoutingGraphPtr() const;
  lanelet::traffic_rules::TrafficRulesPtr getTrafficRulesPtr() const;
  std::shared_ptr<const lanelet::routing::RoutingGraphContainer> getOverallGraphPtr() const;
  lanelet::LaneletMapPtr getLaneletMapPtr() const;
  static bool isNoDrivableLane(const lanelet::ConstLanelet & llt);

  // for routing
  bool planPathLaneletsBetweenCheckpoints(
    const Pose & start_checkpoint, const Pose & goal_checkpoint,
    lanelet::ConstLanelets * path_lanelets, const bool consider_no_drivable_lanes = false) const;
  std::vector<LaneletSegment> createMapSegments(const lanelet::ConstLanelets & path_lanelets) const;
  static bool isRouteLooped(const RouteSections & route_sections);

  // for goal
  bool isInGoalRouteSection(const lanelet::ConstLanelet & lanelet) const;
  Pose getGoalPose() const;
  Pose getStartPose() const;
  Pose getOriginalStartPose() const;
  Pose getOriginalGoalPose() const;
  lanelet::Id getGoalLaneId() const;
  bool getGoalLanelet(lanelet::ConstLanelet * goal_lanelet) const;
  std::vector<lanelet::ConstLanelet> getLanesAfterGoal(const double vehicle_length) const;

  // for lanelet
  bool getPreviousLaneletsWithinRoute(
    const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelets * prev_lanelets) const;
  bool getNextLaneletsWithinRoute(
    const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelets * next_lanelets) const;
  bool getNextLaneletWithinRoute(
    const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * next_lanelet) const;
  bool isDeadEndLanelet(const lanelet::ConstLanelet & lanelet) const;
  lanelet::ConstLanelets getLaneChangeableNeighbors(const lanelet::ConstLanelet & lanelet) const;

  /**
   * @brief Check if same-direction lane is available at the right side of the lanelet
   * Searches for any lanes regardless of whether it is lane-changeable or not.
   * Required the linestring to be shared(same line ID) between the lanelets.
   * @param the lanelet of interest
   * @return vector of lanelet having same direction if true
   */
  std::optional<lanelet::ConstLanelet> getRightLanelet(
    const lanelet::ConstLanelet & lanelet, const bool enable_same_root = false,
    const bool get_shoulder_lane = true) const;

  /**
   * @brief Check if same-direction lane is available at the left side of the lanelet
   * Searches for any lanes regardless of whether it is lane-changeable or not.
   * Required the linestring to be shared(same line ID) between the lanelets.
   * @param the lanelet of interest
   * @return vector of lanelet having same direction if true
   */
  std::optional<lanelet::ConstLanelet> getLeftLanelet(
    const lanelet::ConstLanelet & lanelet, const bool enable_same_root = false,
    const bool get_shoulder_lane = true) const;
  lanelet::ConstLanelets getNextLanelets(const lanelet::ConstLanelet & lanelet) const;
  lanelet::ConstLanelets getPreviousLanelets(const lanelet::ConstLanelet & lanelet) const;

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
   * @brief Searches and return all lanelet on the left that shares same linestring
   * @param the lanelet of interest
   * @param (optional) flag to include the lane with opposite direction
   * @param (optional) flag to invert the opposite lanelet
   * @return vector of lanelet that is connected via share linestring
   */
  lanelet::ConstLanelets getAllLeftSharedLinestringLanelets(
    const lanelet::ConstLanelet & lane, const bool & include_opposite,
    const bool & invert_opposite = false) const noexcept;

  /**
   * @brief Searches and return all lanelet on the right that shares same linestring
   * @param the lanelet of interest
   * @param (optional) flag to include the lane with opposite direction
   * @param (optional) flag to invert the opposite lanelet
   * @return vector of lanelet that is connected via share linestring
   */
  lanelet::ConstLanelets getAllRightSharedLinestringLanelets(
    const lanelet::ConstLanelet & lane, const bool & include_opposite,
    const bool & invert_opposite = false) const noexcept;

  /**
   * @brief Searches and return all lanelet (left and right) that shares same linestring
   * @param the lanelet of interest
   * @param (optional) flag to search only right side
   * @param (optional) flag to search only left side
   * @param (optional) flag to include the lane with opposite direction
   * @param (optional) flag to invert the opposite lanelet
   * @return vector of lanelet that is connected via share linestring
   */
  lanelet::ConstLanelets getAllSharedLineStringLanelets(
    const lanelet::ConstLanelet & current_lane, bool is_right = true, bool is_left = true,
    bool is_opposite = true, const bool & invert_opposite = false) const noexcept;

  /**
   * @brief Check if same-direction lane is available at the right side of the lanelet
   * Searches for any lanes regardless of whether it is lane-changeable or not.
   * Required the linestring to be shared(same line ID) between the lanelets.
   * @param the lanelet of interest
   * @return vector of lanelet having same direction if true
   */
  lanelet::ConstLanelet getMostRightLanelet(
    const lanelet::ConstLanelet & lanelet, const bool enable_same_root = false,
    const bool get_shoulder_lane = false) const;

  /**
   * @brief Check if same-direction lane is available at the left side of the lanelet
   * Searches for any lanes regardless of whether it is lane-changeable or not.
   * Required the linestring to be shared(same line ID) between the lanelets.
   * @param the lanelet of interest
   * @return vector of lanelet having same direction if true
   */
  lanelet::ConstLanelet getMostLeftLanelet(
    const lanelet::ConstLanelet & lanelet, const bool enable_same_root = false,
    const bool get_shoulder_lane = false) const;

  /**
   * Retrieves a sequence of lanelets before the given lanelet.
   * The total length of retrieved lanelet sequence at least given length. Returned lanelet sequence
   * does not include input lanelet.]
   * @param graph [input lanelet routing graph]
   * @param lanelet [input lanelet]
   * @param length [minimum length of retrieved lanelet sequence]
   * @return   [lanelet sequence that leads to given lanelet]
   */
  std::vector<lanelet::ConstLanelets> getPrecedingLaneletSequence(
    const lanelet::ConstLanelet & lanelet, const double length,
    const lanelet::ConstLanelets & exclude_lanelets = {}) const;

  /**
   * Query input lanelet  to see whether it exist in the preferred lane. If it doesn't exist, return
   * the number of lane-changeable lane to the preferred lane.
   * @param Desired lanelet to query
   * @param lane change direction
   * @return number of lanes from input to the preferred lane
   */
  int getNumLaneToPreferredLane(
    const lanelet::ConstLanelet & lanelet, const Direction direction = Direction::NONE) const;

  /**
   * Query input lanelet to see whether it exist in the preferred lane. If it doesn't exist, return
   * the distance to the preferred lane from the give lane.
   * This computes each lateral interval to the preferred lane from the given lanelet
   * @param lanelet lanelet to query
   * @param direction change direction
   * @return number of lanes from input to the preferred lane
   */
  std::vector<double> getLateralIntervalsToPreferredLane(
    const lanelet::ConstLanelet & lanelet, const Direction direction = Direction::NONE) const;

  bool getClosestLaneletWithinRoute(
    const Pose & search_pose, lanelet::ConstLanelet * closest_lanelet) const;
  bool getClosestPreferredLaneletWithinRoute(
    const Pose & search_pose, lanelet::ConstLanelet * closest_lanelet) const;
  bool getClosestLaneletWithConstrainsWithinRoute(
    const Pose & search_pose, lanelet::ConstLanelet * closest_lanelet, const double dist_threshold,
    const double yaw_threshold) const;

  /**
   * Finds the closest route lanelet to the search pose satisfying the distance and yaw constraints
   * with respect to a reference lanelet, the search set will include previous route lanelets,
   * next route lanelets, and neighbors of reference lanelet. Returns false if failed to find
   * lanelet.
   * @param search_pose pose to find closest lanelet to
   * @param reference_lanelet reference lanelet to decide the search set
   * @param dist_threshold distance constraint closest lanelet must be within
   * @param yaw_threshold yaw constraint closest lanelet direction must be within
   */
  bool getClosestRouteLaneletFromLanelet(
    const Pose & search_pose, const lanelet::ConstLanelet & reference_lanelet,
    lanelet::ConstLanelet * closest_lanelet, const double dist_threshold,
    const double yaw_threshold) const;

  lanelet::ConstLanelet getLaneletsFromId(const lanelet::Id id) const;
  lanelet::ConstLanelets getLaneletsFromIds(const lanelet::Ids & ids) const;
  lanelet::ConstLanelets getLaneletSequence(
    const lanelet::ConstLanelet & lanelet, const Pose & current_pose,
    const double backward_distance, const double forward_distance,
    const bool only_route_lanes = true) const;
  lanelet::ConstLanelets getLaneletSequence(
    const lanelet::ConstLanelet & lanelet,
    const double backward_distance = std::numeric_limits<double>::max(),
    const double forward_distance = std::numeric_limits<double>::max(),
    const bool only_route_lanes = true) const;
  lanelet::ConstLanelets getShoulderLaneletSequence(
    const lanelet::ConstLanelet & lanelet, const Pose & pose,
    const double backward_distance = std::numeric_limits<double>::max(),
    const double forward_distance = std::numeric_limits<double>::max()) const;
  bool isShoulderLanelet(const lanelet::ConstLanelet & lanelet) const;
  bool isRouteLanelet(const lanelet::ConstLanelet & lanelet) const;
  bool isRoadLanelet(const lanelet::ConstLanelet & lanelet) const;
  lanelet::ConstLanelets getPreferredLanelets() const;

  // for path
  PathWithLaneId getCenterLinePath(
    const lanelet::ConstLanelets & lanelet_sequence, const double s_start, const double s_end,
    bool use_exact = true) const;
  std::vector<Waypoints> calcWaypointsVector(const lanelet::ConstLanelets & lanelet_sequence) const;
  void removeOverlappedCenterlineWithWaypoints(
    std::vector<PiecewiseReferencePoints> & piecewise_ref_points_vec,
    const std::vector<geometry_msgs::msg::Point> & piecewise_waypoints,
    const lanelet::ConstLanelets & lanelet_sequence,
    const size_t piecewise_waypoints_lanelet_sequence_index,
    const bool is_removing_direction_forward) const;
  std::optional<lanelet::ConstLanelet> getLaneChangeTarget(
    const lanelet::ConstLanelets & lanelets, const Direction direction = Direction::NONE) const;
  std::optional<lanelet::ConstLanelet> getLaneChangeTargetExceptPreferredLane(
    const lanelet::ConstLanelets & lanelets, const Direction direction) const;
  std::optional<lanelet::ConstLanelet> getPullOverTarget(const Pose & goal_pose) const;
  std::optional<lanelet::ConstLanelet> getPullOutStartLane(
    const Pose & pose, const double vehicle_width) const;
  lanelet::ConstLanelets getRoadLaneletsAtPose(const Pose & pose) const;
  std::optional<lanelet::ConstLanelet> getLeftShoulderLanelet(
    const lanelet::ConstLanelet & lanelet) const;
  std::optional<lanelet::ConstLanelet> getRightShoulderLanelet(
    const lanelet::ConstLanelet & lanelet) const;

  /**
   * @brief Search and return shoulder lanelets that intersect with a given pose.
   * @param pose reference pose at which to search for shoulder lanelets.
   * @return vector of shoulder lanelets intersecting with given pose.
   */
  lanelet::ConstLanelets getShoulderLaneletsAtPose(const Pose & pose) const;

private:
  // MUST
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;
  std::shared_ptr<const lanelet::routing::RoutingGraphContainer> overall_graphs_ptr_;
  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::ConstLanelets route_lanelets_;
  lanelet::ConstLanelets preferred_lanelets_;
  lanelet::ConstLanelets start_lanelets_;
  lanelet::ConstLanelets goal_lanelets_;
  std::shared_ptr<LaneletRoute> route_ptr_{nullptr};

  rclcpp::Logger logger_{rclcpp::get_logger("route_handler")};

  bool is_map_msg_ready_{false};
  bool is_handler_ready_{false};

  // save original(not modified) route start pose for start planer execution
  Pose original_start_pose_;
  Pose original_goal_pose_;

  // non-const methods
  void setLaneletsFromRouteMsg();

  // const methods
  // for routing
  lanelet::ConstLanelets getMainLanelets(const lanelet::ConstLanelets & path_lanelets) const;

  // for lanelet
  lanelet::ConstLanelets getRouteLanelets() const;
  lanelet::ConstLanelets getLaneletSequenceUpTo(
    const lanelet::ConstLanelet & lanelet,
    const double min_length = std::numeric_limits<double>::max(),
    const bool only_route_lanes = true) const;
  lanelet::ConstLanelets getLaneletSequenceAfter(
    const lanelet::ConstLanelet & lanelet,
    const double min_length = std::numeric_limits<double>::max(),
    const bool only_route_lanes = true) const;
  std::optional<lanelet::ConstLanelet> getFollowingShoulderLanelet(
    const lanelet::ConstLanelet & lanelet) const;
  lanelet::ConstLanelets getShoulderLaneletSequenceAfter(
    const lanelet::ConstLanelet & lanelet,
    const double min_length = std::numeric_limits<double>::max()) const;
  std::optional<lanelet::ConstLanelet> getPreviousShoulderLanelet(
    const lanelet::ConstLanelet & lanelet) const;
  lanelet::ConstLanelets getShoulderLaneletSequenceUpTo(
    const lanelet::ConstLanelet & lanelet,
    const double min_length = std::numeric_limits<double>::max()) const;
  lanelet::ConstLanelets getPreviousLaneletSequence(
    const lanelet::ConstLanelets & lanelet_sequence) const;
  lanelet::ConstLanelets getNeighborsWithinRoute(const lanelet::ConstLanelet & lanelet) const;

  // for path

  /**
   * @brief Checks if a path has a no_drivable_lane or not
   * @param path lanelet path
   * @return true if the lanelet path includes at least one no_drivable_lane, false if it does not
   * include any.
   */
  bool hasNoDrivableLaneInPath(const lanelet::routing::LaneletPath & path) const;
  /**
   * @brief Searches for the shortest path between start and goal lanelets that does not include any
   * no_drivable_lane.
   * @param start_lanelet start lanelet
   * @param goal_lanelet goal lanelet
   * @return the lanelet path (if found)
   */
  std::optional<lanelet::routing::LaneletPath> findDrivableLanePath(
    const lanelet::ConstLanelet & start_lanelet, const lanelet::ConstLanelet & goal_lanelet) const;
};

/// @brief custom routing cost with infinity cost for no drivable lanes
class RoutingCostDrivable : public lanelet::routing::RoutingCostDistance
{
public:
  RoutingCostDrivable() : lanelet::routing::RoutingCostDistance(10.0) {}
  inline double getCostSucceeding(
    const lanelet::traffic_rules::TrafficRules & trafficRules,
    const lanelet::ConstLaneletOrArea & from, const lanelet::ConstLaneletOrArea & to) const
  {
    if (
      (from.isLanelet() && RouteHandler::isNoDrivableLane(*from.lanelet())) ||
      (to.isLanelet() && RouteHandler::isNoDrivableLane(*to.lanelet())))
      return std::numeric_limits<double>::infinity();
    return lanelet::routing::RoutingCostDistance::getCostSucceeding(trafficRules, from, to);
  }
  inline double getCostLaneChange(
    const lanelet::traffic_rules::TrafficRules & trafficRules, const lanelet::ConstLanelets & from,
    const lanelet::ConstLanelets & to) const noexcept
  {
    if (
      std::any_of(from.begin(), from.end(), RouteHandler::isNoDrivableLane) ||
      std::any_of(to.begin(), to.end(), RouteHandler::isNoDrivableLane))
      return std::numeric_limits<double>::infinity();
    return lanelet::routing::RoutingCostDistance::getCostLaneChange(trafficRules, from, to);
  }
};  // class RoutingCostDrivable
}  // namespace autoware::route_handler
#endif  // AUTOWARE__ROUTE_HANDLER__ROUTE_HANDLER_HPP_
