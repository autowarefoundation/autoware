// Copyright 2021-2023 Tier IV, Inc.
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

#include <rclcpp/logger.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/lanelet_segment.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_routing/Forward.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include <limits>
#include <memory>
#include <optional>
#include <vector>

namespace route_handler
{
using autoware_auto_mapping_msgs::msg::HADMapBin;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::LaneletSegment;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using std_msgs::msg::Header;
using unique_identifier_msgs::msg::UUID;
using RouteSections = std::vector<autoware_planning_msgs::msg::LaneletSegment>;

enum class Direction { NONE, LEFT, RIGHT };
enum class PullOverDirection { NONE, LEFT, RIGHT };
enum class PullOutDirection { NONE, LEFT, RIGHT };

class RouteHandler
{
public:
  RouteHandler() = default;
  explicit RouteHandler(const HADMapBin & map_msg);

  // non-const methods
  void setMap(const HADMapBin & map_msg);
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
  std::vector<lanelet::ConstLanelet> getLanesBeforePose(
    const geometry_msgs::msg::Pose & pose, const double vehicle_length) const;
  std::vector<lanelet::ConstLanelet> getLanesAfterGoal(const double vehicle_length) const;

  // for lanelet
  bool getPreviousLaneletsWithinRoute(
    const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelets * prev_lanelets) const;
  bool getNextLaneletWithinRoute(
    const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * next_lanelet) const;
  bool isDeadEndLanelet(const lanelet::ConstLanelet & lanelet) const;
  lanelet::ConstLanelets getLaneletsFromPoint(const lanelet::ConstPoint3d & point) const;
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
   * @brief Searches the furthest linestring to the right side of the lanelet
   * Only lanelet with same direction is considered
   * @param the lanelet of interest
   * @return right most linestring of the lane with same direction
   */
  lanelet::ConstLineString3d getRightMostSameDirectionLinestring(
    const lanelet::ConstLanelet & lanelet, const bool enable_same_root = false) const noexcept;

  /**
   * @brief Searches the furthest linestring to the right side of the lanelet
   * Used to search for road shoulders. Lane direction is ignored
   * @param the lanelet of interest
   * @return right most linestring
   */
  lanelet::ConstLineString3d getRightMostLinestring(
    const lanelet::ConstLanelet & lanelet, const bool enable_same_root = false) const noexcept;

  /**
   * @brief Searches the furthest linestring to the left side of the lanelet
   * Only lanelet with same direction is considered
   * @param the lanelet of interest
   * @return left most linestring of the lane with same direction
   */
  lanelet::ConstLineString3d getLeftMostSameDirectionLinestring(
    const lanelet::ConstLanelet & lanelet, const bool enable_same_root = false) const noexcept;

  /**
   * @brief Searches the furthest linestring to the left side of the lanelet
   * Used to search for road shoulders. Lane direction is ignored
   * @param the lanelet of interest
   * @return left most linestring
   */
  lanelet::ConstLineString3d getLeftMostLinestring(
    const lanelet::ConstLanelet & lanelet, const bool enable_same_root = false) const noexcept;

  /**
   * @brief Return furthest linestring on both side of the lanelet
   * @param the lanelet of interest
   * @param (optional) search furthest right side
   * @param (optional) search furthest left side
   * @param (optional) include opposite lane as well
   * @return right and left linestrings
   */
  lanelet::ConstLineStrings3d getFurthestLinestring(
    const lanelet::ConstLanelet & lanelet, bool is_right = true, bool is_left = true,
    bool is_opposite = true, bool enable_same_root = false) const noexcept;

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
   * The total distance is computed from the front point of the centerline of the given lane to
   * the front point of the preferred lane.
   * @param lanelet lanelet to query
   * @param direction change direction
   * @return number of lanes from input to the preferred lane
   */
  double getTotalLateralDistanceToPreferredLane(
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
  lanelet::ConstLanelets getCheckTargetLanesFromPath(
    const PathWithLaneId & path, const lanelet::ConstLanelets & target_lanes,
    const double check_length) const;
  lanelet::routing::RelationType getRelation(
    const lanelet::ConstLanelet & prev_lane, const lanelet::ConstLanelet & next_lane) const;
  lanelet::ConstLanelets getShoulderLanelets() const;
  bool isShoulderLanelet(const lanelet::ConstLanelet & lanelet) const;
  bool isRouteLanelet(const lanelet::ConstLanelet & lanelet) const;

  // for path
  PathWithLaneId getCenterLinePath(
    const lanelet::ConstLanelets & lanelet_sequence, const double s_start, const double s_end,
    bool use_exact = true) const;
  std::optional<lanelet::ConstLanelet> getLaneChangeTarget(
    const lanelet::ConstLanelets & lanelets, const Direction direction = Direction::NONE) const;
  std::optional<lanelet::ConstLanelet> getLaneChangeTargetExceptPreferredLane(
    const lanelet::ConstLanelets & lanelets, const Direction direction) const;
  bool getRightLaneChangeTargetExceptPreferredLane(
    const lanelet::ConstLanelets & lanelets, lanelet::ConstLanelet * target_lanelet) const;
  bool getLeftLaneChangeTargetExceptPreferredLane(
    const lanelet::ConstLanelets & lanelets, lanelet::ConstLanelet * target_lanelet) const;
  static bool getPullOverTarget(
    const lanelet::ConstLanelets & lanelets, const Pose & goal_pose,
    lanelet::ConstLanelet * target_lanelet);
  static bool getPullOutStartLane(
    const lanelet::ConstLanelets & lanelets, const Pose & pose, const double vehicle_width,
    lanelet::ConstLanelet * target_lanelet);
  double getLaneChangeableDistance(const Pose & current_pose, const Direction & direction) const;
  bool getLeftShoulderLanelet(
    const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * left_lanelet) const;
  bool getRightShoulderLanelet(
    const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * right_lanelet) const;
  lanelet::ConstPolygon3d getIntersectionAreaById(const lanelet::Id id) const;
  bool isPreferredLane(const lanelet::ConstLanelet & lanelet) const;

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
  bool isInTargetLane(const PoseStamped & pose, const lanelet::ConstLanelets & target) const;
  bool isInPreferredLane(const PoseStamped & pose) const;
  bool isBijectiveConnection(
    const lanelet::ConstLanelets & lanelet_section1,
    const lanelet::ConstLanelets & lanelet_section2) const;
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
    const double min_length = std::numeric_limits<double>::max(),
    const bool only_route_lanes = true) const;
  lanelet::ConstLanelets getLaneletSequenceAfter(
    const lanelet::ConstLanelet & lanelet,
    const double min_length = std::numeric_limits<double>::max(),
    const bool only_route_lanes = true) const;
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
  /**
   * @brief Checks if a path has a no_drivable_lane or not
   * @param path lanelet path
   * @return true if the lanelet path includes at least one no_drivable_lane, false if it does not
   * include any.
   */
  bool hasNoDrivableLaneInPath(const lanelet::routing::LaneletPath & path) const;
  /**
   * @brief Searches for a path between start and goal lanelets that does not include any
   * no_drivable_lane. If there is more than one path fount, the function returns the shortest path
   * that does not include any no_drivable_lane.
   * @param start_lanelet start lanelet
   * @param goal_lanelet goal lanelet
   * @param drivable_lane_path output path that does not include no_drivable_lane.
   * @return true if a path without any no_drivable_lane found, false if this path is not found.
   */
  bool findDrivableLanePath(
    const lanelet::ConstLanelet & start_lanelet, const lanelet::ConstLanelet & goal_lanelet,
    lanelet::routing::LaneletPath & drivable_lane_path) const;
};
}  // namespace route_handler
#endif  // ROUTE_HANDLER__ROUTE_HANDLER_HPP_
