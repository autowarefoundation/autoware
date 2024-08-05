// Copyright 2024 Tier IV, Inc.
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

#ifndef OBJECT_MANAGER_HPP_
#define OBJECT_MANAGER_HPP_

#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <boost/functional/hash.hpp>
#include <boost/uuid/uuid.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

namespace std
{
template <>
struct hash<unique_identifier_msgs::msg::UUID>
{
  size_t operator()(const unique_identifier_msgs::msg::UUID & uid) const
  {
    const auto & ids = uid.uuid;
    boost::uuids::uuid u = {ids[0], ids[1], ids[2],  ids[3],  ids[4],  ids[5],  ids[6],  ids[7],
                            ids[8], ids[9], ids[10], ids[11], ids[12], ids[13], ids[14], ids[15]};
    return boost::hash<boost::uuids::uuid>()(u);
  }
};
}  // namespace std

namespace autoware::behavior_velocity_planner
{

/**
 * @brief store collision information
 */
struct CollisionInterval
{
  enum LanePosition {
    FIRST,
    SECOND,
    ELSE,
  };
  LanePosition lane_position{LanePosition::ELSE};

  //! original predicted path
  std::vector<geometry_msgs::msg::Pose> path;

  //! possible collision interval position index on path
  std::pair<size_t, size_t> interval_position;

  //! possible collision interval time(without TTC margin)
  std::pair<double, double> interval_time;
};

struct CollisionKnowledge
{
  //! the time when the expected collision is judged
  rclcpp::Time stamp;

  enum SafeType {
    UNSAFE,
    SAFE,
    SAFE_UNDER_TRAFFIC_CONTROL,
  };
  SafeType safe_type{SafeType::UNSAFE};

  //! if !safe, this has value, and it safe, this maybe null if the predicted path does not
  //! intersect with ego path
  std::optional<CollisionInterval> interval{std::nullopt};

  double observed_velocity;
};

/**
 * @brief store collision information of object on the attention area
 */
class ObjectInfo
{
public:
  explicit ObjectInfo(const unique_identifier_msgs::msg::UUID & uuid);

  const autoware_perception_msgs::msg::PredictedObject & predicted_object() const
  {
    return predicted_object_;
  };

  std::optional<CollisionInterval> unsafe_info() const
  {
    if (safe_under_traffic_control_) {
      return std::nullopt;
    }
    if (!unsafe_interval_) {
      return std::nullopt;
    }
    return unsafe_interval_;
  }

  bool is_safe_under_traffic_control() const { return safe_under_traffic_control_; }

  /**
   * @brief update predicted_object_, attention_lanelet, stopline, dist_to_stopline
   */
  void initialize(
    const autoware_perception_msgs::msg::PredictedObject & object,
    std::optional<lanelet::ConstLanelet> attention_lanelet_opt,
    std::optional<lanelet::ConstLineString3d> stopline_opt);

  /**
   * @brief update unsafe_knowledge
   */
  void update_safety(
    const std::optional<CollisionInterval> & unsafe_interval,
    const std::optional<CollisionInterval> & safe_interval, const bool safe_under_traffic_control);

  /**
   * @brief find the estimated position of the object in the past
   */
  std::optional<geometry_msgs::msg::Point> estimated_past_position(
    const double past_duration) const;

  /**
   * @brief check if object can stop before stopline under the deceleration. return false if
   * stopline is null for conservative collision  checking
   */
  bool can_stop_before_stopline(const double brake_deceleration) const;

  /**
   * @brief check if object can stop before stopline within the overshoot margin. return false if
   * stopline is null for conservative collision checking
   */
  bool can_stop_before_ego_lane(
    const double brake_deceleration, const double tolerable_overshoot,
    lanelet::ConstLanelet ego_lane) const;

  /**
   * @brief check if the object is before the stopline within the specified margin
   */
  bool before_stopline_by(const double margin) const;

  void setDecisionAt1stPassJudgeLinePassage(const CollisionKnowledge & knowledge)
  {
    decision_at_1st_pass_judge_line_passage_ = knowledge;
  }

  void setDecisionAt2ndPassJudgeLinePassage(const CollisionKnowledge & knowledge)
  {
    decision_at_2nd_pass_judge_line_passage_ = knowledge;
  }

  const std::optional<CollisionInterval> & unsafe_interval() const { return unsafe_interval_; }

  double observed_velocity() const
  {
    return predicted_object_.kinematics.initial_twist_with_covariance.twist.linear.x;
  }

  const std::optional<CollisionKnowledge> & decision_at_1st_pass_judge_line_passage() const
  {
    return decision_at_1st_pass_judge_line_passage_;
  }

  const std::optional<CollisionKnowledge> & decision_at_2nd_pass_judge_line_passage() const
  {
    return decision_at_2nd_pass_judge_line_passage_;
  }

  const std::string uuid_str;

private:
  autoware_perception_msgs::msg::PredictedObject predicted_object_;

  //! null if the object in intersection_area but not in attention_area
  std::optional<lanelet::ConstLanelet> attention_lanelet_opt_{std::nullopt};

  //! null if the object in intersection_area but not in attention_area
  std::optional<lanelet::ConstLineString3d> stopline_opt_{std::nullopt};

  //! null if the object in intersection_area but not in attention_area
  std::optional<double> dist_to_stopline_opt_{std::nullopt};

  //! store the information if judged as UNSAFE
  std::optional<CollisionInterval> unsafe_interval_{std::nullopt};

  //! store the information if judged as SAFE
  std::optional<CollisionInterval> safe_interval_{std::nullopt};

  //! true if the object is judged as negligible given traffic light color
  bool safe_under_traffic_control_{false};

  std::optional<CollisionKnowledge> decision_at_1st_pass_judge_line_passage_{std::nullopt};
  std::optional<CollisionKnowledge> decision_at_2nd_pass_judge_line_passage_{std::nullopt};

  /**
   * @brief calculate/update the distance to corresponding stopline
   */
  void calc_dist_to_stopline();
};

/**
 * @brief store predicted objects for intersection
 */
class ObjectInfoManager
{
public:
  std::shared_ptr<ObjectInfo> registerObject(
    const unique_identifier_msgs::msg::UUID & uuid, const bool belong_attention_area,
    const bool belong_intersection_area, const bool is_parked_vehicle);

  void registerExistingObject(
    const unique_identifier_msgs::msg::UUID & uuid, const bool belong_attention_area,
    const bool belong_intersection_area, const bool is_parked_vehicle,
    std::shared_ptr<ObjectInfo> object);

  void clearObjects();

  const std::vector<std::shared_ptr<ObjectInfo>> & attentionObjects() const
  {
    return attention_area_objects_;
  }

  const std::vector<std::shared_ptr<ObjectInfo>> & parkedObjects() const { return parked_objects_; }

  std::vector<std::shared_ptr<ObjectInfo>> allObjects() const;

  const std::unordered_map<unique_identifier_msgs::msg::UUID, std::shared_ptr<ObjectInfo>> &
  getObjectsMap() const
  {
    return objects_info_;
  }

  void setPassed1stPassJudgeLineFirstTime(const rclcpp::Time & time)
  {
    passed_1st_judge_line_first_time_ = time;
  }
  void setPassed2ndPassJudgeLineFirstTime(const rclcpp::Time & time)
  {
    passed_2nd_judge_line_first_time_ = time;
  }

private:
  std::unordered_map<unique_identifier_msgs::msg::UUID, std::shared_ptr<ObjectInfo>> objects_info_;

  //! belong to attention area
  std::vector<std::shared_ptr<ObjectInfo>> attention_area_objects_;

  //! does not belong to attention area but to intersection area
  std::vector<std::shared_ptr<ObjectInfo>> intersection_area_objects_;

  //! parked objects on attention_area/intersection_area
  std::vector<std::shared_ptr<ObjectInfo>> parked_objects_;

  std::optional<rclcpp::Time> passed_1st_judge_line_first_time_{std::nullopt};
  std::optional<rclcpp::Time> passed_2nd_judge_line_first_time_{std::nullopt};
};

/**
 * @brief return the CollisionInterval struct if the predicted path collides ego path geometrically
 */
std::optional<CollisionInterval> findPassageInterval(
  const autoware_perception_msgs::msg::PredictedPath & predicted_path,
  const autoware_perception_msgs::msg::Shape & shape, const lanelet::BasicPolygon2d & ego_lane_poly,
  const std::optional<lanelet::ConstLanelet> & first_attention_lane_opt,
  const std::optional<lanelet::ConstLanelet> & second_attention_lane_opt);

}  // namespace autoware::behavior_velocity_planner

#endif  // OBJECT_MANAGER_HPP_
