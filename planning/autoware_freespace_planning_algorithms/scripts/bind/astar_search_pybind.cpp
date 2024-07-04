// Copyright 2024 TIER IV, Inc. All rights reserved.
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

#include "autoware/freespace_planning_algorithms/abstract_algorithm.hpp"
#include "autoware/freespace_planning_algorithms/astar_search.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace autoware::freespace_planning_algorithms
{
struct PlannerWaypointsVector
{
  std::vector<std::vector<double>> waypoints;
  double length;
};

class AstarSearchPython : public freespace_planning_algorithms::AstarSearch
{
  using AstarSearch::AstarSearch;

public:
  void setMapByte(const std::string & costmap_byte)
  {
    rclcpp::SerializedMessage serialized_msg;
    static constexpr size_t message_header_length = 8u;
    serialized_msg.reserve(message_header_length + costmap_byte.size());
    serialized_msg.get_rcl_serialized_message().buffer_length = costmap_byte.size();
    for (size_t i = 0; i < costmap_byte.size(); ++i) {
      serialized_msg.get_rcl_serialized_message().buffer[i] = costmap_byte[i];
    }
    nav_msgs::msg::OccupancyGrid costmap;
    static rclcpp::Serialization<nav_msgs::msg::OccupancyGrid> serializer;
    serializer.deserialize_message(&serialized_msg, &costmap);

    freespace_planning_algorithms::AstarSearch::setMap(costmap);
  }

  bool makePlanByte(const std::string & start_pose_byte, const std::string & goal_pose_byte)
  {
    rclcpp::SerializedMessage serialized_start_msg;
    static constexpr size_t message_header_length = 8u;
    serialized_start_msg.reserve(message_header_length + start_pose_byte.size());
    serialized_start_msg.get_rcl_serialized_message().buffer_length = start_pose_byte.size();
    for (size_t i = 0; i < start_pose_byte.size(); ++i) {
      serialized_start_msg.get_rcl_serialized_message().buffer[i] = start_pose_byte[i];
    }
    geometry_msgs::msg::Pose start_pose;
    static rclcpp::Serialization<geometry_msgs::msg::Pose> start_serializer;
    start_serializer.deserialize_message(&serialized_start_msg, &start_pose);

    rclcpp::SerializedMessage serialized_goal_msg;
    serialized_goal_msg.reserve(message_header_length + goal_pose_byte.size());
    serialized_goal_msg.get_rcl_serialized_message().buffer_length = goal_pose_byte.size();
    for (size_t i = 0; i < goal_pose_byte.size(); ++i) {
      serialized_goal_msg.get_rcl_serialized_message().buffer[i] = goal_pose_byte[i];
    }
    geometry_msgs::msg::Pose goal_pose;
    static rclcpp::Serialization<geometry_msgs::msg::Pose> goal_serializer;
    goal_serializer.deserialize_message(&serialized_goal_msg, &goal_pose);

    return freespace_planning_algorithms::AstarSearch::makePlan(start_pose, goal_pose);
  }

  PlannerWaypointsVector getWaypointsAsVector()
  {
    freespace_planning_algorithms::PlannerWaypoints waypoints =
      freespace_planning_algorithms::AstarSearch::getWaypoints();
    PlannerWaypointsVector waypoints_vector;
    for (const auto & waypoint : waypoints.waypoints) {
      // NOTE: it was difficult to return the deserialized pose_byte and serialize the pose_byte on
      // python-side. So this function returns [*position, *quaternion] as double array
      const auto & xyz = waypoint.pose.pose.position;
      const auto & quat = waypoint.pose.pose.orientation;
      const double & is_back = waypoint.is_back;
      waypoints_vector.waypoints.push_back(
        std::vector<double>({xyz.x, xyz.y, xyz.z, quat.x, quat.y, quat.z, quat.w, is_back}));
    }
    waypoints_vector.length = waypoints.compute_length();
    return waypoints_vector;
  }
};

namespace py = pybind11;

// cppcheck-suppress syntaxError
PYBIND11_MODULE(autoware_freespace_planning_algorithms_pybind, p)
{
  auto pyPlannerWaypointsVector =
    py::class_<PlannerWaypointsVector>(p, "PlannerWaypointsVector", py::dynamic_attr())
      .def(py::init<>())
      .def_readwrite("waypoints", &PlannerWaypointsVector::waypoints)
      .def_readwrite("length", &PlannerWaypointsVector::length);
  auto pyAstarParam =
    py::class_<freespace_planning_algorithms::AstarParam>(p, "AstarParam", py::dynamic_attr())
      .def(py::init<>())
      .def_readwrite(
        "only_behind_solutions", &freespace_planning_algorithms::AstarParam::only_behind_solutions)
      .def_readwrite("use_back", &freespace_planning_algorithms::AstarParam::use_back)
      .def_readwrite(
        "distance_heuristic_weight",
        &freespace_planning_algorithms::AstarParam::distance_heuristic_weight);
  auto pyPlannerCommonParam =
    py::class_<freespace_planning_algorithms::PlannerCommonParam>(
      p, "PlannerCommonParam", py::dynamic_attr())
      .def(py::init<>())
      .def_readwrite("time_limit", &freespace_planning_algorithms::PlannerCommonParam::time_limit)
      .def_readwrite(
        "minimum_turning_radius",
        &freespace_planning_algorithms::PlannerCommonParam::minimum_turning_radius)
      .def_readwrite(
        "maximum_turning_radius",
        &freespace_planning_algorithms::PlannerCommonParam::maximum_turning_radius)
      .def_readwrite(
        "turning_radius_size",
        &freespace_planning_algorithms::PlannerCommonParam::turning_radius_size)
      .def_readwrite("theta_size", &freespace_planning_algorithms::PlannerCommonParam::theta_size)
      .def_readwrite(
        "curve_weight", &freespace_planning_algorithms::PlannerCommonParam::curve_weight)
      .def_readwrite(
        "reverse_weight", &freespace_planning_algorithms::PlannerCommonParam::reverse_weight)
      .def_readwrite(
        "lateral_goal_range",
        &freespace_planning_algorithms::PlannerCommonParam::lateral_goal_range)
      .def_readwrite(
        "longitudinal_goal_range",
        &freespace_planning_algorithms::PlannerCommonParam::longitudinal_goal_range)
      .def_readwrite(
        "angle_goal_range", &freespace_planning_algorithms::PlannerCommonParam::angle_goal_range)
      .def_readwrite(
        "obstacle_threshold",
        &freespace_planning_algorithms::PlannerCommonParam::obstacle_threshold);
  auto pyVehicleShape =
    py::class_<freespace_planning_algorithms::VehicleShape>(p, "VehicleShape", py::dynamic_attr())
      .def(py::init<>())
      .def(py::init<double, double, double>())
      .def_readwrite("length", &freespace_planning_algorithms::VehicleShape::length)
      .def_readwrite("width", &freespace_planning_algorithms::VehicleShape::width)
      .def_readwrite("base2back", &freespace_planning_algorithms::VehicleShape::base2back);

  py::class_<freespace_planning_algorithms::AbstractPlanningAlgorithm>(
    p, "AbstractPlanningAlgorithm");
  py::class_<
    freespace_planning_algorithms::AstarSearch,
    freespace_planning_algorithms::AbstractPlanningAlgorithm>(p, "AstarSearchCpp");
  py::class_<AstarSearchPython, freespace_planning_algorithms::AstarSearch>(p, "AstarSearch")
    .def(py::init<
         freespace_planning_algorithms::PlannerCommonParam &,
         freespace_planning_algorithms::VehicleShape &,
         freespace_planning_algorithms::AstarParam &>())
    .def("setMap", &AstarSearchPython::setMapByte)
    .def("makePlan", &AstarSearchPython::makePlanByte)
    .def("getWaypoints", &AstarSearchPython::getWaypointsAsVector);
}
}  // namespace autoware::freespace_planning_algorithms
