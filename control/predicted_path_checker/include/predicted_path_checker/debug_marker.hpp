// Copyright 2023 LeoDrive A.Ş. All rights reserved.
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
#ifndef PREDICTED_PATH_CHECKER__DEBUG_MARKER_HPP_
#define PREDICTED_PATH_CHECKER__DEBUG_MARKER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>

#include <memory>
#include <string>
#include <vector>

#define EIGEN_MPL2_ONLY

#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

namespace autoware::motion::control::predicted_path_checker
{

enum class PolygonType : int8_t { Vehicle = 0, Collision };

enum class PointType : int8_t { Stop = 0 };

enum class PoseType : int8_t { Stop = 0, Collision };

class PredictedPathCheckerDebugNode
{
public:
  explicit PredictedPathCheckerDebugNode(rclcpp::Node * node, const double base_link2front);

  ~PredictedPathCheckerDebugNode() {}

  bool pushPolygon(
    const autoware::universe_utils::Polygon2d & polygon, const double z, const PolygonType & type);

  bool pushPolygon(const std::vector<Eigen::Vector3d> & polygon, const PolygonType & type);

  bool pushPolyhedron(
    const autoware::universe_utils::Polygon2d & polyhedron, const double z_min, const double z_max,
    const PolygonType & type);

  bool pushPolyhedron(const std::vector<Eigen::Vector3d> & polyhedron, const PolygonType & type);

  bool pushPose(const geometry_msgs::msg::Pose & pose, const PoseType & type);

  bool pushObstaclePoint(const geometry_msgs::msg::Point & obstacle_point, const PointType & type);

  visualization_msgs::msg::MarkerArray makeVirtualWallMarker();

  visualization_msgs::msg::MarkerArray makeVisualizationMarker();

  void publish();

private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr virtual_wall_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_viz_pub_;
  rclcpp::Node * node_;
  double base_link2front_;

  std::shared_ptr<geometry_msgs::msg::Pose> stop_pose_ptr_;
  std::shared_ptr<geometry_msgs::msg::Pose> collision_pose_ptr_;
  std::shared_ptr<geometry_msgs::msg::Point> stop_obstacle_point_ptr_;
  std::vector<std::vector<Eigen::Vector3d>> vehicle_polygons_;
  std::vector<std::vector<Eigen::Vector3d>> collision_polygons_;
  std::vector<std::vector<Eigen::Vector3d>> vehicle_polyhedrons_;
  std::vector<std::vector<Eigen::Vector3d>> collision_polyhedrons_;
};

}  // namespace autoware::motion::control::predicted_path_checker

#endif  // PREDICTED_PATH_CHECKER__DEBUG_MARKER_HPP_
