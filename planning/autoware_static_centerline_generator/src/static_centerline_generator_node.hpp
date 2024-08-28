// Copyright 2022 Tier IV, Inc.
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

#ifndef STATIC_CENTERLINE_GENERATOR_NODE_HPP_
#define STATIC_CENTERLINE_GENERATOR_NODE_HPP_

#include "autoware/universe_utils/ros/parameter.hpp"
#include "autoware_static_centerline_generator/srv/load_map.hpp"
#include "autoware_static_centerline_generator/srv/plan_path.hpp"
#include "autoware_static_centerline_generator/srv/plan_route.hpp"
#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"
#include "centerline_source/optimization_trajectory_based_centerline.hpp"
#include "rclcpp/rclcpp.hpp"
#include "type_alias.hpp"

#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tier4_map_msgs/msg/map_projector_info.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::static_centerline_generator
{
using autoware_static_centerline_generator::srv::LoadMap;
using autoware_static_centerline_generator::srv::PlanPath;
using autoware_static_centerline_generator::srv::PlanRoute;
using tier4_map_msgs::msg::MapProjectorInfo;

struct CenterlineWithRoute
{
  std::vector<TrajectoryPoint> centerline{};
  std::vector<lanelet::Id> route_lane_ids{};
};
struct CenterlineHandler
{
  CenterlineHandler() = default;
  explicit CenterlineHandler(const CenterlineWithRoute & centerline_with_route)
  : whole_centerline_with_route(centerline_with_route),
    start_index(0),
    end_index(centerline_with_route.centerline.size() - 1)
  {
  }
  std::vector<TrajectoryPoint> get_selected_centerline() const
  {
    if (!whole_centerline_with_route) {
      return std::vector<TrajectoryPoint>{};
    }
    const auto & centerline_begin = whole_centerline_with_route->centerline.begin();
    return std::vector<TrajectoryPoint>(
      centerline_begin + start_index, centerline_begin + end_index + 1);
  }
  std::vector<lanelet::Id> get_route_lane_ids() const
  {
    if (!whole_centerline_with_route) {
      return std::vector<lanelet::Id>{};
    }
    return whole_centerline_with_route->route_lane_ids;
  }
  bool is_valid() const { return whole_centerline_with_route && start_index < end_index; }
  bool update_start_index(const int arg_start_index)
  {
    if (whole_centerline_with_route && arg_start_index < end_index) {
      start_index = arg_start_index;
      return true;
    }
    return false;
  }
  bool update_end_index(const int arg_end_index)
  {
    if (whole_centerline_with_route && start_index < arg_end_index) {
      end_index = arg_end_index;
      return true;
    }
    return false;
  }

  std::optional<CenterlineWithRoute> whole_centerline_with_route{std::nullopt};
  int start_index{};
  int end_index{};
};

struct RoadBounds
{
  std::vector<geometry_msgs::msg::Point> left_bound;
  std::vector<geometry_msgs::msg::Point> right_bound;
};

class StaticCenterlineGeneratorNode : public rclcpp::Node
{
public:
  explicit StaticCenterlineGeneratorNode(const rclcpp::NodeOptions & node_options);
  void generate_centerline();
  void validate();
  void save_map();

private:
  // load map
  void load_map(const std::string & lanelet2_input_file_path);
  void on_load_map(
    const LoadMap::Request::SharedPtr request, const LoadMap::Response::SharedPtr response);

  // plan route
  std::vector<lanelet::Id> plan_route_by_lane_ids(
    const lanelet::Id start_lanelet_id, const lanelet::Id end_lanelet_id);
  std::vector<lanelet::Id> plan_route(
    const geometry_msgs::msg::Pose & start_center_pose,
    const geometry_msgs::msg::Pose & end_center_pose);

  void on_plan_route(
    const PlanRoute::Request::SharedPtr request, const PlanRoute::Response::SharedPtr response);

  // plan centerline
  CenterlineWithRoute generate_whole_centerline_with_route();
  std::vector<lanelet::Id> get_route_lane_ids_from_points(
    const std::vector<TrajectoryPoint> & points);
  void on_plan_path(
    const PlanPath::Request::SharedPtr request, const PlanPath::Response::SharedPtr response);

  void visualize_selected_centerline();

  // parameter
  template <typename T>
  T getRosParameter(const std::string & param_name)
  {
    return autoware::universe_utils::getOrDeclareParameter<T>(*this, param_name);
  }

  lanelet::LaneletMapPtr original_map_ptr_{nullptr};
  LaneletMapBin::ConstSharedPtr map_bin_ptr_{nullptr};
  std::shared_ptr<RouteHandler> route_handler_ptr_{nullptr};
  std::unique_ptr<MapProjectorInfo> map_projector_info_{nullptr};

  CenterlineHandler centerline_handler_;

  float footprint_margin_for_road_bound_{0.0};

  enum class CenterlineSource {
    OptimizationTrajectoryBase = 0,
    BagEgoTrajectoryBase,
  };
  CenterlineSource centerline_source_;
  OptimizationTrajectoryBasedCenterline optimization_trajectory_based_centerline_;

  // publisher
  rclcpp::Publisher<LaneletMapBin>::SharedPtr pub_map_bin_{nullptr};
  rclcpp::Publisher<Trajectory>::SharedPtr pub_whole_centerline_{nullptr};
  rclcpp::Publisher<Trajectory>::SharedPtr pub_centerline_{nullptr};
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_validation_results_{nullptr};
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_debug_ego_footprint_bounds_{nullptr};
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_debug_markers_{nullptr};

  // subscriber
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_traj_start_index_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_traj_end_index_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_save_map_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_validate_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_traj_resample_interval_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_footprint_margin_for_road_bound_;

  // service
  rclcpp::Service<LoadMap>::SharedPtr srv_load_map_;
  rclcpp::Service<PlanRoute>::SharedPtr srv_plan_route_;
  rclcpp::Service<PlanPath>::SharedPtr srv_plan_path_;

  // callback group for service
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  // vehicle info
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
};
}  // namespace autoware::static_centerline_generator
#endif  // STATIC_CENTERLINE_GENERATOR_NODE_HPP_
