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

#ifndef STATIC_CENTERLINE_GENERATOR__STATIC_CENTERLINE_GENERATOR_NODE_HPP_
#define STATIC_CENTERLINE_GENERATOR__STATIC_CENTERLINE_GENERATOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "static_centerline_generator/centerline_source/optimization_trajectory_based_centerline.hpp"
#include "static_centerline_generator/srv/load_map.hpp"
#include "static_centerline_generator/srv/plan_path.hpp"
#include "static_centerline_generator/srv/plan_route.hpp"
#include "static_centerline_generator/type_alias.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include <geography_utils/lanelet2_projector.hpp>

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace static_centerline_generator
{
using static_centerline_generator::srv::LoadMap;
using static_centerline_generator::srv::PlanPath;
using static_centerline_generator::srv::PlanRoute;

struct CenterlineWithRoute
{
  std::vector<TrajectoryPoint> centerline{};
  std::vector<lanelet::Id> route_lane_ids{};
};

class StaticCenterlineGeneratorNode : public rclcpp::Node
{
public:
  explicit StaticCenterlineGeneratorNode(const rclcpp::NodeOptions & node_options);
  void run();

private:
  // load map
  void load_map(const std::string & lanelet2_input_file_path);
  void on_load_map(
    const LoadMap::Request::SharedPtr request, const LoadMap::Response::SharedPtr response);

  // plan route
  std::vector<lanelet::Id> plan_route(
    const lanelet::Id start_lanelet_id, const lanelet::Id end_lanelet_id);
  void on_plan_route(
    const PlanRoute::Request::SharedPtr request, const PlanRoute::Response::SharedPtr response);

  // plan centerline
  CenterlineWithRoute generate_centerline_with_route();
  void on_plan_path(
    const PlanPath::Request::SharedPtr request, const PlanPath::Response::SharedPtr response);

  void update_centerline_range(const int traj_start_index, const int traj_end_index);
  void evaluate(
    const std::vector<lanelet::Id> & route_lane_ids,
    const std::vector<TrajectoryPoint> & optimized_traj_points);
  void save_map(
    const std::string & lanelet2_output_file_path,
    const CenterlineWithRoute & centerline_with_route);

  lanelet::LaneletMapPtr original_map_ptr_{nullptr};
  HADMapBin::ConstSharedPtr map_bin_ptr_{nullptr};
  std::shared_ptr<RouteHandler> route_handler_ptr_{nullptr};
  std::unique_ptr<lanelet::Projector> map_projector_{nullptr};

  std::pair<int, int> traj_range_indices_{0, 0};
  std::optional<CenterlineWithRoute> centerline_with_route_{std::nullopt};

  enum class CenterlineSource {
    OptimizationTrajectoryBase = 0,
    BagEgoTrajectoryBase,
  };
  CenterlineSource centerline_source_;
  OptimizationTrajectoryBasedCenterline optimization_trajectory_based_centerline_;

  // publisher
  rclcpp::Publisher<HADMapBin>::SharedPtr pub_map_bin_{nullptr};
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_debug_unsafe_footprints_{nullptr};
  rclcpp::Publisher<Trajectory>::SharedPtr pub_whole_centerline_{nullptr};
  rclcpp::Publisher<Trajectory>::SharedPtr pub_centerline_{nullptr};

  // subscriber
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_traj_start_index_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_traj_end_index_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_save_map_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_traj_resample_interval_;

  // service
  rclcpp::Service<LoadMap>::SharedPtr srv_load_map_;
  rclcpp::Service<PlanRoute>::SharedPtr srv_plan_route_;
  rclcpp::Service<PlanPath>::SharedPtr srv_plan_path_;

  // callback group for service
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  // vehicle info
  vehicle_info_util::VehicleInfo vehicle_info_;
};
}  // namespace static_centerline_generator
#endif  // STATIC_CENTERLINE_GENERATOR__STATIC_CENTERLINE_GENERATOR_NODE_HPP_
