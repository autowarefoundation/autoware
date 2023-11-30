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

#ifndef STATIC_CENTERLINE_OPTIMIZER__STATIC_CENTERLINE_OPTIMIZER_NODE_HPP_
#define STATIC_CENTERLINE_OPTIMIZER__STATIC_CENTERLINE_OPTIMIZER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "static_centerline_optimizer/srv/load_map.hpp"
#include "static_centerline_optimizer/srv/plan_path.hpp"
#include "static_centerline_optimizer/srv/plan_route.hpp"
#include "static_centerline_optimizer/type_alias.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include <memory>
#include <string>
#include <vector>

namespace static_centerline_optimizer
{
using static_centerline_optimizer::srv::LoadMap;
using static_centerline_optimizer::srv::PlanPath;
using static_centerline_optimizer::srv::PlanRoute;

class StaticCenterlineOptimizerNode : public rclcpp::Node
{
public:
  explicit StaticCenterlineOptimizerNode(const rclcpp::NodeOptions & node_options);
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

  // plan path
  std::vector<TrajectoryPoint> plan_path(const std::vector<lanelet::Id> & route_lane_ids);
  void on_plan_path(
    const PlanPath::Request::SharedPtr request, const PlanPath::Response::SharedPtr response);

  void evaluate(
    const std::vector<lanelet::Id> & route_lane_ids,
    const std::vector<TrajectoryPoint> & optimized_traj_points);
  void save_map(
    const std::string & lanelet2_output_file_path, const std::vector<lanelet::Id> & route_lane_ids,
    const std::vector<TrajectoryPoint> & optimized_traj_points);

  HADMapBin::ConstSharedPtr map_bin_ptr_{nullptr};
  std::shared_ptr<RouteHandler> route_handler_ptr_{nullptr};

  // publisher
  rclcpp::Publisher<HADMapBin>::SharedPtr pub_map_bin_{nullptr};
  rclcpp::Publisher<PathWithLaneId>::SharedPtr pub_raw_path_with_lane_id_{nullptr};
  rclcpp::Publisher<Path>::SharedPtr pub_raw_path_{nullptr};
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_debug_unsafe_footprints_{nullptr};
  rclcpp::Publisher<Trajectory>::SharedPtr pub_optimized_centerline_{nullptr};

  // service
  rclcpp::Service<LoadMap>::SharedPtr srv_load_map_;
  rclcpp::Service<PlanRoute>::SharedPtr srv_plan_route_;
  rclcpp::Service<PlanPath>::SharedPtr srv_plan_path_;

  // callback group for service
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  // vehicle info
  vehicle_info_util::VehicleInfo vehicle_info_;
};
}  // namespace static_centerline_optimizer
#endif  // STATIC_CENTERLINE_OPTIMIZER__STATIC_CENTERLINE_OPTIMIZER_NODE_HPP_
