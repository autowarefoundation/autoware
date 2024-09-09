// Copyright 2024 TIER IV, Inc.
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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/behavior_path_goal_planner_module/util.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

class TestUtilWithMap : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    // parameters
    auto node_options = rclcpp::NodeOptions{};
    node_options.arguments(std::vector<std::string>{
      "--ros-args", "--params-file",
      ament_index_cpp::get_package_share_directory("autoware_test_utils") +
        "/config/test_vehicle_info.param.yaml"});
    auto node = rclcpp::Node::make_shared("test", node_options);
    vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(*node).getVehicleInfo();

    // lanelet map
    const std::string shoulder_map_path = autoware::test_utils::get_absolute_path_to_lanelet_map(
      "autoware_behavior_path_planner_common", "road_shoulder/lanelet2_map.osm");
    const auto map_bin_msg = autoware::test_utils::make_map_bin_msg(shoulder_map_path, 0.5);

    // load map
    route_handler = std::make_shared<autoware::route_handler::RouteHandler>(map_bin_msg);
  }

  void TearDown() override { rclcpp::shutdown(); }

public:
  std::shared_ptr<autoware::route_handler::RouteHandler> route_handler;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info;
};

TEST_F(TestUtilWithMap, getBusStopAreaPolygons)
{
  const auto lanes = lanelet::utils::query::laneletLayer(route_handler->getLaneletMapPtr());
  const auto shoulder_lanes = lanelet::utils::query::shoulderLanelets(lanes);
  const auto bus_stop_area_polygons =
    autoware::behavior_path_planner::goal_planner_utils::getBusStopAreaPolygons(shoulder_lanes);
  EXPECT_EQ(bus_stop_area_polygons.size(), 2);
}

TEST_F(TestUtilWithMap, isWithinAreas)
{
  const auto lanes = lanelet::utils::query::laneletLayer(route_handler->getLaneletMapPtr());
  const auto shoulder_lanes = lanelet::utils::query::shoulderLanelets(lanes);
  const auto bus_stop_area_polygons =
    autoware::behavior_path_planner::goal_planner_utils::getBusStopAreaPolygons(shoulder_lanes);

  const auto footprint = vehicle_info.createFootprint();
  const geometry_msgs::msg::Pose baselink_pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(
        geometry_msgs::build<geometry_msgs::msg::Point>().x(273.102814).y(402.194794).z(0.0))
      .orientation(
        geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.707390).w(
          0.706824));
  const auto baselink_footprint = autoware::universe_utils::transformVector(
    footprint, autoware::universe_utils::pose2transform(baselink_pose));
  EXPECT_EQ(
    autoware::behavior_path_planner::goal_planner_utils::isWithinAreas(
      baselink_footprint, bus_stop_area_polygons),
    true);
}
