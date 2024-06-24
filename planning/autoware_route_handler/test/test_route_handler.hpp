// Copyright 2024 TIER IV
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

#ifndef TEST_ROUTE_HANDLER_HPP_
#define TEST_ROUTE_HANDLER_HPP_

#include "autoware_test_utils/autoware_test_utils.hpp"
#include "autoware_test_utils/mock_data_parser.hpp"
#include "gtest/gtest.h"

#include <autoware/route_handler/route_handler.hpp>
#include <lanelet2_extension/io/autoware_osm_parser.hpp>
#include <lanelet2_extension/projection/mgrs_projector.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>

#include <lanelet2_io/Io.h>

#include <memory>
#include <string>
namespace autoware::route_handler::test
{

using autoware::test_utils::get_absolute_path_to_lanelet_map;
using autoware::test_utils::get_absolute_path_to_route;
using autoware_map_msgs::msg::LaneletMapBin;
class TestRouteHandler : public ::testing::Test
{
public:
  TestRouteHandler()
  {
    set_route_handler("2km_test.osm");
    set_test_route(lane_change_right_test_route_filename);
  }

  TestRouteHandler(const TestRouteHandler &) = delete;
  TestRouteHandler(TestRouteHandler &&) = delete;
  TestRouteHandler & operator=(const TestRouteHandler &) = delete;
  TestRouteHandler & operator=(TestRouteHandler &&) = delete;
  ~TestRouteHandler() override = default;

  void set_route_handler(const std::string & lanelet_map_filename)
  {
    route_handler_.reset();
    const auto lanelet2_path =
      get_absolute_path_to_lanelet_map(autoware_test_utils_dir, lanelet_map_filename);
    const auto map_bin_msg =
      autoware::test_utils::make_map_bin_msg(lanelet2_path, center_line_resolution);
    route_handler_ = std::make_shared<RouteHandler>(map_bin_msg);
  }

  void set_test_route(const std::string & route_filename)
  {
    const auto rh_test_route =
      get_absolute_path_to_route(autoware_route_handler_dir, route_filename);
    route_handler_->setRoute(autoware::test_utils::parse_lanelet_route_file(rh_test_route));
  }

  lanelet::ConstLanelets get_current_lanes()
  {
    const auto current_pose = autoware::test_utils::createPose(-50.0, 1.75, 0.0, 0.0, 0.0, 0.0);
    lanelet::ConstLanelet closest_lanelet;
    [[maybe_unused]] const auto found_closest_lanelet =
      route_handler_->getClosestLaneletWithConstrainsWithinRoute(
        current_pose, &closest_lanelet, dist_threshold, yaw_threshold);
    return route_handler_->getLaneletSequence(
      closest_lanelet, current_pose, backward_path_length, forward_path_length);
  }

  std::shared_ptr<RouteHandler> route_handler_;
  std::string autoware_test_utils_dir{"autoware_test_utils"};
  std::string autoware_route_handler_dir{"autoware_route_handler"};
  std::string lane_change_right_test_route_filename{"lane_change_test_route.yaml"};

  static constexpr double center_line_resolution{5.0};
  static constexpr double dist_threshold{3.0};
  static constexpr double yaw_threshold{1.045};
  static constexpr double backward_path_length{5.0};
  static constexpr double forward_path_length{300.0};
};
}  // namespace autoware::route_handler::test

#endif  // TEST_ROUTE_HANDLER_HPP_
