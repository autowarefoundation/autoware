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

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "autoware_test_utils/autoware_test_utils.hpp"
#include "autoware_test_utils/mock_data_parser.hpp"
#include "gtest/gtest.h"

#include <autoware_route_handler/route_handler.hpp>
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

using autoware_map_msgs::msg::LaneletMapBin;

class TestRouteHandler : public ::testing::Test
{
public:
  TestRouteHandler()
  {
    autoware_test_utils_dir = ament_index_cpp::get_package_share_directory("autoware_test_utils");
    set_route_handler("/test_map/2km_test.osm");
    set_test_route("/test_route/lane_change_test_route.yaml");
  }

  TestRouteHandler(const TestRouteHandler &) = delete;
  TestRouteHandler(TestRouteHandler &&) = delete;
  TestRouteHandler & operator=(const TestRouteHandler &) = delete;
  TestRouteHandler & operator=(TestRouteHandler &&) = delete;
  ~TestRouteHandler() override = default;

  void set_route_handler(const std::string & relative_path)
  {
    route_handler_.reset();
    const auto lanelet2_path = autoware_test_utils_dir + relative_path;
    const auto map_bin_msg =
      autoware::test_utils::make_map_bin_msg(lanelet2_path, center_line_resolution);
    route_handler_ = std::make_shared<RouteHandler>(map_bin_msg);
  }

  void set_test_route(const std::string & route_path)
  {
    const auto route_handler_dir =
      ament_index_cpp::get_package_share_directory("autoware_route_handler");
    const auto rh_test_route = route_handler_dir + route_path;
    route_handler_->setRoute(autoware::test_utils::parse_lanelet_route_file(rh_test_route));
  }

  std::shared_ptr<RouteHandler> route_handler_;
  std::string autoware_test_utils_dir;
  static constexpr double center_line_resolution = 5.0;
};
}  // namespace autoware::route_handler::test

#endif  // TEST_ROUTE_HANDLER_HPP_
