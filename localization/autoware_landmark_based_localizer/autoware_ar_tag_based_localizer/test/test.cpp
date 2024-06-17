// Copyright 2023 Autoware Foundation
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

#include "../src/ar_tag_based_localizer.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>
#include <rcl_yaml_param_parser/parser.h>

#include <memory>
#include <string>
#include <vector>

class TestArTagBasedLocalizer : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const std::string yaml_path =
      ament_index_cpp::get_package_share_directory("autoware_ar_tag_based_localizer") +
      "/config/ar_tag_based_localizer.param.yaml";

    rcl_params_t * params_st = rcl_yaml_node_struct_init(rcl_get_default_allocator());
    if (!rcl_parse_yaml_file(yaml_path.c_str(), params_st)) {
      std::cout << "Failed to parse yaml file" << std::endl;
      return;
    }

    const rclcpp::ParameterMap param_map = rclcpp::parameter_map_from(params_st, "");
    rclcpp::NodeOptions node_options;
    for (const auto & param_pair : param_map) {
      for (const auto & param : param_pair.second) {
        node_options.parameter_overrides().push_back(param);
      }
    }
    node_ = std::make_shared<ArTagBasedLocalizer>(node_options);
    rcl_yaml_node_struct_fini(params_st);
  }

  std::shared_ptr<ArTagBasedLocalizer> node_;
};

TEST_F(TestArTagBasedLocalizer, test_setup)  // NOLINT
{
  // Check if the constructor finishes successfully
  // For some unknown reason, the test sometimes fails to terminate normally and results in failure
  // unless a 1-second wait is implemented.
  std::this_thread::sleep_for(std::chrono::seconds(1));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
