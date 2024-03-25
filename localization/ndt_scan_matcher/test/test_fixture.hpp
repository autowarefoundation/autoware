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

#ifndef TEST_FIXTURE_HPP_
#define TEST_FIXTURE_HPP_

#include "../include/ndt_scan_matcher/ndt_scan_matcher_core.hpp"
#include "stub_initialpose_client.hpp"
#include "stub_pcd_loader.hpp"
#include "stub_sensor_pcd_publisher.hpp"
#include "stub_trigger_node_client.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>
#include <rcl_yaml_param_parser/parser.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <memory>
#include <string>
#include <vector>

class TestNDTScanMatcher : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const std::string yaml_path = ament_index_cpp::get_package_share_directory("ndt_scan_matcher") +
                                  "/config/ndt_scan_matcher.param.yaml";

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
    node_ = std::make_shared<NDTScanMatcher>(node_options);
    rcl_yaml_node_struct_fini(params_st);

    // prepare tf_static "base_link -> sensor_frame"
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
    geometry_msgs::msg::TransformStamped tf_static;
    tf_static.header.stamp.sec = 0;
    tf_static.header.stamp.nanosec = 0;
    tf_static.header.frame_id = "base_link";
    tf_static.child_frame_id = "sensor_frame";
    tf_static.transform.translation.x = 0.0;
    tf_static.transform.translation.y = 0.0;
    tf_static.transform.translation.z = 0.0;
    tf_static.transform.rotation.x = 0.0;
    tf_static.transform.rotation.y = 0.0;
    tf_static.transform.rotation.z = 0.0;
    tf_static.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(tf_static);

    // prepare stubs
    pcd_loader_ = std::make_shared<StubPcdLoader>();
    initialpose_client_ = std::make_shared<StubInitialposeClient>();
    trigger_node_client_ = std::make_shared<StubTriggerNodeClient>();
    sensor_pcd_publisher_ = std::make_shared<StubSensorPcdPublisher>();
  }

  std::shared_ptr<NDTScanMatcher> node_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<StubPcdLoader> pcd_loader_;
  std::shared_ptr<StubInitialposeClient> initialpose_client_;
  std::shared_ptr<StubTriggerNodeClient> trigger_node_client_;
  std::shared_ptr<StubSensorPcdPublisher> sensor_pcd_publisher_;
};

#endif  // TEST_FIXTURE_HPP_
