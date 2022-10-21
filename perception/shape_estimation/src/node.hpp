// Copyright 2018 Autoware Foundation. All rights reserved.
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

#ifndef NODE_HPP_
#define NODE_HPP_

#include "shape_estimation/shape_estimator.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>

#include <memory>

using autoware_auto_perception_msgs::msg::DetectedObjects;
using tier4_perception_msgs::msg::DetectedObjectsWithFeature;
class ShapeEstimationNode : public rclcpp::Node
{
private:
  // ros
  rclcpp::Publisher<DetectedObjectsWithFeature>::SharedPtr pub_;
  rclcpp::Subscription<DetectedObjectsWithFeature>::SharedPtr sub_;

  void callback(const DetectedObjectsWithFeature::ConstSharedPtr input_msg);

  std::unique_ptr<ShapeEstimator> estimator_;
  bool use_vehicle_reference_yaw_;
  bool use_vehicle_reference_shape_size_;

public:
  explicit ShapeEstimationNode(const rclcpp::NodeOptions & node_options);
};

#endif  // NODE_HPP_
