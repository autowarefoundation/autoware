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

#ifndef SHAPE_ESTIMATION_NODE_HPP_
#define SHAPE_ESTIMATION_NODE_HPP_

#include "autoware/shape_estimation/shape_estimator.hpp"

#ifdef USE_CUDA
#include "autoware/shape_estimation/tensorrt_shape_estimator.hpp"

#include <tensorrt_common/tensorrt_common.hpp>
#endif

#include <autoware/universe_utils/ros/debug_publisher.hpp>
#include <autoware/universe_utils/ros/published_time_publisher.hpp>
#include <autoware/universe_utils/system/stop_watch.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>

#include <memory>

namespace autoware::shape_estimation
{

using autoware_perception_msgs::msg::DetectedObjects;
using tier4_perception_msgs::msg::DetectedObjectsWithFeature;
class ShapeEstimationNode : public rclcpp::Node
{
private:
  // ros
  rclcpp::Publisher<DetectedObjectsWithFeature>::SharedPtr pub_;
  rclcpp::Subscription<DetectedObjectsWithFeature>::SharedPtr sub_;
  std::unique_ptr<autoware::universe_utils::PublishedTimePublisher> published_time_publisher_;

  // debug publisher
  std::unique_ptr<autoware::universe_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
  std::unique_ptr<autoware::universe_utils::DebugPublisher> processing_time_publisher_;

  void callback(const DetectedObjectsWithFeature::ConstSharedPtr input_msg);

  std::unique_ptr<ShapeEstimator> estimator_;
  bool use_vehicle_reference_yaw_;
  bool use_vehicle_reference_shape_size_;
  bool fix_filtered_objects_label_to_unknown_;

#ifdef USE_CUDA
  std::unique_ptr<TrtShapeEstimator> tensorrt_shape_estimator_;
#endif

  bool use_ml_shape_estimation_;
  size_t min_points_;

public:
  explicit ShapeEstimationNode(const rclcpp::NodeOptions & node_options);
};
}  // namespace autoware::shape_estimation

#endif  // SHAPE_ESTIMATION_NODE_HPP_
