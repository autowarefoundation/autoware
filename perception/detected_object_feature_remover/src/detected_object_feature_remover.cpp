// Copyright 2021 Tier IV, Inc.
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

#include <detected_object_feature_remover/detected_object_feature_remover.hpp>

namespace detected_object_feature_remover
{
DetectedObjectFeatureRemover::DetectedObjectFeatureRemover(const rclcpp::NodeOptions & node_options)
: Node("detected_object_feature_remover", node_options)
{
  using std::placeholders::_1;
  pub_ = this->create_publisher<DetectedObjects>("~/output", rclcpp::QoS(1));
  sub_ = this->create_subscription<DetectedObjectsWithFeature>(
    "~/input", 1, std::bind(&DetectedObjectFeatureRemover::objectCallback, this, _1));
}

void DetectedObjectFeatureRemover::objectCallback(
  const DetectedObjectsWithFeature::ConstSharedPtr input)
{
  DetectedObjects output;
  convert(*input, output);
  pub_->publish(output);
}

void DetectedObjectFeatureRemover::convert(
  const DetectedObjectsWithFeature & objs_with_feature, DetectedObjects & objs)
{
  objs.header = objs_with_feature.header;
  for (const auto & obj_with_feature : objs_with_feature.feature_objects) {
    objs.objects.emplace_back(obj_with_feature.object);
  }
}

}  // namespace detected_object_feature_remover

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(detected_object_feature_remover::DetectedObjectFeatureRemover)
