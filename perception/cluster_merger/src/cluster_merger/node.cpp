// Copyright 2023 TIER IV, Inc.
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

#include "cluster_merger/node.hpp"

#include "object_recognition_utils/object_recognition_utils.hpp"

#include <memory>
#include <string>
#include <vector>
namespace cluster_merger
{

ClusterMergerNode::ClusterMergerNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("cluster_merger_node", node_options),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_),
  objects0_sub_(this, "input/cluster0", rclcpp::QoS{1}.get_rmw_qos_profile()),
  objects1_sub_(this, "input/cluster1", rclcpp::QoS{1}.get_rmw_qos_profile()),
  sync_(SyncPolicy(10), objects0_sub_, objects1_sub_)
{
  output_frame_id_ = declare_parameter<std::string>("output_frame_id");
  using std::placeholders::_1;
  using std::placeholders::_2;
  sync_.registerCallback(std::bind(&ClusterMergerNode::objectsCallback, this, _1, _2));

  // Publisher
  pub_objects_ = create_publisher<DetectedObjectsWithFeature>("~/output/clusters", rclcpp::QoS{1});
}

void ClusterMergerNode::objectsCallback(
  const DetectedObjectsWithFeature::ConstSharedPtr & input_objects0_msg,
  const DetectedObjectsWithFeature::ConstSharedPtr & input_objects1_msg)
{
  if (pub_objects_->get_subscription_count() < 1) {
    return;
  }
  // TODO(badai-nguyen): transform input topics to desired frame before concatenating
  /* transform to the same with cluster0 frame id*/
  DetectedObjectsWithFeature transformed_objects0;
  DetectedObjectsWithFeature transformed_objects1;
  if (
    !object_recognition_utils::transformObjectsWithFeature(
      *input_objects0_msg, output_frame_id_, tf_buffer_, transformed_objects0) ||
    !object_recognition_utils::transformObjectsWithFeature(
      *input_objects1_msg, output_frame_id_, tf_buffer_, transformed_objects1)) {
    return;
  }

  DetectedObjectsWithFeature output_objects;
  output_objects.header = input_objects0_msg->header;
  // add check frame id and transform if they are different
  output_objects.feature_objects = transformed_objects0.feature_objects;
  output_objects.feature_objects.insert(
    output_objects.feature_objects.end(), transformed_objects1.feature_objects.begin(),
    transformed_objects1.feature_objects.end());
  pub_objects_->publish(output_objects);
}
}  // namespace cluster_merger

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(cluster_merger::ClusterMergerNode)
