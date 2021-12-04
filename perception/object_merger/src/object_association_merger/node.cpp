// Copyright 2020 Tier IV, Inc.
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

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>

#include <chrono>
#include <unordered_map>
// #include <tf2_sensor_msgs/msg/tf2_sensor_msgs.hpp>
#include <object_association_merger/node.hpp>
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace object_association
{
ObjectAssociationMergerNode::ObjectAssociationMergerNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("cluster_data_association_node", node_options),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_),
  object0_sub_(this, "input/object0", rclcpp::QoS{1}.get_rmw_qos_profile()),
  object1_sub_(this, "input/object1", rclcpp::QoS{1}.get_rmw_qos_profile()),
  sync_(SyncPolicy(10), object0_sub_, object1_sub_)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  sync_.registerCallback(std::bind(&ObjectAssociationMergerNode::objectsCallback, this, _1, _2));

  merged_object_pub_ = create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>(
    "output/object", rclcpp::QoS{1});
}

void ObjectAssociationMergerNode::objectsCallback(
  const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr & input_object0_msg,
  const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr & input_object1_msg)
{
  // Guard
  if (merged_object_pub_->get_subscription_count() < 1) {
    return;
  }

  // build output msg
  autoware_auto_perception_msgs::msg::DetectedObjects output_msg;
  output_msg.header = input_object0_msg->header;

  /* global nearest neighbor */
  std::unordered_map<int, int> direct_assignment;
  std::unordered_map<int, int> reverse_assignment;
  Eigen::MatrixXd score_matrix =
    data_association_.calcScoreMatrix(*input_object1_msg, *input_object0_msg);
  data_association_.assign(score_matrix, direct_assignment, reverse_assignment);
  for (size_t object0_idx = 0; object0_idx < input_object0_msg->objects.size(); ++object0_idx) {
    if (direct_assignment.find(object0_idx) != direct_assignment.end()) {  // found
      // The one with the higher score will be hired.
      if (
        input_object1_msg->objects.at(direct_assignment.at(object0_idx)).existence_probability <
        input_object0_msg->objects.at(object0_idx).existence_probability) {
        output_msg.objects.push_back(input_object0_msg->objects.at(object0_idx));
      } else {
        output_msg.objects.push_back(
          input_object1_msg->objects.at(direct_assignment.at(object0_idx)));
      }
    } else {  // not found
      output_msg.objects.push_back(input_object0_msg->objects.at(object0_idx));
    }
  }
  for (size_t object1_idx = 0; object1_idx < input_object1_msg->objects.size(); ++object1_idx) {
    if (reverse_assignment.find(object1_idx) != reverse_assignment.end()) {  // found
    } else {                                                                 // not found
      output_msg.objects.push_back(input_object1_msg->objects.at(object1_idx));
    }
  }

  // publish output msg
  merged_object_pub_->publish(output_msg);
}
}  // namespace object_association

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(object_association::ObjectAssociationMergerNode)
