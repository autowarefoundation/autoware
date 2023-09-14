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

#include "simple_object_merger/simple_object_merger_node.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>
#include <string>
#include <vector>

namespace
{
template <class T>
bool update_param(
  const std::vector<rclcpp::Parameter> & params, const std::string & name, T & value)
{
  const auto itr = std::find_if(
    params.cbegin(), params.cend(),
    [&name](const rclcpp::Parameter & p) { return p.get_name() == name; });

  // Not found
  if (itr == params.cend()) {
    return false;
  }

  value = itr->template get_value<T>();
  return true;
}

autoware_auto_perception_msgs::msg::DetectedObjects::SharedPtr getTransformedObjects(
  autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr objects,
  const std::string & target_frame_id,
  geometry_msgs::msg::TransformStamped::ConstSharedPtr transform)
{
  autoware_auto_perception_msgs::msg::DetectedObjects::SharedPtr output_objects =
    std::const_pointer_cast<autoware_auto_perception_msgs::msg::DetectedObjects>(objects);

  if (objects->header.frame_id == target_frame_id) {
    return output_objects;
  }

  output_objects->header.frame_id = target_frame_id;
  for (auto & object : output_objects->objects) {
    // convert by tf
    geometry_msgs::msg::PoseStamped pose_stamped{};
    pose_stamped.pose = object.kinematics.pose_with_covariance.pose;
    geometry_msgs::msg::PoseStamped transformed_pose_stamped{};
    tf2::doTransform(pose_stamped, transformed_pose_stamped, *transform);
    object.kinematics.pose_with_covariance.pose = transformed_pose_stamped.pose;
  }
  return output_objects;
}

}  // namespace

namespace simple_object_merger
{
using namespace std::literals;
using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::nanoseconds;

SimpleObjectMergerNode::SimpleObjectMergerNode(const rclcpp::NodeOptions & node_options)
: Node("simple_object_merger", node_options)
{
  // Parameter Server
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&SimpleObjectMergerNode::onSetParam, this, std::placeholders::_1));

  // Node Parameter
  node_param_.update_rate_hz = declare_parameter<double>("update_rate_hz", 20.0);
  node_param_.new_frame_id = declare_parameter<std::string>("new_frame_id", "base_link");
  node_param_.timeout_threshold = declare_parameter<double>("timeout_threshold", 1.0);

  declare_parameter("input_topics", std::vector<std::string>());
  node_param_.topic_names = get_parameter("input_topics").as_string_array();
  if (node_param_.topic_names.empty()) {
    RCLCPP_ERROR(get_logger(), "Need a 'input_topics' parameter to be set before continuing");
    return;
  }

  input_topic_size = node_param_.topic_names.size();
  if (input_topic_size == 1) {
    RCLCPP_ERROR(get_logger(), "Only one topic given. Need at least two topics to continue.");
    return;
  }

  // Subscriber
  transform_listener_ = std::make_shared<tier4_autoware_utils::TransformListener>(this);
  sub_objects_array.resize(input_topic_size);
  objects_data_.resize(input_topic_size);

  for (size_t i = 0; i < input_topic_size; i++) {
    std::function<void(const DetectedObjects::ConstSharedPtr msg)> func =
      std::bind(&SimpleObjectMergerNode::onData, this, std::placeholders::_1, i);
    sub_objects_array.at(i) =
      create_subscription<DetectedObjects>(node_param_.topic_names.at(i), rclcpp::QoS{1}, func);
  }

  // Publisher
  pub_objects_ = create_publisher<DetectedObjects>("~/output/objects", 1);

  // Timer
  const auto update_period_ns = rclcpp::Rate(node_param_.update_rate_hz).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), update_period_ns, std::bind(&SimpleObjectMergerNode::onTimer, this));
}

void SimpleObjectMergerNode::onData(DetectedObjects::ConstSharedPtr msg, const size_t array_number)
{
  objects_data_.at(array_number) = msg;
}

rcl_interfaces::msg::SetParametersResult SimpleObjectMergerNode::onSetParam(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  try {
    // Node Parameter
    {
      auto & p = node_param_;
      // Update params
      update_param(params, "update_rate_hz", p.update_rate_hz);
      update_param(params, "timeout_threshold", p.timeout_threshold);
      update_param(params, "new_frame_id", p.new_frame_id);
      update_param(params, "topic_names", p.topic_names);
    }
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
    return result;
  }
  result.successful = true;
  result.reason = "success";
  return result;
}

bool SimpleObjectMergerNode::isDataReady()
{
  for (size_t i = 0; i < input_topic_size; i++) {
    if (!objects_data_.at(i)) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "waiting for object msg...");
      return false;
    }
  }

  return true;
}

void SimpleObjectMergerNode::onTimer()
{
  if (!isDataReady()) {
    return;
  }

  DetectedObjects output_objects;
  output_objects.header = objects_data_.at(0)->header;
  output_objects.header.frame_id = node_param_.new_frame_id;

  for (size_t i = 0; i < input_topic_size; i++) {
    double time_diff = rclcpp::Time(objects_data_.at(i)->header.stamp).seconds() -
                       rclcpp::Time(objects_data_.at(0)->header.stamp).seconds();
    if (std::abs(time_diff) < node_param_.timeout_threshold) {
      transform_ = transform_listener_->getTransform(
        node_param_.new_frame_id, objects_data_.at(i)->header.frame_id,
        objects_data_.at(i)->header.stamp, rclcpp::Duration::from_seconds(0.01));

      DetectedObjects::SharedPtr transformed_objects =
        getTransformedObjects(objects_data_.at(i), node_param_.new_frame_id, transform_);

      output_objects.objects.insert(
        output_objects.objects.end(), std::begin(transformed_objects->objects),
        std::end(transformed_objects->objects));
    } else {
      RCLCPP_INFO(
        rclcpp::get_logger("simple_object_merger"), "Topic of %s is timeout by %f sec",
        node_param_.topic_names.at(i).c_str(), time_diff);
    }
  }

  pub_objects_->publish(output_objects);
}

}  // namespace simple_object_merger

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(simple_object_merger::SimpleObjectMergerNode)
