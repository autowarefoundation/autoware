// Copyright 2023 The Autoware Foundation
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
#ifndef ADAPTER_PERCEPTION_HPP_
#define ADAPTER_PERCEPTION_HPP_

#include "adapter_base.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>

#include <string>

namespace autoware_auto_msgs_adapter
{
using Objects_auto = autoware_auto_perception_msgs::msg::PredictedObjects;
using Objects = autoware_perception_msgs::msg::PredictedObjects;

class AdapterPerception : public autoware_auto_msgs_adapter::AdapterBase<Objects, Objects_auto>
{
public:
  AdapterPerception(
    rclcpp::Node & node, const std::string & topic_name_source,
    const std::string & topic_name_target, const rclcpp::QoS & qos = rclcpp::QoS{1})
  : AdapterBase(node, topic_name_source, topic_name_target, qos)
  {
    RCLCPP_DEBUG(
      node.get_logger(), "AdapterPerception is initialized to convert: %s -> %s",
      topic_name_source.c_str(), topic_name_target.c_str());
  }

protected:
  Objects_auto convert(const Objects & msg_source) override
  {
    Objects_auto msg_auto;
    msg_auto.header = msg_source.header;

    autoware_auto_perception_msgs::msg::PredictedObject object_auto;
    for (size_t it_of_objects = 0; it_of_objects < msg_source.objects.size(); it_of_objects++) {
      // convert id and probability
      object_auto.object_id = msg_source.objects[it_of_objects].object_id;
      object_auto.existence_probability = msg_source.objects[it_of_objects].existence_probability;
      // convert classification
      autoware_auto_perception_msgs::msg::ObjectClassification classification;
      for (size_t i = 0; i < msg_source.objects[it_of_objects].classification.size(); i++) {
        classification.label = msg_source.objects[it_of_objects].classification[i].label;
        classification.probability =
          msg_source.objects[it_of_objects].classification[i].probability;
        object_auto.classification.push_back(classification);
      }
      // convert kinematics
      object_auto.kinematics.initial_pose_with_covariance =
        msg_source.objects[it_of_objects].kinematics.initial_pose_with_covariance;
      object_auto.kinematics.initial_twist_with_covariance =
        msg_source.objects[it_of_objects].kinematics.initial_twist_with_covariance;
      object_auto.kinematics.initial_acceleration_with_covariance =
        msg_source.objects[it_of_objects].kinematics.initial_acceleration_with_covariance;
      for (size_t j = 0; j < msg_source.objects[it_of_objects].kinematics.predicted_paths.size();
           j++) {
        autoware_auto_perception_msgs::msg::PredictedPath predicted_path;
        for (size_t k = 0;
             k < msg_source.objects[it_of_objects].kinematics.predicted_paths[j].path.size(); k++) {
          predicted_path.path.push_back(
            msg_source.objects[it_of_objects].kinematics.predicted_paths[j].path[k]);
        }
        predicted_path.time_step =
          msg_source.objects[it_of_objects].kinematics.predicted_paths[j].time_step;
        predicted_path.confidence =
          msg_source.objects[it_of_objects].kinematics.predicted_paths[j].confidence;
        object_auto.kinematics.predicted_paths.push_back(predicted_path);
      }
      // convert shape
      object_auto.shape.type = msg_source.objects[it_of_objects].shape.type;
      object_auto.shape.footprint = msg_source.objects[it_of_objects].shape.footprint;
      object_auto.shape.dimensions = msg_source.objects[it_of_objects].shape.dimensions;

      // add to objects list
      msg_auto.objects.push_back(object_auto);
    }
    return msg_auto;
  }
};
}  // namespace autoware_auto_msgs_adapter

#endif  // ADAPTER_PERCEPTION_HPP_
