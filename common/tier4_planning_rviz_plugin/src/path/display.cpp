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

#include <path/display.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace rviz_plugins
{
AutowarePathWithLaneIdDisplay::AutowarePathWithLaneIdDisplay()
: property_lane_id_view_{"View LaneId", true, "", this},
  property_lane_id_scale_{"Scale", 0.1, "", &property_lane_id_view_}
{
}

AutowarePathWithLaneIdDisplay::~AutowarePathWithLaneIdDisplay()
{
  for (const auto & e : lane_id_obj_ptrs_) {
    scene_node_->removeChild(e.first.get());
  }
  lane_id_obj_ptrs_.clear();
  lane_id_obj_ptrs_.shrink_to_fit();
}

void AutowarePathWithLaneIdDisplay::preprocessMessageDetail(
  const autoware_auto_planning_msgs::msg::PathWithLaneId::ConstSharedPtr msg_ptr)
{
  const size_t size = msg_ptr->points.size();
  if (size > lane_id_obj_ptrs_.size()) {
    for (std::size_t i = lane_id_obj_ptrs_.size(); i < size; i++) {
      std::unique_ptr<Ogre::SceneNode> node_ptr;
      node_ptr.reset(scene_node_->createChildSceneNode());
      auto text_ptr =
        std::make_unique<rviz_rendering::MovableText>("not initialized", "Liberation Sans", 0.1);
      text_ptr->setVisible(false);
      text_ptr->setTextAlignment(
        rviz_rendering::MovableText::H_CENTER, rviz_rendering::MovableText::V_ABOVE);
      node_ptr->attachObject(text_ptr.get());
      lane_id_obj_ptrs_.push_back(std::make_pair(std::move(node_ptr), std::move(text_ptr)));
    }
  } else {
    for (std::size_t i = lane_id_obj_ptrs_.size() - 1; i >= size; i--) {
      scene_node_->removeChild(lane_id_obj_ptrs_.at(i).first.get());
    }
    lane_id_obj_ptrs_.resize(size);
  }
}

void AutowarePathWithLaneIdDisplay::processMessageDetail(
  const autoware_auto_planning_msgs::msg::PathWithLaneId::ConstSharedPtr msg_ptr,
  const size_t p_idx)
{
  const auto & point = msg_ptr->points.at(p_idx);

  // LaneId
  if (property_lane_id_view_.getBool()) {
    Ogre::Vector3 position;
    position.x = point.point.pose.position.x;
    position.y = point.point.pose.position.y;
    position.z = point.point.pose.position.z;
    auto & node_ptr = lane_id_obj_ptrs_.at(p_idx).first;
    node_ptr->setPosition(position);

    const auto & text_ptr = lane_id_obj_ptrs_.at(p_idx).second;
    std::string lane_ids_str = "";
    for (const auto & e : point.lane_ids) {
      lane_ids_str += std::to_string(e) + ", ";
    }
    text_ptr->setCaption(lane_ids_str);
    text_ptr->setCharacterHeight(property_lane_id_scale_.getFloat());
    text_ptr->setVisible(true);
  } else {
    const auto & text_ptr = lane_id_obj_ptrs_.at(p_idx).second;
    text_ptr->setVisible(false);
  }
}
}  // namespace rviz_plugins

PLUGINLIB_EXPORT_CLASS(rviz_plugins::AutowarePathWithLaneIdDisplay, rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(rviz_plugins::AutowarePathDisplay, rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(rviz_plugins::AutowareTrajectoryDisplay, rviz_common::Display)
