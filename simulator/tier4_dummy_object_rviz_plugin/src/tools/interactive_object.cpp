// Copyright 2022 Tier IV, Inc.
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
/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "interactive_object.hpp"

#include "util.hpp"

#include <rviz_common/display_context.hpp>

#include <algorithm>
#include <memory>
#include <random>
#include <string>

namespace rviz_plugins
{
InteractiveObject::InteractiveObject(const Ogre::Vector3 & point)
{
  velocity_ = Ogre::Vector3::ZERO;
  point_ = point;

  std::mt19937 gen(std::random_device{}());
  std::independent_bits_engine<std::mt19937, 8, uint8_t> bit_eng(gen);
  std::generate(uuid_.begin(), uuid_.end(), bit_eng);
}

std::array<uint8_t, 16> InteractiveObject::uuid() const
{
  return uuid_;
}

void InteractiveObject::twist(geometry_msgs::msg::Twist & twist) const
{
  twist.linear.x = velocity_.length();
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;
}

void InteractiveObject::transform(tf2::Transform & tf_map2object) const
{
  tf2::Transform tf_object_origin2moved_object;
  tf2::Transform tf_map2object_origin;

  {
    geometry_msgs::msg::Transform ros_object_origin2moved_object;
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, theta_);
    ros_object_origin2moved_object.rotation = tf2::toMsg(quat);
    tf2::fromMsg(ros_object_origin2moved_object, tf_object_origin2moved_object);
  }

  {
    geometry_msgs::msg::Transform ros_object_map2object_origin;
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, 0.0);
    ros_object_map2object_origin.rotation = tf2::toMsg(quat);
    ros_object_map2object_origin.translation.x = point_.x;
    ros_object_map2object_origin.translation.y = point_.y;
    ros_object_map2object_origin.translation.z = point_.z;
    tf2::fromMsg(ros_object_map2object_origin, tf_map2object_origin);
  }

  tf_map2object = tf_map2object_origin * tf_object_origin2moved_object;
}

void InteractiveObject::update(const Ogre::Vector3 & point)
{
  velocity_ = (point - point_) * 0.05;
  point_ += velocity_;
  theta_ = (std::abs(velocity_.x) < 1.0e-6 && std::abs(velocity_.y) < 1.0e-6)
             ? theta_
             : std::atan2(velocity_.y, velocity_.x);
}

void InteractiveObject::reset()
{
  velocity_ = Ogre::Vector3::ZERO;
}

double InteractiveObject::distance(const Ogre::Vector3 & point)
{
  return point_.distance(point);
}

InteractiveObjectCollection::InteractiveObjectCollection()
{
  target_ = nullptr;
}

void InteractiveObjectCollection::select(const Ogre::Vector3 & point)
{
  const size_t index = nearest(point);
  if (index != objects_.size()) {
    target_ = objects_[index].get();
  }
}

boost::optional<std::array<uint8_t, 16>> InteractiveObjectCollection::reset()
{
  if (!target_) {
    return {};
  }

  const auto uuid = target_->uuid();
  target_->reset();
  target_ = nullptr;

  return uuid;
}

boost::optional<std::array<uint8_t, 16>> InteractiveObjectCollection::create(
  const Ogre::Vector3 & point)
{
  objects_.emplace_back(std::make_unique<InteractiveObject>(point));
  target_ = objects_.back().get();

  if (target_) {
    return target_->uuid();
  }

  return {};
}

boost::optional<std::array<uint8_t, 16>> InteractiveObjectCollection::remove(
  const Ogre::Vector3 & point)
{
  const size_t index = nearest(point);
  if (index != objects_.size()) {
    const auto removed_uuid = objects_[index].get()->uuid();
    objects_.erase(objects_.begin() + index);
    return removed_uuid;
  }

  return {};
}

boost::optional<std::array<uint8_t, 16>> InteractiveObjectCollection::update(
  const Ogre::Vector3 & point)
{
  if (target_) {
    target_->update(point);
    return target_->uuid();
  }

  return {};
}

boost::optional<geometry_msgs::msg::Twist> InteractiveObjectCollection::twist(
  const std::array<uint8_t, 16> & uuid) const
{
  for (const auto & object : objects_) {
    if (object->uuid() != uuid) {
      continue;
    }

    geometry_msgs::msg::Twist ret;
    object->twist(ret);
    return ret;
  }

  return {};
}

boost::optional<tf2::Transform> InteractiveObjectCollection::transform(
  const std::array<uint8_t, 16> & uuid) const
{
  for (const auto & object : objects_) {
    if (object->uuid() != uuid) {
      continue;
    }

    tf2::Transform ret;
    object->transform(ret);
    return ret;
  }

  return {};
}

size_t InteractiveObjectCollection::nearest(const Ogre::Vector3 & point)
{
  const size_t npos = objects_.size();
  if (objects_.empty()) {
    return npos;
  }

  std::vector<double> distances;
  for (const auto & object : objects_) {
    distances.push_back(object->distance(point));
  }

  const auto compare = [&](int x, int y) { return distances[x] < distances[y]; };
  std::vector<size_t> indices(distances.size());
  std::iota(indices.begin(), indices.end(), 0);
  std::sort(indices.begin(), indices.end(), compare);

  const size_t index = indices[0];
  return distances[index] < 2.0 ? index : npos;
}

void InteractiveObjectTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("2D Dummy Object");
  updateTopic();
}

void InteractiveObjectTool::updateTopic()
{
  rclcpp::Node::SharedPtr raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  dummy_object_info_pub_ = raw_node->create_publisher<Object>(topic_property_->getStdString(), 1);
  clock_ = raw_node->get_clock();
  move_tool_.initialize(context_);
  property_frame_->setFrameManager(context_->getFrameManager());
}

void InteractiveObjectTool::onPoseSet(double x, double y, double theta)
{
  if (enable_interactive_property_->getBool()) {
    return;
  }

  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);

  auto output_msg = createObjectMsg();
  output_msg.initial_state.pose_covariance.pose.position.x = x;
  output_msg.initial_state.pose_covariance.pose.position.y = y;
  output_msg.initial_state.pose_covariance.pose.position.z = position_z_->getFloat();
  output_msg.initial_state.pose_covariance.pose.orientation = tf2::toMsg(quat);
  output_msg.initial_state.twist_covariance.twist.linear.x = velocity_->getFloat();
  output_msg.initial_state.twist_covariance.twist.linear.y = 0.0;
  output_msg.initial_state.twist_covariance.twist.linear.z = 0.0;
  output_msg.initial_state.accel_covariance.accel.linear.x = accel_->getFloat();
  output_msg.initial_state.accel_covariance.accel.linear.y = 0.0;
  output_msg.initial_state.accel_covariance.accel.linear.z = 0.0;
  output_msg.max_velocity = max_velocity_->getFloat();
  output_msg.min_velocity = min_velocity_->getFloat();
  output_msg.action = Object::ADD;

  dummy_object_info_pub_->publish(output_msg);
}

void InteractiveObjectTool::publishObjectMsg(
  const std::array<uint8_t, 16> & uuid, const uint32_t action)
{
  auto output_msg = createObjectMsg();
  output_msg.action = action;
  output_msg.id.uuid = uuid;

  if (action == Object::DELETE) {
    dummy_object_info_pub_->publish(output_msg);
    return;
  }

  const auto object_tf = objects_.transform(uuid);
  const auto object_twist = objects_.twist(uuid);

  if (!object_tf || !object_twist) {
    return;
  }

  tf2::toMsg(object_tf.get(), output_msg.initial_state.pose_covariance.pose);
  output_msg.initial_state.twist_covariance.twist = object_twist.get();

  dummy_object_info_pub_->publish(output_msg);
}

int InteractiveObjectTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  if (!enable_interactive_property_->getBool()) {
    return PoseTool::processMouseEvent(event);
  }

  if (event.rightDown()) {
    const auto point = get_point_from_mouse(event);
    if (point) {
      if (event.shift()) {
        const auto uuid = objects_.create(point.value());
        publishObjectMsg(uuid.get(), Object::ADD);
      } else if (event.alt()) {
        const auto uuid = objects_.remove(point.value());
        publishObjectMsg(uuid.get(), Object::DELETE);
      } else {
        objects_.select(point.value());
      }
    }
    return 0;
  }

  if (event.rightUp()) {
    const auto uuid = objects_.reset();
    publishObjectMsg(uuid.get(), Object::MODIFY);
    return 0;
  }

  if (event.right()) {
    const auto point = get_point_from_mouse(event);
    if (point) {
      const auto uuid = objects_.update(point.value());
      publishObjectMsg(uuid.get(), Object::MODIFY);
    }
    return 0;
  }

  return move_tool_.processMouseEvent(event);
}

int InteractiveObjectTool::processKeyEvent(QKeyEvent * event, rviz_common::RenderPanel * panel)
{
  PoseTool::processKeyEvent(event, panel);
  return move_tool_.processKeyEvent(event, panel);
}
}  // end namespace rviz_plugins
