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

#ifndef TIER4_PLANNING_RVIZ_PLUGIN__POSE_WITH_UUID_STAMPED__DISPLAY_HPP_
#define TIER4_PLANNING_RVIZ_PLUGIN__POSE_WITH_UUID_STAMPED__DISPLAY_HPP_

#include <rviz_common/message_filter_display.hpp>

#include <autoware_planning_msgs/msg/pose_with_uuid_stamped.hpp>

#include <deque>
#include <memory>
#include <string>

namespace rviz_rendering
{
class Axes;
class MovableText;
}  // namespace rviz_rendering
namespace rviz_common::properties
{
class FloatProperty;
class TfFrameProperty;
}  // namespace rviz_common::properties

namespace rviz_plugins
{
class AutowarePoseWithUuidStampedDisplay
: public rviz_common::MessageFilterDisplay<autoware_planning_msgs::msg::PoseWithUuidStamped>
{
  Q_OBJECT

public:
  AutowarePoseWithUuidStampedDisplay();
  ~AutowarePoseWithUuidStampedDisplay() override;
  AutowarePoseWithUuidStampedDisplay(const AutowarePoseWithUuidStampedDisplay &) = delete;
  AutowarePoseWithUuidStampedDisplay(const AutowarePoseWithUuidStampedDisplay &&) = delete;
  AutowarePoseWithUuidStampedDisplay & operator=(const AutowarePoseWithUuidStampedDisplay &) =
    delete;
  AutowarePoseWithUuidStampedDisplay & operator=(const AutowarePoseWithUuidStampedDisplay &&) =
    delete;

protected:
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;

private Q_SLOTS:
  void updateVisualization();

private:
  void subscribe() override;
  void unsubscribe() override;
  void processMessage(
    const autoware_planning_msgs::msg::PoseWithUuidStamped::ConstSharedPtr meg_ptr) override;

  std::unique_ptr<rviz_rendering::Axes> axes_;
  std::unique_ptr<Ogre::SceneNode> uuid_node_;
  rviz_rendering::MovableText * uuid_;

  rviz_common::properties::FloatProperty * length_property_;
  rviz_common::properties::FloatProperty * radius_property_;
  rviz_common::properties::TfFrameProperty * frame_property_;

  rviz_common::properties::BoolProperty * uuid_text_view_property_;
  rviz_common::properties::FloatProperty * uuid_text_scale_property_;

  autoware_planning_msgs::msg::PoseWithUuidStamped::ConstSharedPtr last_msg_ptr_;
};

}  // namespace rviz_plugins

#endif  // TIER4_PLANNING_RVIZ_PLUGIN__POSE_WITH_UUID_STAMPED__DISPLAY_HPP_
