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

#ifndef PATH__DISPLAY_HPP_
#define PATH__DISPLAY_HPP_

#include <path/display_base.hpp>

#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>

#include <memory>
#include <utility>
#include <vector>

namespace
{
bool validateBoundFloats(const std::vector<geometry_msgs::msg::Point> & bound)
{
  for (auto && point : bound) {
    if (!rviz_common::validateFloats(point)) {
      return false;
    }
  }
  return true;
}

template <typename T>
bool validateBoundFloats(const typename T::ConstSharedPtr & msg_ptr)
{
  return validateBoundFloats(msg_ptr->left_bound) && validateBoundFloats(msg_ptr->right_bound);
}

void visualizeBound(
  const std::vector<geometry_msgs::msg::Point> & bound, const Ogre::ColourValue & color,
  const float width, Ogre::ManualObject * bound_object)
{
  if (bound.size() < 2) {
    return;
  }

  bound_object->estimateVertexCount(bound.size() * 2);
  bound_object->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_STRIP);

  for (size_t i = 0; i < bound.size(); ++i) {
    const auto & curr_p = i == bound.size() - 1 ? bound.at(i - 1) : bound.at(i);
    const auto & next_p = i == bound.size() - 1 ? bound.at(i) : bound.at(i + 1);
    const auto yaw = tier4_autoware_utils::calcAzimuthAngle(curr_p, next_p);
    const auto x_offset = static_cast<float>(width * 0.5 * std::sin(yaw));
    const auto y_offset = static_cast<float>(width * 0.5 * std::cos(yaw));
    auto target_lp = bound.at(i);
    target_lp.x = target_lp.x - x_offset;
    target_lp.y = target_lp.y + y_offset;
    target_lp.z = target_lp.z + 0.5;
    bound_object->position(target_lp.x, target_lp.y, target_lp.z);
    bound_object->colour(color);
    auto target_rp = bound.at(i);
    target_rp.x = target_rp.x + x_offset;
    target_rp.y = target_rp.y - y_offset;
    target_rp.z = target_rp.z + 0.5;
    bound_object->position(target_rp.x, target_rp.y, target_rp.z);
    bound_object->colour(color);
  }
  bound_object->end();
}
}  // namespace

namespace rviz_plugins
{
template <class T>
class AutowarePathWithDrivableAreaDisplay : public AutowarePathBaseDisplay<T>
{
public:
  AutowarePathWithDrivableAreaDisplay()
  : property_drivable_area_view_{"View Drivable Area", true, "", this},
    property_drivable_area_alpha_{"Alpha", 0.999, "", &property_drivable_area_view_},
    property_drivable_area_color_{"Color", QColor(0, 148, 205), "", &property_drivable_area_view_},
    property_drivable_area_width_{"Width", 0.3f, "", &property_drivable_area_view_}
  {
    property_drivable_area_alpha_.setMin(0.0);
    property_drivable_area_alpha_.setMax(1.0);
    property_drivable_area_width_.setMin(0.001);
  }

  ~AutowarePathWithDrivableAreaDisplay()
  {
    if (this->initialized()) {
      this->scene_manager_->destroyManualObject(left_bound_object_);
      this->scene_manager_->destroyManualObject(right_bound_object_);
    }
  }
  void onInitialize() override
  {
    AutowarePathBaseDisplay<T>::onInitialize();
    left_bound_object_ = this->scene_manager_->createManualObject();
    right_bound_object_ = this->scene_manager_->createManualObject();
    left_bound_object_->setDynamic(true);
    right_bound_object_->setDynamic(true);
    this->scene_node_->attachObject(left_bound_object_);
    this->scene_node_->attachObject(right_bound_object_);
  }

  void reset() override
  {
    AutowarePathBaseDisplay<T>::reset();
    left_bound_object_->clear();
    right_bound_object_->clear();
  }

protected:
  void visualizeDrivableArea(const typename T::ConstSharedPtr msg_ptr) override
  {
    left_bound_object_->clear();
    right_bound_object_->clear();

    if (!validateBoundFloats<T>(msg_ptr)) {
      this->setStatus(
        rviz_common::properties::StatusProperty::Error, "Topic",
        "Message drivable area contained invalid floating point values (nans or infs)");
      return;
    }

    if (property_drivable_area_view_.getBool()) {
      Ogre::ColourValue color =
        rviz_common::properties::qtToOgre(property_drivable_area_color_.getColor());
      color.a = property_drivable_area_alpha_.getFloat();
      const auto line_width = property_drivable_area_width_.getFloat();
      visualizeBound(msg_ptr->left_bound, color, line_width, left_bound_object_);
      visualizeBound(msg_ptr->right_bound, color, line_width, right_bound_object_);
    }
  }

private:
  Ogre::ManualObject * left_bound_object_{nullptr};
  Ogre::ManualObject * right_bound_object_{nullptr};

  rviz_common::properties::BoolProperty property_drivable_area_view_;
  rviz_common::properties::FloatProperty property_drivable_area_alpha_;
  rviz_common::properties::ColorProperty property_drivable_area_color_;
  rviz_common::properties::FloatProperty property_drivable_area_width_;
};

class AutowarePathWithLaneIdDisplay
: public AutowarePathWithDrivableAreaDisplay<autoware_auto_planning_msgs::msg::PathWithLaneId>
{
  Q_OBJECT
public:
  AutowarePathWithLaneIdDisplay();
  ~AutowarePathWithLaneIdDisplay();

private:
  void preprocessMessageDetail(
    const autoware_auto_planning_msgs::msg::PathWithLaneId::ConstSharedPtr msg_ptr) override;
  void processMessageDetail(
    const autoware_auto_planning_msgs::msg::PathWithLaneId::ConstSharedPtr msg_ptr,
    const size_t p_idx) override;

  rviz_common::properties::BoolProperty property_lane_id_view_;
  rviz_common::properties::FloatProperty property_lane_id_scale_;

  using LaneIdObject =
    std::pair<std::unique_ptr<Ogre::SceneNode>, std::unique_ptr<rviz_rendering::MovableText>>;
  std::vector<LaneIdObject> lane_id_obj_ptrs_;
};

class AutowarePathDisplay
: public AutowarePathWithDrivableAreaDisplay<autoware_auto_planning_msgs::msg::Path>
{
  Q_OBJECT
};

class AutowareTrajectoryDisplay
: public AutowarePathBaseDisplay<autoware_auto_planning_msgs::msg::Trajectory>
{
  Q_OBJECT
};
}  // namespace rviz_plugins

#endif  // PATH__DISPLAY_HPP_
