// Copyright 2022 TIER IV, Inc.
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

/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "obstacle_cruise_planner/optimization_based_planner/box2d.hpp"

#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include <tf2/utils.h>

Box2d::Box2d(const geometry_msgs::msg::Pose & center_pose, const double length, const double width)
: center_(center_pose.position),
  length_(length),
  width_(width),
  half_length_(length / 2.0),
  half_width_(width / 2.0)
{
  max_x_ = std::numeric_limits<double>::min();
  max_y_ = std::numeric_limits<double>::min();
  min_x_ = std::numeric_limits<double>::max();
  min_y_ = std::numeric_limits<double>::max();
  heading_ = tf2::getYaw(center_pose.orientation);
  cos_heading_ = std::cos(heading_);
  sin_heading_ = std::sin(heading_);
  initCorners();
}

void Box2d::initCorners()
{
  const double dx1 = cos_heading_ * half_length_;
  const double dy1 = sin_heading_ * half_length_;
  const double dx2 = sin_heading_ * half_width_;
  const double dy2 = -cos_heading_ * half_width_;

  const auto p1 =
    tier4_autoware_utils::createPoint(center_.x + dx1 + dx2, center_.y + dy1 + dy2, center_.z);
  const auto p2 =
    tier4_autoware_utils::createPoint(center_.x + dx1 - dx2, center_.y + dy1 - dy2, center_.z);
  const auto p3 =
    tier4_autoware_utils::createPoint(center_.x - dx1 - dx2, center_.y - dy1 - dy2, center_.z);
  const auto p4 =
    tier4_autoware_utils::createPoint(center_.x - dx1 + dx2, center_.y - dy1 + dy2, center_.z);
  corners_.clear();
  corners_.resize(4);
  corners_.at(0) = p1;
  corners_.at(1) = p2;
  corners_.at(2) = p3;
  corners_.at(3) = p4;

  for (auto & corner : corners_) {
    max_x_ = std::fmax(corner.x, max_x_);
    min_x_ = std::fmin(corner.x, min_x_);
    max_y_ = std::fmax(corner.y, max_y_);
    min_y_ = std::fmin(corner.y, min_y_);
  }
}

bool Box2d::hasOverlap(const Box2d & box) const
{
  if (
    box.getMaxX() < getMinX() || box.getMinX() > getMaxX() || box.getMaxY() < getMinY() ||
    box.getMinY() > getMaxY()) {
    return false;
  }

  const double shift_x = box.getCenterX() - center_.x;
  const double shift_y = box.getCenterY() - center_.y;

  const double dx1 = cos_heading_ * half_length_;
  const double dy1 = sin_heading_ * half_length_;
  const double dx2 = sin_heading_ * half_width_;
  const double dy2 = -cos_heading_ * half_width_;
  const double dx3 = box.getCosHeading() * box.getHalfLength();
  const double dy3 = box.getSinHeading() * box.getHalfLength();
  const double dx4 = box.getSinHeading() * box.getHalfWidth();
  const double dy4 = -box.getCosHeading() * box.getHalfWidth();

  return std::abs(shift_x * cos_heading_ + shift_y * sin_heading_) <=
           std::abs(dx3 * cos_heading_ + dy3 * sin_heading_) +
             std::abs(dx4 * cos_heading_ + dy4 * sin_heading_) + half_length_ &&
         std::abs(shift_x * sin_heading_ - shift_y * cos_heading_) <=
           std::abs(dx3 * sin_heading_ - dy3 * cos_heading_) +
             std::abs(dx4 * sin_heading_ - dy4 * cos_heading_) + half_width_ &&
         std::abs(shift_x * box.getCosHeading() + shift_y * box.getSinHeading()) <=
           std::abs(dx1 * box.getCosHeading() + dy1 * box.getSinHeading()) +
             std::abs(dx2 * box.getCosHeading() + dy2 * box.getSinHeading()) +
             box.getHalfLength() &&
         std::abs(shift_x * box.getSinHeading() - shift_y * box.getCosHeading()) <=
           std::abs(dx1 * box.getSinHeading() - dy1 * box.getCosHeading()) +
             std::abs(dx2 * box.getSinHeading() - dy2 * box.getCosHeading()) + box.getHalfWidth();
}
