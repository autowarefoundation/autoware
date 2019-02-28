/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef HERMITE_CURVE_H
#define HERMITE_CURVE_H

// C++ includes
#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>

// ROS includes
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include "autoware_msgs/Waypoint.h"

namespace lane_planner
{
struct Element2D
{
  Element2D(double x, double y)
  {
    this->x = x;
    this->y = y;
  }
  void set(double x, double y)
  {
    this->x = x;
    this->y = y;
  }
  double x;
  double y;
};

std::vector<Element2D> generateHermiteCurve(const Element2D &p0, const Element2D &v0, const Element2D &p1,
                                            const Element2D &v1, const double vlength = 20);
std::vector<autoware_msgs::Waypoint> generateHermiteCurveForROS(const geometry_msgs::Pose &start,
                                                                const geometry_msgs::Pose &end, const double velocity,
                                                                const double vlength);
void createVectorFromPose(const geometry_msgs::Pose &p, tf::Vector3 *v);
}  // namespace
#endif  // HERMITE_CURVE_H
