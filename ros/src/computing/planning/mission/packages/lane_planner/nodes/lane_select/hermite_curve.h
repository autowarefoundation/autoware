/*
 *  Copyright (c) 2015, Nagoya University

 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

#include "autoware_msgs/waypoint.h"

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
std::vector<autoware_msgs::waypoint> generateHermiteCurveForROS(const geometry_msgs::Pose &start,
                                                                const geometry_msgs::Pose &end, const double velocity,
                                                                const double vlength);
void createVectorFromPose(const geometry_msgs::Pose &p, tf::Vector3 *v);
}  // namespace
#endif  // HERMITE_CURVE_H
