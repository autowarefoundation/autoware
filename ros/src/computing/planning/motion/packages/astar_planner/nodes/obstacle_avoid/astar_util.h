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

#ifndef ASTAR_UTIL_H
#define ASTAR_UTIL_H

#include <tf/transform_listener.h>

namespace astar_planner
{
enum class STATUS : uint8_t
{
  NONE,
  OPEN,
  CLOSED,
  OBS
};

struct AstarNode
{
  double x, y, theta;            // Coordinate of each node
  STATUS status = STATUS::NONE;  // NONE, OPEN, CLOSED or OBS
  double gc = 0;                 // Actual cost
  double hc = 0;                 // heuristic cost
  double move_distance = 0;      // actual move distance
  bool back;                     // true if the current direction of the vehicle is back
  AstarNode *parent = NULL;      // parent node
};

struct WaveFrontNode
{
  int index_x;
  int index_y;
  double hc;

  WaveFrontNode();
  WaveFrontNode(int x, int y, double cost);
};

struct NodeUpdate
{
  double shift_x;
  double shift_y;
  double rotation;
  double step;
  int index_theta;
  bool curve;
  bool back;
};

// For open list and goal list
struct SimpleNode
{
  int index_x;
  int index_y;
  int index_theta;
  double cost;

  bool operator>(const SimpleNode &right) const
  {
    return cost > right.cost;
  }

  SimpleNode();
  SimpleNode(int x, int y, int theta, double gc, double hc);
};

inline double calcDistance(double x1, double y1, double x2, double y2)
{
  return std::hypot(x2 - x1, y2 - y1);
}

inline double modifyTheta(double theta)
{
  if (theta < 0)
    return theta + 2 * M_PI;
  if (theta >= 2 * M_PI)
    return theta - 2 * M_PI;

  return theta;
}

inline geometry_msgs::Pose transformPose(geometry_msgs::Pose &pose, tf::Transform &tf)
{
  // Convert ROS pose to TF pose
  tf::Pose tf_pose;
  tf::poseMsgToTF(pose, tf_pose);

  // Transform pose
  tf_pose = tf * tf_pose;

  // Convert TF pose to ROS pose
  geometry_msgs::Pose ros_pose;
  tf::poseTFToMsg(tf_pose, ros_pose);

  return ros_pose;
}

inline WaveFrontNode getWaveFrontNode(int x, int y, double cost)
{
  WaveFrontNode node(x, y, cost);

  return node;
}

inline geometry_msgs::Point calcRelativeCoordinate(geometry_msgs::Pose pose, tf::Point point)
{
  tf::Transform transform;
  tf::poseMsgToTF(pose, transform);
  transform = transform.inverse();

  point = transform * point;
  geometry_msgs::Point point_msg;
  tf::pointTFToMsg(point, point_msg);

  return point_msg;
}

inline double calcDiffOfRadian(double a, double b)
{
  double diff = std::fabs(a - b);
  if (diff < M_PI)
    return diff;
  else
    return 2 * M_PI - diff;
}

inline geometry_msgs::Pose xytToPoseMsg(double x, double y, double theta)
{
  geometry_msgs::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.orientation = tf::createQuaternionMsgFromYaw(theta);

  return p;
}

}  // namespace astar_planner

#endif
