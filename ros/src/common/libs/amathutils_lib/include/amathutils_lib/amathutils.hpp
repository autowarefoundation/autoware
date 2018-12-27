#ifndef __AMATHUTILS_HPP
#define __AMATHUTILS_HPP

#include <cmath>
#include <iostream>

// ROS Messages
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>

using namespace std;

namespace amathutils
{
#define G_MPSS 9.80665  // m/s^2

inline double rad2deg(double _angle)
{
  return _angle * 180 / M_PI;
}
inline double deg2rad(double _angle)
{
  return _angle / 180 * M_PI;
}

inline double mps2kmph(double _mpsval)
{
  return (_mpsval * 3.6);  // mps * 60sec * 60minutes / 1000m
}

inline double kmph2mps(double _kmphval)
{
  return (_kmphval * 1000 / 60 / 60);  // kmph * 1000m / 60sec / 60sec
}

inline double getGravityAcceleration(double _acceleration_mpss)
{
  return _acceleration_mpss / G_MPSS;
}

inline double getAcceleration(double _v0, double _v, double _x)
{
  return (_v * _v - _v0 * _v0) / 2 / _x;
}

inline double getTimefromAcceleration(double _v0, double _v, double _a)
{
  return (_v - _v0) / _a;
}

geometry_msgs::Point getNearPtOnLine(const geometry_msgs::Point &_p, const geometry_msgs::Point &_a,
                                     const geometry_msgs::Point &_b);
double find_distance(const geometry_msgs::Point &_from, const geometry_msgs::Point &_to);
double find_distance(const geometry_msgs::Pose &_from, const geometry_msgs::Pose &_to);
double find_angle(const geometry_msgs::Point &_from, const geometry_msgs::Point &_to);
bool isIntersectLine(const geometry_msgs::Point &_l1_p1, const geometry_msgs::Point &_l1_p2,
                     const geometry_msgs::Point &_l2_p1, const geometry_msgs::Point &_l2_p2);
int isPointLeftFromLine(const geometry_msgs::Point &_target, const geometry_msgs::Point &_line_p1,
                        const geometry_msgs::Point &_line_p2);
double getPoseYawAngle(const geometry_msgs::Pose &_pose);
double calcPosesAngleDiffRaw(const geometry_msgs::Pose &p_from, const geometry_msgs::Pose &_p_to);
double radianNormalize(double _angle);
double calcPosesAngleDiffDeg(const geometry_msgs::Pose &_p_from, const geometry_msgs::Pose &_p_to);
double calcPosesAngleDiffRad(const geometry_msgs::Pose &_p_from, const geometry_msgs::Pose &_p_to);
}
#endif
