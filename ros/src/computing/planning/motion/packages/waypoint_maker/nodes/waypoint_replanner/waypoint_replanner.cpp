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
#include "waypoint_replanner.h"

namespace waypoint_maker
{

WaypointReplanner::WaypointReplanner()
{
}

WaypointReplanner::~WaypointReplanner()
{
}

void WaypointReplanner::initParameter(const autoware_config_msgs::ConfigWaypointReplanner::ConstPtr& conf)
{
  velocity_max_ = kmph2mps(conf->velocity_max);
  velocity_min_ = kmph2mps(conf->velocity_min);
  accel_limit_ = conf->accel_limit;
  decel_limit_ = conf->decel_limit;
  r_th_ = conf->radius_thresh;
  r_min_ = conf->radius_min;
  lookup_crv_width_ = 5;
  resample_mode_ = conf->resample_mode;
  resample_interval_ = conf->resample_interval;
  replan_curve_mode_ = conf->replan_curve_mode;
  replan_endpoint_mode_ = conf->replan_endpoint_mode;
  overwrite_vmax_mode_ = conf->overwrite_vmax_mode;
  velocity_offset_ = conf->velocity_offset;
  end_point_offset_ = conf->end_point_offset;
  braking_distance_ = conf->braking_distance;
  r_inf_ = 10 * r_th_;
  vel_param_ = calcVelParam(velocity_max_);
}

void WaypointReplanner::changeVelSign(autoware_msgs::Lane& lane, bool positive) const
{
  const int sgn = positive ? 1 : -1;
  for (auto& el : lane.waypoints)
  {
    el.twist.twist.linear.x = sgn * fabs(el.twist.twist.linear.x);
  }
}

int WaypointReplanner::getDirection(const autoware_msgs::Lane& lane) const
{
  if (lane.waypoints.size() < 2)
  {
    return 1;
  }
  const geometry_msgs::Pose& pose = lane.waypoints[0].pose.pose;
  const geometry_msgs::Point& point = lane.waypoints[1].pose.pose.position;
  const geometry_msgs::Point rlt_point(calcRelativeCoordinate(point, pose));
  return rlt_point.x < 0 ? -1 : 1;
}

void WaypointReplanner::replanLaneWaypointVel(autoware_msgs::Lane& lane)
{
  if (lane.waypoints.empty())
  {
    return;
  }
  const int dir = getDirection(lane);
  const unsigned long last = lane.waypoints.size() - 1;
  changeVelSign(lane, true);
  limitVelocityByRange(0, last, 0, velocity_max_, lane);
  if (resample_mode_)
  {
    resampleLaneWaypoint(resample_interval_, lane, dir);
  }
  if (replan_curve_mode_)
  {
    std::vector<double> curve_radius;
    KeyVal curve_list;
    createRadiusList(lane, curve_radius);
    createCurveList(curve_radius, curve_list);
    if (overwrite_vmax_mode_)
    {// set velocity_max for all_point
      setVelocityByRange(0, last, 0, velocity_max_, lane);
    }
    // set velocity by curve
    for (const auto& el : curve_list)
    {
      const double& radius = el.second.second;
      double vmin = velocity_max_ - vel_param_ * (r_th_ - radius);
      vmin = (vmin < velocity_min_) ? velocity_min_ : vmin;
      limitVelocityByRange(el.first, el.second.first, velocity_offset_, vmin, lane);
    }
  }
  // set velocity on start & end of lane
  if (replan_endpoint_mode_)
  {
    const unsigned long zerospeed_start = last - end_point_offset_;
    const unsigned long lowspeed_start = zerospeed_start - braking_distance_;
    raiseVelocityByRange(0, last, 0, velocity_min_, lane);
    limitVelocityByRange(0, 0, 0, velocity_min_, lane);
    limitVelocityByRange(lowspeed_start, last, 0, velocity_min_, lane);
    setVelocityByRange(zerospeed_start, last, 0, 0.0, lane);
  }
  if (dir < 0)
  {
    changeVelSign(lane, false);
  }
}

void WaypointReplanner::resampleLaneWaypoint(const double resample_interval, autoware_msgs::Lane& lane, int dir)
{
  if (lane.waypoints.size() < 2)
  {
    return;
  }
  autoware_msgs::Lane original_lane(lane);
  lane.waypoints.clear();
  lane.waypoints.emplace_back(original_lane.waypoints[0]);
  lane.waypoints.reserve(ceil(1.5 * calcPathLength(original_lane) / resample_interval_));

  for (unsigned long i = 1; i < original_lane.waypoints.size(); i++)
  {
    CbufGPoint curve_point = getCrvPointsOnResample(lane, original_lane, i);
    const std::vector<double> curve_param = calcCurveParam(curve_point);
    lane.waypoints.back().twist.twist = original_lane.waypoints[i - 1].twist.twist;
    lane.waypoints.back().wpstate = original_lane.waypoints[i - 1].wpstate;
    lane.waypoints.back().change_flag = original_lane.waypoints[i - 1].change_flag;
    // if going straight
    if (curve_param.empty())
    {
      resampleOnStraight(curve_point, lane);
    }
    // else if turnning curve
    else
    {
      resampleOnCurve(curve_point[1], curve_param, lane, dir);
    }
  }
  lane.waypoints[0].pose.pose.orientation = lane.waypoints[1].pose.pose.orientation;
  lane.waypoints.back().twist.twist = original_lane.waypoints.back().twist.twist;
  lane.waypoints.back().wpstate = original_lane.waypoints.back().wpstate;
  lane.waypoints.back().change_flag = original_lane.waypoints.back().change_flag;
}

void WaypointReplanner::resampleOnStraight(const CbufGPoint& curve_point, autoware_msgs::Lane& lane)
{
  if (curve_point.size() != 3)
  {
    return;
  }
  autoware_msgs::Waypoint wp(lane.waypoints.back());
  const geometry_msgs::Point& pt = wp.pose.pose.position;
  const double yaw = atan2(curve_point[2].y - curve_point[0].y, curve_point[2].x - curve_point[0].x);
  wp.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

  const std::vector<double> nvec = { curve_point[1].x - pt.x, curve_point[1].y - pt.y, curve_point[1].z - pt.z };
  double dist = sqrt(nvec[0] * nvec[0] + nvec[1] * nvec[1]);
  std::vector<double> resample_vec = nvec;
  const double coeff = resample_interval_ / dist;
  for (auto& el : resample_vec)
  {
    el *= coeff;
  }
  for (; dist > resample_interval_; dist -= resample_interval_)
  {
    wp.pose.pose.position.x += resample_vec[0];
    wp.pose.pose.position.y += resample_vec[1];
    wp.pose.pose.position.z += resample_vec[2];
    lane.waypoints.emplace_back(wp);
  }
}

void WaypointReplanner::resampleOnCurve(const geometry_msgs::Point& target_point,
                                        const std::vector<double>& curve_param, autoware_msgs::Lane& lane, int dir)
{
  if (curve_param.size() != 3)
  {
    return;
  }
  autoware_msgs::Waypoint wp(lane.waypoints.back());
  const double& cx = curve_param[0];
  const double& cy = curve_param[1];
  const double& radius = curve_param[2];
  const double reverse_angle = (dir < 0) ? M_PI : 0.0;

  const geometry_msgs::Point& p0 = wp.pose.pose.position;
  const geometry_msgs::Point& p1 = target_point;
  double theta = fmod(atan2(p1.y - cy, p1.x - cx) - atan2(p0.y - cy, p0.x - cx), 2 * M_PI);
  int sgn = (theta > 0.0) ? (1) : (-1);
  if (fabs(theta) > M_PI)
  {
    theta -= 2 * sgn * M_PI;
  }
  sgn = (theta > 0.0) ? (1) : (-1);
  // interport
  double t = atan2(p0.y - cy, p0.x - cx);
  double dist = radius * fabs(theta);
  const double resample_dz = resample_interval_ * (p1.z - p0.z) / dist;
  for (; dist > resample_interval_; dist -= resample_interval_)
  {
    if (lane.waypoints.size() == lane.waypoints.capacity())
    {
      break;
    }
    t += sgn * resample_interval_ / radius;
    const double yaw = fmod(t + sgn * M_PI / 2.0, 2 * M_PI) + reverse_angle;
    wp.pose.pose.position.x = cx + radius * cos(t);
    wp.pose.pose.position.y = cy + radius * sin(t);
    wp.pose.pose.position.z += resample_dz;
    wp.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    lane.waypoints.emplace_back(wp);
  }
}

// Three points used for curve detection (the target point is the center)
// [0] = previous point, [1] = target point, [2] = next point
const CbufGPoint WaypointReplanner::getCrvPointsOnResample(
    const autoware_msgs::Lane& lane, const autoware_msgs::Lane& original_lane, unsigned long original_index) const
{
  unsigned long id = original_index;
  CbufGPoint curve_point(3);
  const unsigned int n = (lookup_crv_width_ - 1) / 2;
  const autoware_msgs::Waypoint cp[3] = {
    (lane.waypoints.size() < n) ? lane.waypoints.front() : lane.waypoints[lane.waypoints.size() - n],
    original_lane.waypoints[id],
    (id < original_lane.waypoints.size() - n) ? original_lane.waypoints[id + n] : original_lane.waypoints.back()
  };
  for (int i = 0; i < 3; i++)
  {
    curve_point.push_back(cp[i].pose.pose.position);
  }
  return curve_point;
}

const CbufGPoint WaypointReplanner::getCrvPoints(const autoware_msgs::Lane& lane, unsigned long index) const
{
  CbufGPoint curve_point(3);
  const unsigned int n = (lookup_crv_width_ - 1) / 2;
  const unsigned long curve_index[3] = { (index < n) ? 0 : (index - n), index, (index >= lane.waypoints.size() - n) ?
                                                                                   (lane.waypoints.size() - 1) :
                                                                                   (index + n) };
  for (int i = 0; i < 3; i++)
  {
    curve_point.push_back(lane.waypoints[curve_index[i]].pose.pose.position);
  }
  return curve_point;
}

void WaypointReplanner::createRadiusList(const autoware_msgs::Lane& lane, std::vector<double>& curve_radius)
{
  if (lane.waypoints.empty())
  {
    return;
  }
  curve_radius.resize(lane.waypoints.size());
  curve_radius.at(0) = curve_radius.back() = r_inf_;

  for (unsigned long i = 1; i < lane.waypoints.size() - 1; i++)
  {
    CbufGPoint curve_point(getCrvPoints(lane, i));
    const std::vector<double> curve_param(calcCurveParam(curve_point));

    // if going straight
    if (curve_param.empty())
    {
      curve_radius.at(i) = r_inf_;
    }
    // else if turnning curve
    else
    {
      curve_radius.at(i) = (curve_param[2] > r_inf_) ? r_inf_ : curve_param[2];
    }
  }
}

const double WaypointReplanner::calcVelParam(double vmax) const
{
  if (fabs(r_th_ - r_min_) < 1e-8)
  {
    return DBL_MAX;  // error
  }
  return (vmax - velocity_min_) / (r_th_ - r_min_);
}

void WaypointReplanner::createCurveList(const std::vector<double>& curve_radius, KeyVal& curve_list)
{
  unsigned long index = 0;
  bool on_curve = false;
  double radius_localmin = DBL_MAX;
  for (unsigned long i = 1; i < curve_radius.size(); i++)
  {
    if (!on_curve && curve_radius[i] <= r_th_ && curve_radius[i - 1] > r_th_)
    {
      index = i;
      on_curve = true;
    }
    else if (on_curve && curve_radius[i - 1] <= r_th_ && curve_radius[i] > r_th_)
    {
      on_curve = false;
      if (radius_localmin < r_min_)
      {
        radius_localmin = r_min_;
      }
      curve_list[index] = std::make_pair(i, radius_localmin);
      radius_localmin = DBL_MAX;
    }
    if (!on_curve)
    {
      continue;
    }
    if (radius_localmin > curve_radius[i])
    {
      radius_localmin = curve_radius[i];
    }
  }
}


void WaypointReplanner::setVelocityByRange(unsigned long start_idx, unsigned long end_idx, unsigned int offset,
                                             double vel, autoware_msgs::Lane& lane)
{
  if (lane.waypoints.empty())
  {
    return;
  }
  if (offset > 0)
  {
    start_idx = (start_idx > offset) ? (start_idx - offset) : 0;
    end_idx = (end_idx > offset) ? (end_idx - offset) : 0;
  }
  end_idx = (end_idx >= lane.waypoints.size()) ? lane.waypoints.size() - 1 : end_idx;
  for (unsigned long idx = start_idx; idx <= end_idx; idx++)
  {
    lane.waypoints[idx].twist.twist.linear.x = vel;
  }
}

void WaypointReplanner::raiseVelocityByRange(unsigned long start_idx, unsigned long end_idx, unsigned int offset,
                                           double vmin, autoware_msgs::Lane& lane)
{
  if (lane.waypoints.empty())
  {
    return;
  }
  if (offset > 0)
  {
    start_idx = (start_idx > offset) ? (start_idx - offset) : 0;
    end_idx = (end_idx > offset) ? (end_idx - offset) : 0;
  }
  end_idx = (end_idx >= lane.waypoints.size()) ? lane.waypoints.size() - 1 : end_idx;
  for (unsigned long idx = start_idx; idx <= end_idx; idx++)
  {
    if (lane.waypoints[idx].twist.twist.linear.x >= vmin)
    {
      continue;
    }
    lane.waypoints[idx].twist.twist.linear.x = vmin;
  }
}

void WaypointReplanner::limitVelocityByRange(unsigned long start_idx, unsigned long end_idx, unsigned int offset,
                                             double vmin, autoware_msgs::Lane& lane)
{
  if (lane.waypoints.empty())
  {
    return;
  }
  if (offset > 0)
  {
    start_idx = (start_idx > offset) ? (start_idx - offset) : 0;
    end_idx = (end_idx > offset) ? (end_idx - offset) : 0;
  }
  end_idx = (end_idx >= lane.waypoints.size()) ? lane.waypoints.size() - 1 : end_idx;
  for (unsigned long idx = start_idx; idx <= end_idx; idx++)
  {
    if (lane.waypoints[idx].twist.twist.linear.x < vmin)
    {
      continue;
    }
    lane.waypoints[idx].twist.twist.linear.x = vmin;
  }
  limitAccelDecel(start_idx, lane);
  limitAccelDecel(end_idx, lane);
}

void WaypointReplanner::limitAccelDecel(const unsigned long idx, autoware_msgs::Lane& lane)
{
  const double acc[2] = { accel_limit_, decel_limit_ };
  const unsigned long end_idx[2] = { lane.waypoints.size() - idx, idx + 1 };
  const int sgn[2] = { 1, -1 };
  for (int j = 0; j < 2; j++)  // [j=0]: accel_limit_process, [j=1]: decel_limit_process
  {
    double v = lane.waypoints[idx].twist.twist.linear.x;
    unsigned long next = idx + sgn[j];
    for (unsigned long i = 1; i < end_idx[j]; i++, next += sgn[j])
    {
      const geometry_msgs::Point& p0 = lane.waypoints[next - sgn[j]].pose.pose.position;
      const geometry_msgs::Point& p1 = lane.waypoints[next].pose.pose.position;
      const double dist = std::hypot(p0.x - p1.x, p0.y - p1.y);
      v = sqrt(2 * acc[j] * dist + v * v);
      if (v > velocity_max_ || v > lane.waypoints[next].twist.twist.linear.x)
      {
        break;
      }
      lane.waypoints[next].twist.twist.linear.x = v;
    }
  }
}

// get curve 3-Parameter [center_x, center_y, radius] with 3 point input. If error occured, return empty vector.
const std::vector<double> WaypointReplanner::calcCurveParam(CbufGPoint p) const
{
  for (int i = 0; i < 3; i++, p.push_back(p.front()))  // if exception occured, change points order
  {
    const double d = 2 * ((p[0].y - p[2].y) * (p[0].x - p[1].x) - (p[0].y - p[1].y) * (p[0].x - p[2].x));
    if (fabs(d) < 1e-8)
    {
      continue;
    }
    const std::vector<double> x2 = { p[0].x * p[0].x, p[1].x * p[1].x, p[2].x * p[2].x };
    const std::vector<double> y2 = { p[0].y * p[0].y, p[1].y * p[1].y, p[2].y * p[2].y };
    const double a = y2[0] - y2[1] + x2[0] - x2[1];
    const double b = y2[0] - y2[2] + x2[0] - x2[2];
    std::vector<double> param(3);
    const double cx = param[0] = ((p[0].y - p[2].y) * a - (p[0].y - p[1].y) * b) / d;
    const double cy = param[1] = ((p[0].x - p[2].x) * a - (p[0].x - p[1].x) * b) / -d;
    param[2] = sqrt((cx - p[0].x) * (cx - p[0].x) + (cy - p[0].y) * (cy - p[0].y));
    return param;
  }
  return std::vector<double>();  // error
}

const double WaypointReplanner::calcPathLength(const autoware_msgs::Lane& lane) const
{
  double distance = 0.0;
  for (unsigned long i = 1; i < lane.waypoints.size(); i++)
  {
    const geometry_msgs::Point& p0 = lane.waypoints[i - 1].pose.pose.position;
    const geometry_msgs::Point& p1 = lane.waypoints[i].pose.pose.position;
    tf::Vector3 tf0(p0.x, p0.y, 0.0);
    tf::Vector3 tf1(p1.x, p1.y, 0.0);
    distance += tf::tfDistance(tf0, tf1);
  }
  return distance;
}

};
