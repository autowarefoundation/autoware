/*
 *  Copyright (c) 2018, TierIV, Inc.

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
#include "waypoint_replanner_core.h"

namespace waypoint_maker
{
inline double kmph2mps(double velocity_kmph)
{
  return (velocity_kmph * 1000) / (60 * 60);
}
inline double mps2kmph(double velocity_mps)
{
  return (velocity_mps * 60 * 60) / 1000;
}

WaypointReplannerCore::WaypointReplannerCore() : private_nh_("~")
{
}

WaypointReplannerCore::~WaypointReplannerCore()
{
}

void WaypointReplannerCore::initParameter(const autoware_msgs::ConfigWaypointReplanner::ConstPtr& conf)
{
  velocity_max_ = kmph2mps(conf->velocity_max);
  velocity_min_ = kmph2mps(conf->velocity_min);
  accel_limit_ = conf->accel_limit;
  decel_limit_ = conf->decel_limit;
  r_th_ = conf->radius_thresh;
  r_min_ = conf->radius_min;
  lookup_crv_width_ = 5;
  resample_mode_ = conf->resample_mode;
  decel_curve_mode_ = conf->decel_curve_mode;
  decel_endpoint_mode_ = conf->decel_endpoint_mode;
  resample_interval_ = conf->resample_interval;
  velocity_offset_ = conf->velocity_offset;
  end_point_offset_ = conf->end_point_offset;
  constant_vmax_mode_ = conf->constant_vmax_mode;
  r_inf_ = 10 * r_th_;
}

void WaypointReplannerCore::changeVelSign(autoware_msgs::lane* lane, bool positive) const
{
  const int sgn = positive ? 1 : -1;
  for (auto& el : lane->waypoints)
  {
    el.twist.twist.linear.x = sgn * fabs(el.twist.twist.linear.x);
  }
}

int WaypointReplannerCore::getDirection(const autoware_msgs::lane& lane) const
{
  return lane.waypoints[0].twist.twist.linear.x < 0 ? -1 : 1;
}

void WaypointReplannerCore::replanLaneWaypointVel(autoware_msgs::lane* lane)
{
  if (lane->waypoints.empty())
  {
    return;
  }
  const int dir = getDirection(*lane);
  changeVelSign(lane, true);
  if (resample_mode_)
  {
    resampleLaneWaypoint(resample_interval_, lane, dir);
  }
  if (decel_curve_mode_)
  {
    std::vector<double> curve_radius;
    std::unordered_map<unsigned long, std::pair<unsigned long, double> > curve_list;
    std::unordered_map<unsigned long, std::pair<unsigned long, double> > vmax_list;
    createRadiusList(*lane, &curve_radius);
    createCurveList(curve_radius, &curve_list);
    createVmaxList(*lane, curve_list, velocity_offset_, &vmax_list);
    // set velocity_max for all_point
    for (auto& el : lane->waypoints)
    {
      el.twist.twist.linear.x = velocity_max_;
    }
    for (const auto& el : vmax_list)
    {
      const double vmax = el.second.second;
      const unsigned long start_idx = el.second.first;
      const unsigned long end_idx =
      (el.first == lane->waypoints.size() - 1) ? el.first : curve_list[el.first].first;
      setVelocityByRange(start_idx, end_idx, velocity_offset_, vmax, lane);
    }
    // set velocity by curve
    for (const auto& el : curve_list)
    {
      const double& vmax = vmax_list[el.first].second;
      const double& radius = el.second.second;
      vel_param_ = calcVelParam(vmax);
      double vmin = vmax - vel_param_ * (r_th_ - radius);
      vmin = (vmin < velocity_min_) ? velocity_min_ : vmin;
      limitVelocityByRange(el.first, el.second.first, velocity_offset_, vmin, lane);
    }
  }
  // set velocity on start & end of lane
  if (decel_endpoint_mode_)
  {
    limitVelocityByRange(0, 0, 0, velocity_min_, lane);
    limitVelocityByRange(lane->waypoints.size() - 1 - end_point_offset_, lane->waypoints.size() - 1, 0, 0.0, lane);
  }
  if (dir < 0)
  {
    changeVelSign(lane, false);
  }
}

void WaypointReplannerCore::resampleLaneWaypoint(const double resample_interval, autoware_msgs::lane* lane, int dir)
{
  if (lane->waypoints.size() < 2)
  {
    return;
  }
  autoware_msgs::lane original_lane = *lane;
  lane->waypoints.clear();
  lane->waypoints.push_back(original_lane.waypoints[0]);
  lane->waypoints.reserve(ceil(1.5 * calcPathLength(original_lane) / resample_interval_));

  for (unsigned long i = 1; i < original_lane.waypoints.size(); i++)
  {
    boost::circular_buffer<geometry_msgs::Point> curve_point = getCrvPointsOnResample(*lane, original_lane, i);
    const std::vector<double> curve_param = calcCurveParam(curve_point);
    lane->waypoints.back().twist.twist = original_lane.waypoints[i - 1].twist.twist;
    lane->waypoints.back().wpstate = original_lane.waypoints[i - 1].wpstate;
    lane->waypoints.back().change_flag = original_lane.waypoints[i - 1].change_flag;
    // if going straight
    if (curve_param.empty())
    {
      resampleOnStraight(curve_point, lane, dir);
    }
    // else if turnning curve
    else
    {
      resampleOnCurve(curve_point[1], curve_param, lane, dir);
    }
  }
  lane->waypoints[0].pose.pose.orientation = lane->waypoints[1].pose.pose.orientation;
  lane->waypoints.back().twist.twist = original_lane.waypoints.back().twist.twist;
  lane->waypoints.back().wpstate = original_lane.waypoints.back().wpstate;
  lane->waypoints.back().change_flag = original_lane.waypoints.back().change_flag;
}

void WaypointReplannerCore::resampleOnStraight(const boost::circular_buffer<geometry_msgs::Point>& curve_point,
                                           autoware_msgs::lane* lane, int dir)
{
  autoware_msgs::waypoint wp = lane->waypoints.back();
  const geometry_msgs::Point& pt = wp.pose.pose.position;
  const double reverse_angle = (dir < 0) ? M_PI : 0.0;
  const double yaw = atan2(curve_point[2].y - curve_point[0].y, curve_point[2].x - curve_point[0].x) + reverse_angle;
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
    lane->waypoints.push_back(wp);
  }
}

void WaypointReplannerCore::resampleOnCurve(const geometry_msgs::Point& target_point,
                                        const std::vector<double>& curve_param, autoware_msgs::lane* lane, int dir)
{
  autoware_msgs::waypoint wp = lane->waypoints.back();
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
    if (lane->waypoints.size() == lane->waypoints.capacity())
    {
      break;
    }
    t += sgn * resample_interval_ / radius;
    const double yaw = fmod(t + sgn * M_PI / 2.0, 2 * M_PI) + reverse_angle;
    wp.pose.pose.position.x = cx + radius * cos(t);
    wp.pose.pose.position.y = cy + radius * sin(t);
    wp.pose.pose.position.z += resample_dz;
    wp.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    lane->waypoints.push_back(wp);
  }
}

// Three points used for curve detection (the target point is the center)
// [0] = previous point, [1] = target point, [2] = next point
const boost::circular_buffer<geometry_msgs::Point> WaypointReplannerCore::getCrvPointsOnResample(
    const autoware_msgs::lane& lane, const autoware_msgs::lane& original_lane, unsigned long original_index) const
{
  unsigned long id = original_index;
  boost::circular_buffer<geometry_msgs::Point> curve_point(3);
  const unsigned int n = (lookup_crv_width_ - 1) / 2;
  const autoware_msgs::waypoint cp[3] = {
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

const boost::circular_buffer<geometry_msgs::Point> WaypointReplannerCore::getCrvPoints(const autoware_msgs::lane& lane,
                                                                                   unsigned long index) const
{
  boost::circular_buffer<geometry_msgs::Point> curve_point(3);
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

void WaypointReplannerCore::createRadiusList(const autoware_msgs::lane& lane, std::vector<double>* curve_radius)
{
  if (lane.waypoints.empty())
  {
    return;
  }
  curve_radius->resize(lane.waypoints.size());
  curve_radius->at(0) = curve_radius->back() = r_inf_;

  for (unsigned long i = 1; i < lane.waypoints.size() - 1; i++)
  {
    boost::circular_buffer<geometry_msgs::Point> curve_point = getCrvPoints(lane, i);
    const std::vector<double> curve_param = calcCurveParam(curve_point);

    // if going straight
    if (curve_param.empty())
    {
      curve_radius->at(i) = r_inf_;
    }
    // else if turnning curve
    else
    {
      curve_radius->at(i) = (curve_param[2] > r_inf_) ? r_inf_ : curve_param[2];
    }
  }
}

const double WaypointReplannerCore::calcVelParam(const double vmax) const
{
  if (fabs(r_th_ - r_min_) < 1e-8)
  {
    return DBL_MAX;  // error
  }
  return (vmax - velocity_min_) / (r_th_ - r_min_);
}

void WaypointReplannerCore::createCurveList(
    const std::vector<double>& curve_radius,
    std::unordered_map<unsigned long, std::pair<unsigned long, double> >* curve_list)
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
      (*curve_list)[index] = std::make_pair(i, radius_localmin);
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

void WaypointReplannerCore::createVmaxList(const autoware_msgs::lane& lane, const std::unordered_map<unsigned long, std::pair<unsigned long, double> > &curve_list,
                    unsigned long offset, std::unordered_map<unsigned long, std::pair<unsigned long, double> > *vmax_list)
{
  unsigned long start_idx = 0;
  unsigned int list_idx = 0;
  const unsigned long last_idx = lane.waypoints.size() - 1;
  std::vector<unsigned long> id_list;
  if (!curve_list.empty())
  {
    id_list.resize(curve_list.size());
  }
  for (const auto& el : curve_list)
  {
    id_list[list_idx++] = el.first;
  }
  std::sort(id_list.begin(), id_list.end());


  for (const auto& el : id_list)
  {
    const std::pair<unsigned long, double> &pair = curve_list.at(el);
    if (start_idx != el)
    {
      double vmax = searchVmaxByRange(start_idx, el, offset, lane);
      vmax = (constant_vmax_mode_ || vmax > velocity_max_) ? velocity_max_ : vmax;
      vmax = (vmax < velocity_min_) ? velocity_min_ : vmax;
      (*vmax_list)[el] = std::make_pair(start_idx, vmax);
    }
    start_idx = pair.first + 1;
  }
  if (start_idx != 0 && start_idx < last_idx)
  {
    double vmax = searchVmaxByRange(start_idx, last_idx, offset, lane);
    if (constant_vmax_mode_ || vmax > velocity_max_)
    {
      vmax = velocity_max_;
    }
    if (vmax < velocity_min_)
    {
      vmax = velocity_min_;
    }
    (*vmax_list)[last_idx] = std::make_pair(start_idx, vmax);
  }
}

double WaypointReplannerCore::searchVmaxByRange(unsigned long start_idx, unsigned long end_idx, unsigned int offset,
                          const autoware_msgs::lane &lane) const
{
  if (offset > 0)
  {
    start_idx = (start_idx > offset) ? (start_idx - offset) : 0;
    end_idx = (end_idx > offset) ? (end_idx - offset) : 0;
  }
  double vmax = 0.0;
  for (unsigned long idx = start_idx; idx <= end_idx; idx++)
  {
    const double vel = fabs(lane.waypoints[idx].twist.twist.linear.x);
    vmax = (vmax < vel) ? vel : vmax;
  }
  return vmax;
}

void WaypointReplannerCore::setVelocityByRange(unsigned long start_idx, unsigned long end_idx, unsigned int offset,
                                             double vel, autoware_msgs::lane* lane)
{
  if (offset > 0)
  {
    start_idx = (start_idx > offset) ? (start_idx - offset) : 0;
    end_idx = (end_idx > offset) ? (end_idx - offset) : 0;
  }
  for (unsigned long idx = start_idx; idx <= end_idx; idx++)
  {
    lane->waypoints[idx].twist.twist.linear.x = vel;
  }
}

void WaypointReplannerCore::limitVelocityByRange(unsigned long start_idx, unsigned long end_idx, unsigned int offset,
                                             double vmin, autoware_msgs::lane* lane)
{
  if (offset > 0)
  {
    start_idx = (start_idx > offset) ? (start_idx - offset) : 0;
    end_idx = (end_idx > offset) ? (end_idx - offset) : 0;
  }
  for (unsigned long idx = start_idx; idx <= end_idx; idx++)
  {
    if (lane->waypoints[idx].twist.twist.linear.x < vmin)
    {
      continue;
    }
    lane->waypoints[idx].twist.twist.linear.x = vmin;
  }
  limitAccelDecel(start_idx, lane);
  limitAccelDecel(end_idx, lane);
}

void WaypointReplannerCore::limitAccelDecel(const unsigned long idx, autoware_msgs::lane* lane)
{
  const double acc[2] = { accel_limit_, decel_limit_ };
  const unsigned long end_idx[2] = { lane->waypoints.size() - idx, idx + 1 };
  const int sgn[2] = { 1, -1 };
  for (int j = 0; j < 2; j++)  // [j=0]: accel_limit_process, [j=1]: decel_limit_process
  {
    double v = lane->waypoints[idx].twist.twist.linear.x;
    unsigned long next = idx + sgn[j];
    for (unsigned long i = 1; i < end_idx[j]; i++, next += sgn[j])
    {
      const geometry_msgs::Point& p0 = lane->waypoints[next - sgn[j]].pose.pose.position;
      const geometry_msgs::Point& p1 = lane->waypoints[next].pose.pose.position;
      const double dist = std::hypot(p0.x - p1.x, p0.y - p1.y);
      v = sqrt(2 * acc[j] * dist + v * v);
      if (v > lane->waypoints[next].twist.twist.linear.x)
      {
        break;
      }
      lane->waypoints[next].twist.twist.linear.x = v;
    }
  }
}

// get curve 3-Parameter [center_x, center_y, radius] with 3 point input. If error occured, return empty vector.
const std::vector<double> WaypointReplannerCore::calcCurveParam(boost::circular_buffer<geometry_msgs::Point> p) const
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

const double WaypointReplannerCore::calcPathLength(const autoware_msgs::lane& lane) const
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
}
