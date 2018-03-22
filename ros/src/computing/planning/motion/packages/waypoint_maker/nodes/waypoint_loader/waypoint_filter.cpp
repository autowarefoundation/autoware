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
#include "waypoint_filter.h"

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

  WaypointFilter::WaypointFilter():private_nh_("~")
  {
  }

  WaypointFilter::~WaypointFilter(){}

  void WaypointFilter::initParameter(const autoware_msgs::ConfigWaypointLoader::ConstPtr& conf)
  {
    velocity_max_ = kmph2mps(conf->velocity_max);
    velocity_min_ = kmph2mps(conf->velocity_min);
    accel_limit_ = conf->accel_limit;
    decel_limit_ = conf->decel_limit;
    r_th_ = conf->radius_thresh;
    r_min_ = conf->radius_min;
    lkup_crv_width_ = 5;
    resample_interval_ = conf->resample_interval;
    velocity_offset_ = conf->velocity_offset;
    r_inf_ = 10 * r_th_;
  }

  void WaypointFilter::filterLaneWaypoint(autoware_msgs::lane *lane)
  {
    std::vector<double> curve_radius;
    std::unordered_map<unsigned long, std::pair<unsigned long, double> > curve_list;

    resampleLaneWaypoint(resample_interval_, lane, &curve_radius);
    const std::vector<double> vel_param = calcVelParamFromVmax(velocity_max_);
    createCurveList(curve_radius, &curve_list);
    if(vel_param.size() < 2)
    {
      ROS_ERROR("velocity parameter is invalid");
      return;
    }
    //set velocity_max for all_point
    for(auto& el : lane->waypoints)
      el.twist.twist.linear.x = velocity_max_;
    //set curve_velocity on curve begining
    for(const auto& el : curve_list)
    {
      const unsigned long start_idx = (el.first > velocity_offset_) ? (el.first - velocity_offset_) : 0;
      const unsigned long end_idx = (el.second.first > velocity_offset_) ? (el.second.first - velocity_offset_) : 0;
      const double radius = el.second.second;
      const double vmax = velocity_max_;
      const double vmin = vel_param[0] * radius + vel_param[1];
      for(unsigned long idx = start_idx; idx <= end_idx ; idx++)
      {
        if(lane->waypoints[idx].twist.twist.linear.x < vmin)continue;
        lane->waypoints[idx].twist.twist.linear.x = vmin;
      }
      limitAccelDecel(vmax, vmin, start_idx, lane);
      limitAccelDecel(vmax, vmin, end_idx, lane);
    }
    unsigned long end_id[2] = {0, lane->waypoints.size() - 1};
    for(int i = 0; i < 2; i++)
    {
      const unsigned long idx = end_id[i];
      const double vmax = velocity_max_;
      const double vmin = 0.0;
      if(lane->waypoints[idx].twist.twist.linear.x < vmin)continue;
      lane->waypoints[idx].twist.twist.linear.x = vmin;
      limitAccelDecel(vmax, vmin, idx, lane);
    }
  }

  void WaypointFilter::resampleLaneWaypoint(const double resample_interval, autoware_msgs::lane *lane, std::vector<double> *curve_radius)
  {
    if(lane->waypoints.empty())return;
    autoware_msgs::lane original_lane = *lane;
    lane->waypoints.clear();
    lane->waypoints.push_back(original_lane.waypoints[0]);
    curve_radius->clear();
    curve_radius->push_back(r_inf_);
    double original_len = calcPathLength(original_lane);
    unsigned long waypoints_size = ceil(1.5 * original_len / resample_interval_);
    lane->waypoints.reserve(waypoints_size);
    curve_radius->reserve(waypoints_size);

    const unsigned int n = (lkup_crv_width_ - 1) / 2;
    for(unsigned long i = 1; i < original_lane.waypoints.size() - 1; i++)
    {
      std::vector<geometry_msgs::Point> curve_point(3);
      curve_point[0] = (lane->waypoints.size() < n) ? lane->waypoints[0].pose.pose.position
                                                    : lane->waypoints[lane->waypoints.size() - n].pose.pose.position;
      curve_point[1] = original_lane.waypoints[i].pose.pose.position;
      curve_point[2] = (i >= original_lane.waypoints.size() - n) ? original_lane.waypoints.back().pose.pose.position
                                                                : original_lane.waypoints[i + n].pose.pose.position;
      const std::vector<double> curve_param = getCurveOnce(curve_point);
      //if going straight
      if(curve_param.empty())
      {
        const std::vector<double> vec = {curve_point[2].x - curve_point[0].x, curve_point[2].y - curve_point[0].y};
        autoware_msgs::waypoint wp;
        wp.pose.pose.position = lane->waypoints.back().pose.pose.position;
        wp.pose.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(vec[1], vec[0]));
        const std::vector<double> nvec = {curve_point[1].x - wp.pose.pose.position.x, curve_point[1].y - wp.pose.pose.position.y};
        double dist = sqrt(calcSquareSum(nvec[0], nvec[1]));
        const tf::Vector3 resample_vec(resample_interval_ * vec[0] / dist, resample_interval_ * vec[1] / dist, 0.0);
        for(; dist > resample_interval_; dist -= resample_interval_)
        {
          if(lane->waypoints.size() == lane->waypoints.capacity())break;
          wp.pose.pose.position.x += resample_vec.x();
          wp.pose.pose.position.y += resample_vec.y();
          lane->waypoints.push_back(wp);
          curve_radius->push_back(r_inf_);
        }
      }
      //else if turnning curve
      else
      {
        const double& cx = curve_param[0];
        const double& cy = curve_param[1];
        const double& radius = curve_param[2];
        const double threshold_radius = (radius > r_inf_) ? r_inf_ : radius;

        const geometry_msgs::Point& p0 = lane->waypoints.back().pose.pose.position;
        const geometry_msgs::Point& p1 = curve_point[1];
        double theta = fmod(atan2(p1.y - cy, p1.x - cx) - atan2(p0.y - cy, p0.x - cx), 2 * M_PI);
        if(theta > M_PI)theta -= 2 * M_PI;
        else if(theta < -M_PI)theta += 2 * M_PI;
        //interport
        double t = atan2(p0.y - cy, p0.x - cx);
        for(double dist = radius * fabs(theta); dist > resample_interval_; dist -= resample_interval_)
        {
          if(lane->waypoints.size() == lane->waypoints.capacity())break;
          autoware_msgs::waypoint wp = lane->waypoints[0];
          const int sign = (theta > 0.0) ? (1) : (-1);
          t += sign * resample_interval_ / radius;
          const double yaw = fmod(t + sign * M_PI / 2.0, 2 * M_PI);
          wp.pose.pose.position.x = cx + radius * cos(t);
          wp.pose.pose.position.y = cy + radius * sin(t);
          wp.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
          lane->waypoints.push_back(wp);
          curve_radius->push_back(threshold_radius);
        }
      }
    }
  }

  const std::vector<double> WaypointFilter::calcVelParamFromVmax(const double vmax)const
  {
    std::vector<double> param(2, 0.0);
    param[0] = (vmax - velocity_min_) / (r_th_ - r_min_);//bias
    param[1] = vmax - param[0] * r_th_;//vel_intersept
    return param;
  }

  void WaypointFilter::createCurveList(const std::vector<double>& curve_radius, std::unordered_map<unsigned long, std::pair<unsigned long, double> >* curve_list)
  {
    unsigned long index = 0;
    bool on_curve = false;
    double radius_localmin = DBL_MAX;
    for(unsigned long i = 1; i < curve_radius.size(); i++)
    {
      if(!on_curve && curve_radius[i] <= r_th_ && curve_radius[i -1] > r_th_)
      {
        index = i;
        on_curve = true;
      }
      else if(on_curve && curve_radius[i - 1] <= r_th_ && curve_radius[i] > r_th_)
      {
        on_curve = false;
        if(radius_localmin < r_min_)radius_localmin = r_min_;
        (*curve_list)[index] = std::make_pair(i, radius_localmin);
        radius_localmin = DBL_MAX;
      }
      if(!on_curve)continue;
      if(radius_localmin > curve_radius[i])radius_localmin = curve_radius[i];
    }
  }

  void WaypointFilter::limitAccelDecel(const double vmax, const double vmin_local, const unsigned long idx, autoware_msgs::lane *lane)
  {
    double v = vmin_local;
    for(unsigned long i = 1; ; i++)
    {
      v = sqrt(2 * accel_limit_ * resample_interval_ + v * v);
      if(i > lane->waypoints.size() - idx - 1 || v > vmax)break;
      if(lane->waypoints[idx + i].twist.twist.linear.x < v)break;
      lane->waypoints[idx + i].twist.twist.linear.x = v;
    }

    v = vmin_local;
    for(unsigned long i = 1; ; i++)
    {
      v = sqrt(2 * decel_limit_ * resample_interval_ + v * v);
      if(i > idx || v > vmax)break;
      if(lane->waypoints[idx - i].twist.twist.linear.x < v)break;
      lane->waypoints[idx - i].twist.twist.linear.x = v;
    }
  }

  //get 3 parameter of curve, [center_x, center_y, radius]
  const std::vector<double> WaypointFilter::getCurveOnce(const std::vector<geometry_msgs::Point>& point)const
  {
    std::vector<double> curve_param(3, 0.0);
    tf::Vector3 vec[2];
    geometry_msgs::Point pt_m[2];
    double tan_pt_m[2];
    for(unsigned int i = 0; i < 2; i++)
    {
      const geometry_msgs::Point& p0 = point[i];
      const geometry_msgs::Point& p1 = point[i + 1];
      pt_m[i].x = (p0.x + p1.x) / 2.0;
      pt_m[i].y = (p0.y + p1.y) / 2.0;
      vec[i] = tf::Vector3(p1.x - p0.x, p1.y - p0.y, 0.0);
      tan_pt_m[i] = tan(atan2(vec[i].y(), vec[i].x()) + M_PI / 2.0);
    }
    if(fabs(tan_pt_m[1] - tan_pt_m[0]) < 1e-9)return std::vector<double>();

    {
      const geometry_msgs::Point& p0 = point[0];
      curve_param[0] = pt_m[0].y - pt_m[0].x * tan_pt_m[0] - pt_m[1].y + pt_m[1].x * tan_pt_m[1];
      curve_param[0] /= tan_pt_m[1] - tan_pt_m[0];
      curve_param[1] = pt_m[0].y - (pt_m[0].x - curve_param[0]) * tan_pt_m[0];
      curve_param[2] = sqrt(calcSquareSum(p0.x - curve_param[0], p0.y - curve_param[1]));
    }
    return curve_param;
  }

  const double WaypointFilter::calcSquareSum(const double x, const double y)const
  {
    return (x * x + y * y);
  }

  const double WaypointFilter::calcPathLength(const autoware_msgs::lane& lane)const
  {
    double distance = 0.0;
    for(unsigned long i = 1; i < lane.waypoints.size(); i++)
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
