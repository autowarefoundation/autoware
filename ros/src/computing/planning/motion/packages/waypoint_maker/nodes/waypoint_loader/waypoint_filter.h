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
#ifndef __WAYPOINT_FILTER_H__
#define __WAYPOINT_FILTER_H__

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <autoware_msgs/ConfigWaypointLoader.h>
#include <fstream>
#include <unordered_map>
#include "autoware_msgs/lane.h"

namespace waypoint_maker
{

class WaypointFilter
{
private:
  ros::NodeHandle private_nh_;
  double r_th_, r_min_, r_inf_;
  int lkup_crv_width_;
  double velocity_max_, velocity_min_;
  double accel_limit_, decel_limit_, resample_interval_;
  int velocity_offset_;
public:
  WaypointFilter();
  ~WaypointFilter();
  void initParameter(const autoware_msgs::ConfigWaypointLoader::ConstPtr& conf);
  void filterLaneWaypoint(autoware_msgs::lane *lane);
protected:
  void resampleLaneWaypoint(const double resample_interval, autoware_msgs::lane *lane, std::vector<double> *curve_radius);
  const std::vector<double> calcVelParamFromVmax(const double vmax)const;
  void createCurveList(const std::vector<double>& curve_radius, std::unordered_map<unsigned long, std::pair<unsigned long, double> >* curve_list);
  void limitAccelDecel(const double vmax, const double vmin_local, const unsigned long idx, autoware_msgs::lane *lane);
  const std::vector<double> getCurveOnce(const std::vector<geometry_msgs::Point>& point)const;
  const double calcSquareSum(const double x, const double y)const;
  const double calcPathLength(const autoware_msgs::lane& lane)const;
};
}
#endif
