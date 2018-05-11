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
#ifndef __VELOCITY_REPLANNER_H__
#define __VELOCITY_REPLANNER_H__

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <autoware_msgs/ConfigWaypointLoader.h>
#include <fstream>
#include <unordered_map>
#include <algorithm>
#include <boost/circular_buffer.hpp>
#include "autoware_msgs/lane.h"

namespace waypoint_maker
{
class VelocityReplanner
{
private:
  ros::NodeHandle private_nh_;
  double r_th_, r_min_, r_inf_;
  int lookup_crv_width_;
  double velocity_max_, velocity_min_;
  double accel_limit_, decel_limit_, resample_interval_;
  int velocity_offset_;
  bool resample_mode_;
  int end_point_offset_;
  double vel_param_;

public:
  VelocityReplanner();
  ~VelocityReplanner();
  void initParameter(const autoware_msgs::ConfigWaypointLoader::ConstPtr& conf);
  void replanLaneWaypointVel(autoware_msgs::lane* lane);

protected:
  void resampleLaneWaypoint(const double resample_interval, autoware_msgs::lane* lane);
  void resampleOnStraight(const boost::circular_buffer<geometry_msgs::Point>& curve_point, autoware_msgs::lane* lane);
  void resampleOnCurve(const geometry_msgs::Point& target_point, const std::vector<double>& param,
                       autoware_msgs::lane* lane);

  const boost::circular_buffer<geometry_msgs::Point> getCrvPointsOnResample(const autoware_msgs::lane& lane,
                                                                            const autoware_msgs::lane& original_lane,
                                                                            unsigned long original_index) const;
  const boost::circular_buffer<geometry_msgs::Point> getCrvPoints(const autoware_msgs::lane& lane,
                                                                  unsigned long index) const;

  void createRadiusList(const autoware_msgs::lane& lane, std::vector<double>* curve_radius);
  const double calcVelParam() const;
  void createCurveList(const std::vector<double>& curve_radius,
                       std::unordered_map<unsigned long, std::pair<unsigned long, double> >* curve_list);

  void limitVelocityByRange(unsigned long start_idx, unsigned long end_idx, unsigned int offset, double vmin,
                            autoware_msgs::lane* lane);
  void limitAccelDecel(const unsigned long idx, autoware_msgs::lane* lane);

  const std::vector<double> calcCurveParam(boost::circular_buffer<geometry_msgs::Point> point) const;
  const double calcPathLength(const autoware_msgs::lane& lane) const;
};
}
#endif
