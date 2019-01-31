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
#ifndef __WAYPOINT_REPLANNER_H__
#define __WAYPOINT_REPLANNER_H__

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <autoware_config_msgs/ConfigWaypointLoader.h>
#include <fstream>
#include <unordered_map>
#include <algorithm>
#include <boost/circular_buffer.hpp>
#include "autoware_msgs/Lane.h"

namespace waypoint_maker
{
class WaypointReplanner
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
  WaypointReplanner();
  ~WaypointReplanner();
  void initParameter(const autoware_config_msgs::ConfigWaypointLoader::ConstPtr& conf);
  void replanLaneWaypointVel(autoware_msgs::Lane* lane);

protected:
  void resampleLaneWaypoint(const double resample_interval, autoware_msgs::Lane* lane);
  void resampleOnStraight(const boost::circular_buffer<geometry_msgs::Point>& curve_point, autoware_msgs::Lane* lane);
  void resampleOnCurve(const geometry_msgs::Point& target_point, const std::vector<double>& param,
                       autoware_msgs::Lane* lane);

  const boost::circular_buffer<geometry_msgs::Point> getCrvPointsOnResample(const autoware_msgs::Lane& lane,
                                                                            const autoware_msgs::Lane& original_lane,
                                                                            unsigned long original_index) const;
  const boost::circular_buffer<geometry_msgs::Point> getCrvPoints(const autoware_msgs::Lane& lane,
                                                                  unsigned long index) const;

  void createRadiusList(const autoware_msgs::Lane& lane, std::vector<double>* curve_radius);
  const double calcVelParam() const;
  void createCurveList(const std::vector<double>& curve_radius,
                       std::unordered_map<unsigned long, std::pair<unsigned long, double> >* curve_list);

  void limitVelocityByRange(unsigned long start_idx, unsigned long end_idx, unsigned int offset, double vmin,
                            autoware_msgs::Lane* lane);
  void limitAccelDecel(const unsigned long idx, autoware_msgs::Lane* lane);

  const std::vector<double> calcCurveParam(boost::circular_buffer<geometry_msgs::Point> point) const;
  const double calcPathLength(const autoware_msgs::Lane& lane) const;
};
}
#endif
