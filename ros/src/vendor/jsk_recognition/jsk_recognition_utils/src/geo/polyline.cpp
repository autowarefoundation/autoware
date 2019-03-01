// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#define BOOST_PARAMETER_MAX_ARITY 7
#include "jsk_recognition_utils/geo/polyline.h"
#include <jsk_topic_tools/log_utils.h>
#include <cfloat>

namespace jsk_recognition_utils
{
  PolyLine::PolyLine(const std::vector < Eigen::Vector3f > &points) : Line(points[points.size()-1] - points[0], points[0])
  {
    int n = points.size();
    segments.resize(n-1);
    for (int i = 0; i < n-1; i++) {
      Segment::Ptr ln(new Segment(points[i], points[i+1]));
      segments[i] = ln;
    }
  }

  Segment::Ptr PolyLine::at(int index) const
  {
    return segments.at(index);
  }

  double PolyLine::distanceWithInfo(const Eigen::Vector3f& from,
                                    Eigen::Vector3f& foot_point,
                                    double& distance_to_goal,
                                    int& foot_index,
                                    double& foot_alpha) const
  {
    double min_len = DBL_MAX;
    Eigen::Vector3f point;
    double from_start_to_foot = 0;
    double distance_from_start = 0;

    for(int i = 0; i < segments.size(); i++) {
      double to_goal;
      double dist = segments[i]->distanceWithInfo(from, point, to_goal);
      if (dist < min_len) {
        min_len = dist;
        foot_point = point;
        foot_index = i;
        foot_alpha = (segments[i]->length() -  to_goal);
        from_start_to_foot = distance_from_start + foot_alpha;
      }
      distance_from_start += segments[i]->length();
    }
    distance_to_goal = distance_from_start - from_start_to_foot;
    return min_len;
  }

  double PolyLine::distance(const Eigen::Vector3f& from) const
  {
    double gl, alp;
    int idx;
    Eigen::Vector3f p;
    distanceWithInfo(from, p, gl, idx, alp);
  }

  double PolyLine::distance(const Eigen::Vector3f& from,
                            Eigen::Vector3f& foot_point) const
  {
    double gl, alp;
    int idx;
    distanceWithInfo(from, foot_point, gl, idx, alp);
  }

  void PolyLine::getDirection(int index, Eigen::Vector3f& output) const
  {
    segments[index]->getDirection(output);
  }
  Eigen::Vector3f PolyLine::getDirection(int index) const
  {
    Eigen::Vector3f dir;
    getDirection(index, dir);
    return dir;
  }

  double PolyLine::length() const
  {
    double distance_from_start = 0;
    for(int i = 0; i < segments.size(); i++) {
      distance_from_start += segments[i]->length();
    }
    return distance_from_start;
  }

  PolyLine::Ptr PolyLine::flipPolyLine() const
  {
    PolyLine::Ptr ret;
    return ret;
  }

  void PolyLine::toMarker(visualization_msgs::Marker& marker) const
  {
    marker.type = visualization_msgs::Marker::LINE_STRIP;

    marker.scale.x = 0.02; // line width
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;

    marker.points.resize(0);
    for(int i = 0; i < segments.size(); i++) {
      Eigen::Vector3f p;
      segments[i]->getOrigin(p);
      geometry_msgs::Point pt;
      pt.x = p[0];
      pt.y = p[1];
      pt.z = p[2];
      marker.points.push_back(pt);
    }
    {
      Eigen::Vector3f p;
      segments[segments.size() - 1]->getEnd(p);
      geometry_msgs::Point pt;
      pt.x = p[0];
      pt.y = p[1];
      pt.z = p[2];
      marker.points.push_back(pt);
    }
  }

  std::ostream& operator<<(std::ostream& os, const PolyLine& pl)
  {
    os << "[" << pl.origin_[0];
    os << ", " << pl.origin_[1];
    os << ", " << pl.origin_[2] << "]";

    for (int i = 0; i < pl.segments.size(); i++) {
      Eigen::Vector3f p;
      pl.segments[i]->getEnd(p);
      os << " -- [" << p[0];
      os << ", " << p[1];
      os << ", " << p[2] << "]";
    }
    return os;
  }
}
