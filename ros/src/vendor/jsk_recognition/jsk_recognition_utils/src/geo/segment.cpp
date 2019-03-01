// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
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

#include "jsk_recognition_utils/geo/segment.h"
#include "jsk_recognition_utils/geo_util.h"
namespace jsk_recognition_utils
{
  Segment::Segment(const Eigen::Vector3f& from, const Eigen::Vector3f to):
    Line(to - from, from), to_(to), length_((to - from).norm())
  {
  }

  double Segment::dividingRatio(const Eigen::Vector3f& point) const
  {
    if (to_[0] != origin_[0]) {
      return (point[0] - origin_[0]) / (to_[0] - origin_[0]);
    }
    else if (to_[1] != origin_[1]) {
      return (point[1] - origin_[1]) / (to_[1] - origin_[1]);
    }
    else {
      return (point[2] - origin_[2]) / (to_[2] - origin_[2]);
    }
  }

  void Segment::foot(const Eigen::Vector3f& from, Eigen::Vector3f& output) const
  {
    Eigen::Vector3f foot_point;
    Line::foot(from, foot_point);
    double r = dividingRatio(foot_point);
    if (r < 0.0) {
      output = origin_;
    }
    else if (r > 1.0) {
      output = to_;
    }
    else {
      output = foot_point;
    }
  }

  double Segment::distance(const Eigen::Vector3f& point) const
  {
    Eigen::Vector3f foot_point;
    return distance(point, foot_point);
  }

  double Segment::distance(const Eigen::Vector3f& point,
                           Eigen::Vector3f& foot_point) const
  {
    foot(point, foot_point);
    return (foot_point - point).norm();
  }

  bool Segment::intersect(Plane& plane, Eigen::Vector3f& point) const
  {
    double x = - (plane.getNormal().dot(origin_) + plane.getD()) / (plane.getNormal().dot(direction_));
    point = direction_ * x + origin_;
    double r = dividingRatio(point);
    return 0 <= r && r <= 1.0;
  }

  void Segment::midpoint(Eigen::Vector3f& midpoint) const
  {
    midpoint = (origin_ + to_) * 0.5;
  }

  std::ostream& operator<<(std::ostream& os, const Segment& seg)
  {
    os << "[" << seg.origin_[0] << ", " << seg.origin_[1] << ", " << seg.origin_[2] << "] -- "
       << "[" << seg.to_[0] << ", " << seg.to_[1] << ", " << seg.to_[2] << "]";
  }

  void Segment::getEnd(Eigen::Vector3f& output) const
  {
    output = to_;
  }

  Eigen::Vector3f Segment::getEnd() const
  {
    return to_;
  }

  double Segment::distanceWithInfo(const Eigen::Vector3f& from,
                                   Eigen::Vector3f& foot_point,
                                   double &distance_to_goal) const
  {
    const double alpha = computeAlpha(from);

    if (alpha >= 0 && alpha <= length_) {
      // foot on the line
      foot_point = alpha * direction_ + origin_;
      distance_to_goal = length_ - alpha;
    } else if (alpha < 0) {
      // foot out of the line
      foot_point = origin_;
      distance_to_goal = length_;
    } else {
      foot_point = to_;
      distance_to_goal = 0;
    }
    return (from - foot_point).norm();
  }

  Segment::Ptr Segment::flipSegment() const
  {
    Segment::Ptr ret (new Segment(to_, origin_));
    return ret;
  }

  double Segment::length() const
  {
    return length_;
  }

  void Segment::toMarker(visualization_msgs::Marker& marker) const
  {
    marker.type = visualization_msgs::Marker::ARROW;//

    geometry_msgs::Point st;
    geometry_msgs::Point ed;
    st.x = origin_[0];
    st.y = origin_[1];
    st.z = origin_[2];
    ed.x = to_[0];
    ed.y = to_[1];
    ed.z = to_[2];

    marker.points.push_back(st);
    marker.points.push_back(ed);

    marker.scale.x = 0.012;
    marker.scale.y = 0.02;
    marker.color.a = 1;
    marker.color.r = 1;
    marker.color.g = 1;
    marker.color.b = 0;
  }

  bool Segment::isCross (const Line &ln, double distance_threshold) const
  {
    Eigen::Vector3f ln_origin = ln.getOrigin();
    Eigen::Vector3f ln_direction = ln.getDirection();
    Eigen::Vector3f v12 = (ln_origin - origin_);
    double n1n2 = ln_direction.dot(direction_);
    if (fabs(n1n2) < 1e-20) { // parallel
      return false;
    }
    double alp1 = (ln_direction.dot(v12) - (n1n2 * direction_.dot(v12))) / (1 - n1n2 * n1n2);
    double alp2 = ((n1n2 * ln_direction.dot(v12)) - direction_.dot(v12)) / (1 - n1n2 * n1n2);

    if (// alp1 >= 0 && alp1 <= ln.length() &&
        alp2 >= 0 && alp2 <= length_) {
      Eigen::Vector3f p1 = alp1 * ln_direction + ln_origin;
      Eigen::Vector3f p2 = alp2 * direction_ + origin_;
      if ((p1 - p2).norm() < distance_threshold) {
        return true;
      } else {
        return false;
      }
    }

    return false;
  }

  bool Segment::isCross (const Segment &ln, double distance_threshold) const
  {
    Eigen::Vector3f ln_origin = ln.getOrigin();
    Eigen::Vector3f ln_direction = ln.getDirection();
    Eigen::Vector3f v12 = (ln_origin - origin_);
    double n1n2 = ln_direction.dot(direction_);
    if (fabs(n1n2) < 1e-20) { // parallel
      return false;
    }
    double alp1 = (ln_direction.dot(v12) - (n1n2 * direction_.dot(v12))) / (1 - n1n2 * n1n2);
    double alp2 = ((n1n2 * ln_direction.dot(v12)) - direction_.dot(v12)) / (1 - n1n2 * n1n2);

    if (alp1 >= 0 && alp1 <= ln.length() &&
        alp2 >= 0 && alp2 <= length_) {
      Eigen::Vector3f p1 = alp1 * ln_direction + ln_origin;
      Eigen::Vector3f p2 = alp2 * direction_ + origin_;
      if ((p1 - p2).norm() < distance_threshold) {
        return true;
      } else {
        return false;
      }
    }

    return false;
  }
}
