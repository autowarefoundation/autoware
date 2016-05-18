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
    Line(from - to, from), from_(from), to_(to)
  {
    
  }

  double Segment::dividingRatio(const Eigen::Vector3f& point) const
  {
    if (to_[0] != from_[0]) {
      return (point[0] - from_[0]) / (to_[0] - from_[0]);
    }
    else if (to_[1] != from_[1]) {
      return (point[1] - from_[1]) / (to_[1] - from_[1]);
    }
    else {
      return (point[2] - from_[2]) / (to_[2] - from_[2]);
    }
  }
  
  void Segment::foot(const Eigen::Vector3f& from, Eigen::Vector3f& output) const
  {
    Eigen::Vector3f foot_point;
    Line::foot(from, foot_point);
    double r = dividingRatio(foot_point);
    if (r < 0.0) {
      output = from_;
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

  std::ostream& operator<<(std::ostream& os, const Segment& seg)
  {
    os << "[" << seg.from_[0] << ", " << seg.from_[1] << ", " << seg.from_[2] << "] -- "
       << "[" << seg.to_[0] << ", " << seg.to_[1] << ", " << seg.to_[2] << "]";
  }

}
