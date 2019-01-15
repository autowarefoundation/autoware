// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
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
#include "jsk_recognition_utils/geo/line.h"
#include <jsk_topic_tools/log_utils.h>
#include <cfloat>

namespace jsk_recognition_utils
{
  Line::Line(const Eigen::Vector3f& direction, const Eigen::Vector3f& origin)
    : direction_ (direction.normalized()), origin_(origin)
  {

  }

  void Line::getDirection(Eigen::Vector3f& output) const
  {
    output = direction_;
  }

  Eigen::Vector3f Line::getDirection() const
  {
    return direction_;
  }
  
  void Line::getOrigin(Eigen::Vector3f& output) const
  {
    output = origin_;
  }

  Eigen::Vector3f Line::getOrigin() const
  {
    return origin_;
  }
  
  void Line::foot(const Eigen::Vector3f& point, Eigen::Vector3f& output) const
  {
    const double alpha = computeAlpha(point);
    output = alpha * direction_ + origin_;
  }

  double Line::distanceToPoint(
    const Eigen::Vector3f& from, Eigen::Vector3f& foot_point) const
  {
    foot(from, foot_point);
    return (from - foot_point).norm();
  }
  
  double Line::distanceToPoint(const Eigen::Vector3f& from) const
  {
    Eigen::Vector3f foot_point;
    return distanceToPoint(from, foot_point);
  }

  double Line::angle(const Line& other) const
  {
    double dot = fabs(direction_.dot(other.direction_));
    if (dot > 1.0) {
      return M_PI / 2.0;
    }
    else {
      double theta = acos(dot);
      if (theta > M_PI / 2.0) {
        return M_PI / 2.0 - theta;
      }
      else {
        return theta;
      }
    }
  }

  bool Line::isParallel(const Line& other, double angle_threshold) const
  {
    return angle(other) < angle_threshold;
  }

  bool Line::isPerpendicular(const Line& other, double angle_threshold) const
  {
    return (M_PI / 2.0 - angle(other)) < angle_threshold;
  }

  bool Line::isSameDirection(const Line& other) const
  {
    return direction_.dot(other.direction_) > 0;
  }

  Line::Ptr Line::flip()
  {
    Line::Ptr ret (new Line(-direction_, origin_));
    return ret;
  }
  
  Line::Ptr Line::midLine(const Line& other) const
  {
    Eigen::Vector3f new_directin = (direction_ + other.direction_).normalized();
    Eigen::Vector3f new_origin;
    other.foot(origin_, new_origin);
    Line::Ptr ret (new Line(new_directin, (new_origin + origin_) / 2.0));
    return ret;
  }

  void Line::parallelLineNormal(const Line& other, Eigen::Vector3f& output)
    const
  {
    Eigen::Vector3f foot_point;
    other.foot(origin_, foot_point);
    output = origin_ - foot_point;
  }
  
  Line::Ptr Line::fromCoefficients(const std::vector<float>& coefficients)
  {
    Eigen::Vector3f p(coefficients[0],
                      coefficients[1],
                      coefficients[2]);
    Eigen::Vector3f d(coefficients[3],
                      coefficients[4],
                      coefficients[5]);
    Line::Ptr ret(new Line(d, p));
    return ret;
  }

  double Line::distance(const Line& other) const
  {
    Eigen::Vector3f v12 = (other.origin_ - origin_);
    Eigen::Vector3f n = direction_.cross(other.direction_);
    return fabs(n.dot(v12)) / n.norm();
  }

  Line::Ptr Line::parallelLineOnAPoint(const Eigen::Vector3f& p) const
  {
    Line::Ptr ret (new Line(direction_, p));
    return ret;
  }
  
  double Line::computeAlpha(const Point& p) const
  {
    return p.dot(direction_) - origin_.dot(direction_);
  }
  
  PointPair Line::findEndPoints(const Vertices& points) const
  {
    double min_alpha = DBL_MAX;
    double max_alpha = - DBL_MAX;
    Point min_alpha_point, max_alpha_point;
    for (size_t i = 0; i < points.size(); i++) {
      Point p = points[i];
      double alpha = computeAlpha(p);
      if (alpha > max_alpha) {
        max_alpha_point = p;
        max_alpha = alpha;
      }
      if (alpha < min_alpha) {
        min_alpha_point = p;
        min_alpha = alpha;
      }
    }
    // ROS_INFO("min: %f", min_alpha);
    // ROS_INFO("max: %f", max_alpha);
    return boost::make_tuple<Point, Point>(min_alpha_point, max_alpha_point);
  }

  void Line::print()
  {
    ROS_INFO("d: [%f, %f, %f], p: [%f, %f, %f]", direction_[0], direction_[1], direction_[2],
             origin_[0], origin_[1], origin_[2]);
  }

  void Line::point(double alpha, Eigen::Vector3f& output)
  {
    output = alpha * direction_ + origin_;
  }
}
