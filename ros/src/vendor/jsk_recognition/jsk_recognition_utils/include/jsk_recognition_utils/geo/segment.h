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

#ifndef JSK_RECOGNITION_UTILS_GEO_SEGMENT_H_
#define JSK_RECOGNITION_UTILS_GEO_SEGMENT_H_

#include <iostream>
#include "jsk_recognition_utils/geo/line.h"
#include "visualization_msgs/Marker.h"

namespace jsk_recognition_utils
{
  class Plane;
  /**
   * @brief
   * Class to represent 3-D straight line which has finite length.
   */
  class Segment: public Line
  {
  public:
    typedef boost::shared_ptr<Segment> Ptr;

    /**
     * @brief
     * Construct a line from a start point and a goal point.
     *
     * @param from
     * @param to
     */
    Segment(const Eigen::Vector3f& from, const Eigen::Vector3f to);

    /**
     * @brief
     * get end of the line and assing it to output.
     */
    virtual void getEnd(Eigen::Vector3f& output) const;
    virtual Eigen::Vector3f getEnd() const;

    virtual void foot(const Eigen::Vector3f& point, Eigen::Vector3f& output) const;
    virtual double dividingRatio(const Eigen::Vector3f& point) const;
    virtual double distance(const Eigen::Vector3f& point) const;
    virtual double distance(const Eigen::Vector3f& point, Eigen::Vector3f& foot_point) const;
    virtual bool intersect(Plane& plane, Eigen::Vector3f& point) const;
    virtual void midpoint(Eigen::Vector3f& midpoint) const;
    //virtual double distance(const Segment& other);
    friend std::ostream& operator<<(std::ostream& os, const Segment& seg);

    /**
     * @brief
     * compute a distance to a point
     * @param from
     * @param foot_point
     * @param distance_to_goal
     */
    virtual double distanceWithInfo(const Eigen::Vector3f& from,
                                    Eigen::Vector3f& foot_point,
                                    double &distance_to_goal) const;

    /**
     * @brief
     * return flipped line (line of opposite direction)
     */
    virtual Segment::Ptr flipSegment() const;

    /**
     * @brief
     * return length of the line
     */
    virtual double length() const;

    /**
     * @brief
     * make marker message to display the finite line
     */
    void toMarker(visualization_msgs::Marker& marker) const;

    /**
     * @brief
     * is crossing with another line
     */
    virtual bool isCross (const Line &ln, double distance_threshold = 1e-5) const;
    virtual bool isCross (const Segment &ln, double distance_threshold = 1e-5) const;

  protected:
    Eigen::Vector3f to_;
    double length_;
  private:
  };
}

#endif
