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

#ifndef JSK_RECOGNITION_UTILS_GEO_POLYLINE_H_
#define JSK_RECOGNITION_UTILS_GEO_POLYLINE_H_

#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>
#include "jsk_recognition_utils/types.h"
#include "jsk_recognition_utils/geo/segment.h"
#include "visualization_msgs/Marker.h"

namespace jsk_recognition_utils
{
  /**
   * @brief
   * Class to represent 3-D polyline (not closed).
   */
  class PolyLine : public Line
  {
  public:
    typedef boost::shared_ptr<PolyLine> Ptr;
    /**
     * @brief
     * Construct a polyline from points.
     * The polyline consists of lines which starts with p[i] and ends with p[i+1].
     * @param points
     */
    PolyLine(const std::vector < Eigen::Vector3f > &points);

    /**
     * @ brief
     * get the line positioned at index
     *
     * @param index position of the line
     */
    virtual Segment::Ptr at(int index) const;

    /**
     * @brief
     * compute a distance to a point
     */
    virtual double distance(const Eigen::Vector3f& point,
                            Eigen::Vector3f& foot_point) const;
    virtual double distance(const Eigen::Vector3f& point) const;

    /**
     * @brief
     * compute a distance to a point, get various information
     */
    virtual double distanceWithInfo(const Eigen::Vector3f& from,
                                    Eigen::Vector3f& foot_point,
                                    double& distance_to_goal,
                                    int& foot_index,
                                    double& foot_alpha) const;

    /**
     * @brief
     * get normalized direction vector of the line.
     *
     * @param index position of the line which returns direction
     */
    virtual void getDirection(int index, Eigen::Vector3f& output) const;
    virtual Eigen::Vector3f getDirection(int index) const;

    /**
     * @brief
     * get total length of the polyline
     */
    virtual double length() const;

    /**
     * @ brief
     * flip direction of the polyline.
     */
    virtual PolyLine::Ptr flipPolyLine() const;

    /**
     * @ brief
     * make marker message to display the polyline
     */
    void toMarker(visualization_msgs::Marker& marker) const;

    friend std::ostream& operator<<(std::ostream& os, const PolyLine& pl);
  protected:
    std::vector< Segment::Ptr > segments;
  };
}

#endif
