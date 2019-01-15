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

#ifndef JSK_RECOGNITION_UTILS_GEO_LINE_H_
#define JSK_RECOGNITION_UTILS_GEO_LINE_H_

#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>
#include "jsk_recognition_utils/types.h"

namespace jsk_recognition_utils
{
  /**
   * @brief
   * Class to represent 3-D straight line.
   */
  class Line
  {
  public:
    typedef boost::shared_ptr<Line> Ptr;
    
    /**
     * @brief
     * Construct a line from direction vector and a point on the line.
     *
     * @param direction direction of the line. non-normalized vector is allowed.
     * @param origin point on the line.
     */
    Line(const Eigen::Vector3f& direction, const Eigen::Vector3f& origin);

    /**
     * @brief
     * get normalized direction vector of the line and assign it to output.
     */
    virtual void getDirection(Eigen::Vector3f& output) const;

    /**
     * @brief
     * get normalized direction vector of the line.
     */
    virtual Eigen::Vector3f getDirection() const;

    /**
     * @brief
     * get origin of the line and assing it to output.
     */
    virtual void getOrigin(Eigen::Vector3f& output) const;

    /**
     * @brief
     * get origin of the line.
     */
    virtual Eigen::Vector3f getOrigin() const;

    /**
     * @brief
     * compute a distance to a point
     */
    virtual double distanceToPoint(const Eigen::Vector3f& from) const;

    /**
     * @brief
     * compute a distance to a point and foot point will be assigned to foot.
     */
    virtual double distanceToPoint(const Eigen::Vector3f& from,
                                   Eigen::Vector3f& foot) const;

    /**
     * @brief
     * compute a distance to line.
     */
    virtual double distance(const Line& other) const;

    /**
     * @brief
     * compute a point which gives perpendicular projection.
     */
    virtual void foot(const Eigen::Vector3f& point, Eigen::Vector3f& output) const;

    /**
     * @brief
     * compute angle between a given line.
     */
    virtual double angle(const Line& other) const;

    /**
     * @brief
     * return true if given line is parallel.
     * angle_threshold is error tolerance.
     */
    virtual bool isParallel(const Line& other, double angle_threshold = 0.1) const;
    
    /**
     * @brief
     * return true if given line is perpendicular.
     * angle_threshold is error tolerance.
     */
    virtual bool isPerpendicular(const Line& other, double angle_threshold = 0.1) const;

    /**
     * @brief
     * compute a middle line between given line.
     */
    virtual Ptr midLine(const Line& other) const;

    /**
     * @brief
     * compute a line on a point, whose direction is same to the
     * current line.
     */
    virtual Ptr parallelLineOnAPoint(const Eigen::Vector3f& p) const;

    /**
     * @brief
     * Extract end points from the points on the lines.
     */
    virtual PointPair findEndPoints(const Vertices& points) const;

    /**
     * @ brief
     * Let equation of line's scale factor a.
     * 
     *  x = a d + p
     * where d is a normalized direction vector and p is a point on the vector.
     */
    virtual double computeAlpha(const Point& p) const;

    /**
     * @ brief
     * Return true if given line is towards the same direction.
     */
    virtual bool isSameDirection(const Line& other) const;

    /**
     * @ brief
     * flip direction of line.
     */
    virtual Line::Ptr flip();

    /**
     * @brief
     * compute a perpendicular line of two lines from origin_
     */
    virtual void parallelLineNormal(const Line& other, Eigen::Vector3f& output) const;

    /**
     * @brief
     * Instantiate Line from array of float.
     */
    static Ptr fromCoefficients(const std::vector<float>& coefficients);

    /**
     * @brief
     * Print Line information
     */
    virtual void print();
    
    /**
     * @brief
     * Compute a point on normal from alpha parameter.
     */
    virtual void point(double alpha, Eigen::Vector3f& ouptut);
  protected:
    Eigen::Vector3f direction_;
    Eigen::Vector3f origin_;
  private:
  };

}

#endif
