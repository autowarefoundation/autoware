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

#ifndef JSK_RECOGNITION_UTILS_GEO_PLANE_H_
#define JSK_RECOGNITION_UTILS_GEO_PLANE_H_

#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>
#include <vector>
#include "jsk_recognition_utils/types.h"

namespace jsk_recognition_utils
{
  class Plane
  {
  public:
    typedef boost::shared_ptr<Plane> Ptr;
    Plane(const std::vector<float>& coefficients);
    Plane(const boost::array<float, 4>& coefficients);
    Plane(Eigen::Vector3f normal, double d);
    Plane(Eigen::Vector3f normal, Eigen::Vector3f p);
    virtual ~Plane();
    virtual Plane flip();
    virtual Plane::Ptr faceToOrigin();
    virtual bool isSameDirection(const Plane& another);
    virtual bool isSameDirection(const Eigen::Vector3f& another_normal);
    virtual double signedDistanceToPoint(const Eigen::Vector4f p);
    virtual double distanceToPoint(const Eigen::Vector4f p);
    virtual double signedDistanceToPoint(const Eigen::Vector3f p);
    virtual double distanceToPoint(const Eigen::Vector3f p);
    virtual double distance(const Plane& another);
    virtual double angle(const Plane& another);
    virtual double angle(const Eigen::Vector3f& vector);
    virtual void project(const Eigen::Vector3f& p, Eigen::Vector3f& output);
    virtual void project(const Eigen::Vector3d& p, Eigen::Vector3d& output);
    virtual void project(const Eigen::Vector3d& p, Eigen::Vector3f& output);
    virtual void project(const Eigen::Vector3f& p, Eigen::Vector3d& output);
    virtual void project(const Eigen::Affine3f& pose, Eigen::Affine3f& output);
    virtual Eigen::Vector3f getNormal();
    virtual Eigen::Vector3f getPointOnPlane();
    virtual Plane transform(const Eigen::Affine3d& transform);
    virtual Plane transform(const Eigen::Affine3f& transform);
    virtual void toCoefficients(std::vector<float>& output);
    virtual std::vector<float> toCoefficients();
    virtual double getD();
    virtual Eigen::Affine3f coordinates();
  protected:
    virtual void initializeCoordinates();
    Eigen::Vector3f normal_;
    double d_;
    Eigen::Affine3f plane_coordinates_;
  private:
  };

}

#endif


