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

#include "jsk_recognition_utils/geo/plane.h"
#include "jsk_recognition_utils/geo_util.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"

namespace jsk_recognition_utils
{
  Plane::Plane(const std::vector<float>& coefficients)
  {
    normal_ = Eigen::Vector3f(coefficients[0], coefficients[1], coefficients[2]);
    d_ = coefficients[3] / normal_.norm();
    normal_.normalize();
    initializeCoordinates();
  }

  Plane::Plane(const boost::array<float, 4>& coefficients)
  {
    normal_ = Eigen::Vector3f(coefficients[0], coefficients[1], coefficients[2]);
    d_ = coefficients[3] / normal_.norm();
    normal_.normalize();
    initializeCoordinates();
  }

  Plane::Plane(Eigen::Vector3f normal, double d) :
    normal_(normal.normalized()), d_(d / normal.norm())
  {
    initializeCoordinates();
  }
  
  Plane::Plane(Eigen::Vector3f normal, Eigen::Vector3f p) :
    normal_(normal.normalized()), d_(- normal.dot(p) / normal.norm())
  {
    initializeCoordinates();
  }
          
  
  Plane::~Plane()
  {

  }
  
  Eigen::Vector3f Plane::getPointOnPlane()
  {
    Eigen::Vector3f x = normal_ / (normal_.norm() * normal_.norm()) * (- d_);
    return x;
  }

  Plane Plane::flip()
  {
    return Plane(- normal_, - d_);
  }

  Plane::Ptr Plane::faceToOrigin()
  {
    Eigen::Vector3f p = getPointOnPlane();
    Eigen::Vector3f n = getNormal();
    
    if (p.dot(n) < 0) {
      return Plane::Ptr (new Plane(normal_, d_));
    }
    else {
      return Plane::Ptr (new Plane(- normal_, - d_));
    }
  }

  bool Plane::isSameDirection(const Plane& another)
  {
    return isSameDirection(another.normal_);
  }
  
  bool Plane::isSameDirection(const Eigen::Vector3f& another_normal)
  {
    return normal_.dot(another_normal) > 0;
  }
  
  double Plane::signedDistanceToPoint(const Eigen::Vector3f p)
  {
    return (normal_.dot(p) + d_);
  }
  
  double Plane::signedDistanceToPoint(const Eigen::Vector4f p)
  {
    return signedDistanceToPoint(Eigen::Vector3f(p[0], p[1], p[2]));
  }
  
  double Plane::distanceToPoint(const Eigen::Vector4f p)
  {
    return fabs(signedDistanceToPoint(p));
  }

  double Plane::distanceToPoint(const Eigen::Vector3f p)
  {
    return fabs(signedDistanceToPoint(p));
  }
  
  double Plane::distance(const Plane& another)
  {
    return fabs(fabs(d_) - fabs(another.d_));
  }

  double Plane::angle(const Eigen::Vector3f& vector)
  {
    double dot = normal_.dot(vector);
    if (dot > 1.0) {
      dot = 1.0;
    }
    else if (dot < -1.0) {
      dot = -1.0;
    }
    double theta = acos(dot);
    if (theta > M_PI / 2.0) {
      return M_PI - theta;
    }

    return acos(dot);
  }
  
  double Plane::angle(const Plane& another)
  {
    double dot = normal_.dot(another.normal_);
    if (dot > 1.0) {
      dot = 1.0;
    }
    else if (dot < -1.0) {
      dot = -1.0;
    }
    double theta = acos(dot);
    if (theta > M_PI / 2.0) {
      return M_PI - theta;
    }

    return acos(dot);
  }

  void Plane::project(const Eigen::Vector3f& p, Eigen::Vector3f& output)
  {
    // double alpha = - p.dot(normal_);
    // output = p + alpha * normal_;
    double alpha = p.dot(normal_) + d_;
    //double alpha = p.dot(normal_) - d_;
    output = p - alpha * normal_;
  }

  void Plane::project(const Eigen::Vector3d& p, Eigen::Vector3d& output)
  {
    Eigen::Vector3f output_f;
    project(Eigen::Vector3f(p[0], p[1], p[2]), output_f);
    pointFromVectorToVector<Eigen::Vector3f, Eigen::Vector3d>(output_f, output);
  }

  void Plane::project(const Eigen::Vector3d& p, Eigen::Vector3f& output)
  {
    project(Eigen::Vector3f(p[0], p[1], p[2]), output);
  }

  void Plane::project(const Eigen::Vector3f& p, Eigen::Vector3d& output)
  {
    Eigen::Vector3f output_f;
    project(p, output);
    pointFromVectorToVector<Eigen::Vector3f, Eigen::Vector3d>(output_f, output);
  }

  void Plane::project(const Eigen::Affine3d& pose, Eigen::Affine3d& output)
  {
    Eigen::Affine3f pose_f, output_f;
    convertEigenAffine3(pose, pose_f);
    project(pose_f, output_f);
    convertEigenAffine3(output_f, output);
  }

  void Plane::project(const Eigen::Affine3f& pose, Eigen::Affine3f& output)
  {
    Eigen::Vector3f p(pose.translation());
    Eigen::Vector3f output_p;
    project(p, output_p);
    Eigen::Quaternionf rot;
    rot.setFromTwoVectors(pose.rotation() * Eigen::Vector3f::UnitZ(),
                          coordinates().rotation() * Eigen::Vector3f::UnitZ());
    output = Eigen::Affine3f::Identity() * Eigen::Translation3f(output_p) * rot;
  }

  Plane Plane::transform(const Eigen::Affine3f& transform)
  {
    Eigen::Affine3d transform_d;
    convertEigenAffine3(transform, transform_d);
    return this->transform(transform_d);
  }
  
  Plane Plane::transform(const Eigen::Affine3d& transform)
  {
    Eigen::Vector4d n;
    n[0] = normal_[0];
    n[1] = normal_[1];
    n[2] = normal_[2];
    n[3] = d_;
    Eigen::Matrix4d m = transform.matrix();
    Eigen::Vector4d n_d = m.transpose() * n;
    //Eigen::Vector4d n_dd = n_d.normalized();
    Eigen::Vector4d n_dd = n_d / sqrt(n_d[0] * n_d[0] + n_d[1] * n_d[1] + n_d[2] * n_d[2]);
    return Plane(Eigen::Vector3f(n_dd[0], n_dd[1], n_dd[2]), n_dd[3]);
  }
  
  std::vector<float> Plane::toCoefficients()
  {
    std::vector<float> ret;
    toCoefficients(ret);
    return ret;
  }

  void Plane::toCoefficients(std::vector<float>& output)
  {
    output.push_back(normal_[0]);
    output.push_back(normal_[1]);
    output.push_back(normal_[2]);
    output.push_back(d_);
  }

  Eigen::Vector3f Plane::getNormal()
  {
    return normal_;
  }

  double Plane::getD() 
  {
    return d_;
  }

  void Plane::initializeCoordinates()
  {
    Eigen::Quaternionf rot;
    rot.setFromTwoVectors(Eigen::Vector3f::UnitZ(), normal_);
    double c = normal_[2];
    double z = 0.0;
    // ax + by + cz + d = 0
    // z = - d / c (when x = y = 0)
    if (c == 0.0) {             // its not good
      z = 0.0;
    }
    else {
      z = - d_ / c;
    }
    plane_coordinates_
      = Eigen::Affine3f::Identity() * Eigen::Translation3f(0, 0, z) * rot;
  }
  
  Eigen::Affine3f Plane::coordinates()
  {
    return plane_coordinates_;
  }

}
