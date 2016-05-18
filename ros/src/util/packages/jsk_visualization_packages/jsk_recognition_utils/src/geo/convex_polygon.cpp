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

#include "jsk_recognition_utils/geo/convex_polygon.h"
#include "jsk_recognition_utils/geo_util.h"

namespace jsk_recognition_utils
{
  ConvexPolygon::ConvexPolygon(const Vertices& vertices):
    Polygon(vertices)
  {

  }

  ConvexPolygon::ConvexPolygon(const Vertices& vertices,
                               const std::vector<float>& coefficients):
    Polygon(vertices, coefficients)
  {

  }
  
  void ConvexPolygon::projectOnPlane(
    const Eigen::Vector3f& p, Eigen::Vector3f& output)
  {
    Plane::project(p, output);
  }

  void ConvexPolygon::projectOnPlane(
    const Eigen::Affine3f& pose, Eigen::Affine3f& output)
  {
    Eigen::Vector3f p(pose.translation());
    Eigen::Vector3f output_p;
    projectOnPlane(p, output_p);
    // JSK_ROS_INFO("[ConvexPolygon::projectOnPlane] p: [%f, %f, %f]",
    //          p[0], p[1], p[2]);
    // JSK_ROS_INFO("[ConvexPolygon::projectOnPlane] output_p: [%f, %f, %f]",
    //          output_p[0], output_p[1], output_p[2]);
    Eigen::Quaternionf rot;
    rot.setFromTwoVectors(pose.rotation() * Eigen::Vector3f::UnitZ(),
                          coordinates().rotation() * Eigen::Vector3f::UnitZ());
    Eigen::Quaternionf coords_rot(coordinates().rotation());
    Eigen::Quaternionf pose_rot(pose.rotation());
    // JSK_ROS_INFO("[ConvexPolygon::projectOnPlane] rot: [%f, %f, %f, %f]",
    //          rot.x(), rot.y(), rot.z(), rot.w());
    // JSK_ROS_INFO("[ConvexPolygon::projectOnPlane] coords_rot: [%f, %f, %f, %f]",
    //          coords_rot.x(), coords_rot.y(), coords_rot.z(), coords_rot.w());
    // JSK_ROS_INFO("[ConvexPolygon::projectOnPlane] pose_rot: [%f, %f, %f, %f]",
    //          pose_rot.x(), pose_rot.y(), pose_rot.z(), pose_rot.w());
    // JSK_ROS_INFO("[ConvexPolygon::projectOnPlane] normal: [%f, %f, %f]", normal_[0], normal_[1], normal_[2]);
    // Eigen::Affine3f::Identity() *
    // output.translation() = Eigen::Translation3f(output_p);
    // output.rotation() = rot * pose.rotation();
    //output = Eigen::Translation3f(output_p) * rot * pose.rotation();
    output = Eigen::Affine3f(rot * pose.rotation());
    output.pretranslate(output_p);
    // Eigen::Vector3f projected_point = output * Eigen::Vector3f(0, 0, 0);
    // JSK_ROS_INFO("[ConvexPolygon::projectOnPlane] output: [%f, %f, %f]",
    //          projected_point[0], projected_point[1], projected_point[2]);
  }

  ConvexPolygon ConvexPolygon::flipConvex()
  {
    Vertices new_vertices;
    std::reverse_copy(vertices_.begin(), vertices_.end(),
                      std::back_inserter(new_vertices));
    std::vector<float> reversed_coefficients(4);
    reversed_coefficients[0] = - normal_[0];
    reversed_coefficients[1] = - normal_[1];
    reversed_coefficients[2] = - normal_[2];
    reversed_coefficients[3] = - d_;
    
    return ConvexPolygon(new_vertices, reversed_coefficients);
  }
  
  void ConvexPolygon::project(const Eigen::Vector3f& p, Eigen::Vector3f& output)
  {
    Eigen::Vector3f point_on_plane;
    Plane::project(p, point_on_plane);
    // check point_ is inside or not
    if (isInside(point_on_plane)) {
      output = point_on_plane;
    }
    else {
      // find the minimum foot point
      double min_distance = DBL_MAX;
      Eigen::Vector3f min_point;
      for (size_t i = 0; i < vertices_.size() - 1; i++) {
        Segment seg(vertices_[i], vertices_[i + 1]);
        Eigen::Vector3f foot;
        double distance = seg.distanceToPoint(p, foot);
        if (distance < min_distance) {
          min_distance = distance;
          min_point = foot;
        }
      }
      output = min_point;
    }
  }

  void ConvexPolygon::project(const Eigen::Vector3d& p, Eigen::Vector3d& output)
  {
    Eigen::Vector3f output_f;
    Eigen::Vector3f p_f(p[0], p[1], p[2]);
    project(p_f, output_f);
    pointFromVectorToVector<Eigen::Vector3f, Eigen::Vector3d>(output_f, output);
  }
  
  void ConvexPolygon::project(const Eigen::Vector3d& p, Eigen::Vector3f& output)
  {
    Eigen::Vector3f p_f(p[0], p[1], p[2]);
    project(p_f, output);
  }
  
  void ConvexPolygon::project(const Eigen::Vector3f& p, Eigen::Vector3d& output)
  {
    Eigen::Vector3f output_f;
    project(p, output_f);
    pointFromVectorToVector<Eigen::Vector3f, Eigen::Vector3d>(output_f, output);
  }
  

  Eigen::Vector3f ConvexPolygon::getCentroid()
  {
    Eigen::Vector3f ret(0, 0, 0);
    for (size_t i = 0; i < vertices_.size(); i++) {
      ret = ret + vertices_[i];
    }
    return ret / vertices_.size();
  }

  ConvexPolygon ConvexPolygon::fromROSMsg(const geometry_msgs::Polygon& polygon)
  {
    Vertices vertices;
    for (size_t i = 0; i < polygon.points.size(); i++) {
      Eigen::Vector3f p;
      pointFromXYZToVector<geometry_msgs::Point32, Eigen::Vector3f>(
        polygon.points[i], p);
      vertices.push_back(p);
    }
    return ConvexPolygon(vertices);
  }
  
  ConvexPolygon::Ptr ConvexPolygon::fromROSMsgPtr(const geometry_msgs::Polygon& polygon)
  {
    Vertices vertices;
    for (size_t i = 0; i < polygon.points.size(); i++) {
      Eigen::Vector3f p;
      pointFromXYZToVector<geometry_msgs::Point32, Eigen::Vector3f>(
        polygon.points[i], p);
      vertices.push_back(p);
    }
    return ConvexPolygon::Ptr(new ConvexPolygon(vertices));
  }

  bool ConvexPolygon::distanceSmallerThan(const Eigen::Vector3f& p,
                                          double distance_threshold)
  {
    double dummy_distance;
    return distanceSmallerThan(p, distance_threshold, dummy_distance);
  }
  
  bool ConvexPolygon::distanceSmallerThan(const Eigen::Vector3f& p,
                                          double distance_threshold,
                                          double& output_distance)
  {
    // first check distance as Plane rather than Convex
    double plane_distance = distanceToPoint(p);
    if (plane_distance > distance_threshold) {
      output_distance = plane_distance;
      return false;
    }

    Eigen::Vector3f foot_point;
    project(p, foot_point);
    double convex_distance = (p - foot_point).norm();
    output_distance = convex_distance;
    return convex_distance < distance_threshold;
  }

  bool ConvexPolygon::allEdgesLongerThan(double thr)
  {
    for (size_t i = 0; i < vertices_.size(); i++) {
      Eigen::Vector3f p_k = vertices_[i];
      Eigen::Vector3f p_k_1;
      if (i == vertices_.size() - 1) {
        p_k_1 = vertices_[0];
      }
      else {
        p_k_1 = vertices_[i + 1];
      }
      if ((p_k - p_k_1).norm() < thr) {
        return false;
      }
    }
    return true;
  }

  double ConvexPolygon::distanceFromVertices(const Eigen::Vector3f& p)
  {
    double min_distance = DBL_MAX;
    for (size_t i = 0; i < vertices_.size(); i++) {
      Eigen::Vector3f v = vertices_[i];
      double d = (p - v).norm();
      if (d < min_distance) {
        min_distance = d;
      }
    }
    return min_distance;
  }
  
  ConvexPolygon::Ptr ConvexPolygon::magnifyByDistance(const double distance)
  {
    // compute centroid
    Eigen::Vector3f c = centroid();
    Vertices new_vertices(vertices_.size());
    for (size_t i = 0; i < vertices_.size(); i++) {
      new_vertices[i] = (vertices_[i] - c).normalized() * distance + vertices_[i];
      // JSK_ROS_INFO("old v: [%f, %f, %f]", vertices_[i][0], vertices_[i][1], vertices_[i][2]);
      // JSK_ROS_INFO("new v: [%f, %f, %f]", new_vertices[i][0], new_vertices[i][1], new_vertices[i][2]);
      // JSK_ROS_INFO("");
    }
    
    ConvexPolygon::Ptr ret (new ConvexPolygon(new_vertices));
    return ret;
  }
  
  ConvexPolygon::Ptr ConvexPolygon::magnify(const double scale_factor)
  {
    // compute centroid
    Eigen::Vector3f c = centroid();
    Vertices new_vertices;
    for (size_t i = 0; i < vertices_.size(); i++) {
      new_vertices.push_back((vertices_[i] - c) * scale_factor + c);
    }
    ConvexPolygon::Ptr ret (new ConvexPolygon(new_vertices));
    return ret;
  }

  geometry_msgs::Polygon ConvexPolygon::toROSMsg()
  {
    geometry_msgs::Polygon polygon;
    for (size_t i = 0; i < vertices_.size(); i++) {
      geometry_msgs::Point32 ros_point;
      ros_point.x = vertices_[i][0];
      ros_point.y = vertices_[i][1];
      ros_point.z = vertices_[i][2];
      polygon.points.push_back(ros_point);
    }
    return polygon;
  }


  bool ConvexPolygon::isProjectableInside(const Eigen::Vector3f& p)
  {
    Eigen::Vector3f foot_point;
    Plane::project(p, foot_point);
    return isInside(foot_point);
  }

}
