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

#include "jsk_recognition_utils/geo/cylinder.h"
#include "jsk_recognition_utils/geo_util.h"

namespace jsk_recognition_utils
{
  Cylinder::Cylinder(Eigen::Vector3f point, Eigen::Vector3f direction, double radius):
    point_(point), direction_(direction), radius_(radius)
  {

  }

  void Cylinder::filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                  const double threshold,
                                  pcl::PointIndices& output)
  {
    Line line(direction_, point_);
    output.indices.clear();
    for (size_t i = 0; i < cloud.points.size(); i++) {
      Eigen::Vector3f p = cloud.points[i].getVector3fMap();
      double d = line.distanceToPoint(p);
      if (d < radius_ + threshold && d > radius_ - threshold) {
        output.indices.push_back(i);
      }
    }
  }

  void Cylinder::estimateCenterAndHeight(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                         const pcl::PointIndices& indices,
                                         Eigen::Vector3f& center,
                                         double& height)
  {
    Line line(direction_, point_);
    Vertices points;
    for (size_t i = 0; i < indices.indices.size(); i++) {
      int point_index = indices.indices[i];
      points.push_back(cloud.points[point_index].getVector3fMap());
    }
    PointPair min_max = line.findEndPoints(points);
    Eigen::Vector3f min_point = min_max.get<0>();
    Eigen::Vector3f max_point = min_max.get<1>();
    Eigen::Vector3f min_point_projected, max_point_projected;
    line.foot(min_point, min_point_projected);
    line.foot(max_point, max_point_projected);
    height = (min_point_projected - max_point_projected).norm();
    center = (min_point_projected + max_point_projected) / 2.0;
  }

  void Cylinder::toMarker(visualization_msgs::Marker& marker,
                          const Eigen::Vector3f& center,
                          const Eigen::Vector3f& uz,
                          const double height)
  {
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.pose.position.x = center[0];
    marker.pose.position.y = center[1];
    marker.pose.position.z = center[2];
    Eigen::Vector3f orig_z(0, 0, 1);
    Eigen::Quaternionf q;
    q.setFromTwoVectors(orig_z, uz);
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.scale.x = radius_ * 2;
    marker.scale.y = radius_ * 2;
    marker.scale.z = height;
    marker.color.a = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
  }

  Eigen::Vector3f Cylinder::getDirection()
  {
    return direction_;
  }

}
