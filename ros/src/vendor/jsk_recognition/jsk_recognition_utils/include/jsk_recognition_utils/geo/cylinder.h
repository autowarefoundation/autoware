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

#ifndef JSK_RECOGNITION_UTILS_GEO_CYLINDER_H_
#define JSK_RECOGNITION_UTILS_GEO_CYLINDER_H_

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

namespace jsk_recognition_utils
{
  class Cylinder                // infinite
  {
  public:
    typedef boost::shared_ptr<Cylinder> Ptr;
    Cylinder(Eigen::Vector3f point, Eigen::Vector3f direction, double radius);

    virtual void filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                  const double threshold,
                                  pcl::PointIndices& output);
    virtual void estimateCenterAndHeight(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                         const pcl::PointIndices& indices,
                                         Eigen::Vector3f& center,
                                         double& height);
    virtual void toMarker(visualization_msgs::Marker& marker,
                          const Eigen::Vector3f& center,
                          const Eigen::Vector3f& uz,
                          const double height);
    virtual Eigen::Vector3f getDirection();
    virtual double getRadius() { return radius_; }
  protected:
    Eigen::Vector3f point_;
    Eigen::Vector3f direction_;
    double radius_;
  private:
    
  };
}

#endif
