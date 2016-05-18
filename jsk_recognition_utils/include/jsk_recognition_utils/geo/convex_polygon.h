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

#ifndef JSK_RECOGNITION_UTILS_GEO_CONVEX_POLYGON_H_
#define JSK_RECOGNITION_UTILS_GEO_CONVEX_POLYGON_H_

#include "jsk_recognition_utils/geo/polygon.h"
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include "jsk_recognition_utils/pcl_util.h"


namespace jsk_recognition_utils
{
  class ConvexPolygon: public Polygon
  {
  public:
    typedef boost::shared_ptr<ConvexPolygon> Ptr;
    typedef Eigen::Vector3f Vertex;
    typedef std::vector<Eigen::Vector3f,
                        Eigen::aligned_allocator<Eigen::Vector3f> > Vertices;
    // vertices should be CW
    ConvexPolygon(const Vertices& vertices);
    ConvexPolygon(const Vertices& vertices,
                  const std::vector<float>& coefficients);
    //virtual Polygon flip();
    virtual void project(const Eigen::Vector3f& p, Eigen::Vector3f& output);
    virtual void project(const Eigen::Vector3d& p, Eigen::Vector3d& output);
    virtual void project(const Eigen::Vector3d& p, Eigen::Vector3f& output);
    virtual void project(const Eigen::Vector3f& p, Eigen::Vector3d& output);
    virtual void projectOnPlane(const Eigen::Vector3f& p,
                                Eigen::Vector3f& output);
    virtual void projectOnPlane(const Eigen::Affine3f& p,
                                Eigen::Affine3f& output);
    virtual bool isProjectableInside(const Eigen::Vector3f& p);
    // p should be a point on the plane
    virtual ConvexPolygon flipConvex();
    virtual Eigen::Vector3f getCentroid();
    virtual Ptr magnify(const double scale_factor);
    virtual Ptr magnifyByDistance(const double distance);
    
    static ConvexPolygon fromROSMsg(const geometry_msgs::Polygon& polygon);
    static ConvexPolygon::Ptr fromROSMsgPtr(const geometry_msgs::Polygon& polygon);
    bool distanceSmallerThan(
      const Eigen::Vector3f& p, double distance_threshold);
    bool distanceSmallerThan(
      const Eigen::Vector3f& p, double distance_threshold,
      double& output_distance);
    bool allEdgesLongerThan(double thr);
    double distanceFromVertices(const Eigen::Vector3f& p);
    geometry_msgs::Polygon toROSMsg();
  protected:

  private:
  };

  template<class PointT>
  ConvexPolygon::Ptr convexFromCoefficientsAndInliers(
    const typename pcl::PointCloud<PointT>::Ptr cloud,
    const pcl::PointIndices::Ptr inliers,
    const pcl::ModelCoefficients::Ptr coefficients) {
    typedef typename pcl::PointCloud<PointT> POINTCLOUD;
    typename POINTCLOUD::Ptr projected_cloud(new pcl::PointCloud<PointT>);
    // check inliers has enough points
    if (inliers->indices.size() == 0) {
      return ConvexPolygon::Ptr();
    }
    // project inliers based on coefficients
    pcl::ProjectInliers<PointT> proj;
    proj.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    proj.setInputCloud(cloud);
    proj.setModelCoefficients(coefficients);
    proj.setIndices(inliers);
    proj.filter(*projected_cloud);
    // compute convex with giant mutex
    {
      boost::mutex::scoped_lock lock(global_chull_mutex);
      typename POINTCLOUD::Ptr convex_cloud(new pcl::PointCloud<PointT>);
      pcl::ConvexHull<PointT> chull;
      chull.setDimension(2);
      chull.setInputCloud (projected_cloud);
      chull.reconstruct (*convex_cloud);
      if (convex_cloud->points.size() > 0) {
        // convert pointcloud to vertices
        Vertices vs;
        for (size_t i = 0; i < convex_cloud->points.size(); i++) {
          Eigen::Vector3f v(convex_cloud->points[i].getVector3fMap());
          vs.push_back(v);
        }
        return ConvexPolygon::Ptr(new ConvexPolygon(vs));
      }
      else {
        return ConvexPolygon::Ptr();
      }
    }
  }
}

#endif
