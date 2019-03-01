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

#ifndef JSK_RECOGNITION_UTILS_GEO_POLYGON_H_
#define JSK_RECOGNITION_UTILS_GEO_POLYGON_H_

#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/random.hpp>
#include <geometry_msgs/Polygon.h>
#include <jsk_recognition_msgs/PolygonArray.h>

#include "jsk_recognition_utils/geo/plane.h"
#include "jsk_recognition_utils/geo/segment.h"
#include "jsk_recognition_utils/sensor_model/camera_depth_sensor.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include "jsk_recognition_utils/random_util.h"

namespace jsk_recognition_utils
{
  class Polygon: public Plane
  {
  public:
    typedef boost::shared_ptr<Polygon> Ptr;
    typedef boost::tuple<Ptr, Ptr> PtrPair;
    Polygon(const Vertices& vertices);
    Polygon(const Vertices& vertices,
            const std::vector<float>& coefficients);
    virtual ~Polygon();
    virtual std::vector<Polygon::Ptr> decomposeToTriangles();
    virtual void clearTriangleDecompositionCache() {
      cached_triangles_.clear();
    }
    
    virtual Eigen::Vector3f getNormalFromVertices();
    virtual bool isTriangle();
    Eigen::Vector3f randomSampleLocalPoint(boost::mt19937& random_generator);
    virtual void getLocalMinMax(double& min_x, double& min_y,
                                double& max_x, double& max_y);
    template <class PointT>
    typename pcl::PointCloud<PointT>::Ptr samplePoints(double grid_size)
    {
      typename pcl::PointCloud<PointT>::Ptr
        ret (new pcl::PointCloud<PointT>);
      double min_x, min_y, max_x, max_y;
      getLocalMinMax(min_x, min_y, max_x, max_y);
      // ROS_INFO("min_x: %f", min_x);
      // ROS_INFO("min_y: %f", min_y);
      // ROS_INFO("max_x: %f", max_x);
      // ROS_INFO("max_y: %f", max_y);
      // Decompose into triangle first for optimization
      std::vector<Polygon::Ptr> triangles = decomposeToTriangles();

      for (double x = min_x; x < max_x; x += grid_size) {
        for (double y = min_y; y < max_y; y += grid_size) {
          Eigen::Vector3f candidate(x, y, 0);
          Eigen::Vector3f candidate_global = coordinates() * candidate;
          // check candidate is inside of the polygon or not
          bool insidep = false;
          for (size_t i = 0; i < triangles.size(); i++) {
            if (triangles[i]->isInside(candidate_global)) {
              insidep = true;
              break;
            }
          }
          if (insidep) {
            PointT p;
            p.x = candidate_global[0];
            p.y = candidate_global[1];
            p.z = candidate_global[2];
            p.normal_x = normal_[0];
            p.normal_y = normal_[1];
            p.normal_z = normal_[2];
            ret->points.push_back(p);
          }
        }
      }
      return ret;
    }

    /**
     * @brief
     * get all the edges as point of Segment.
     */
    std::vector<Segment::Ptr> edges() const;
    
    /**
     * @brief
     * Compute distance between point and this polygon.
     */
    double distance(const Eigen::Vector3f& point);

    /**
     * @brief
     * Compute distance between point and this polygon.
     * Nearest point on this polygon can be gotten.
     */
    double distance(const Eigen::Vector3f& point,
                    Eigen::Vector3f& nearest_point);

    /**
     * @brief
     * Compute nearest point from p on this polygon.
     * 
     * This method first project p onth the polygon and
     * if the projected point is inside of polygon,
     * the projected point is the nearest point.
     * If not, distances between the point and edges are
     * computed and search the nearest point.
     *
     * In case of searching edges, it is achieved in brute-force
     * searching and if the number of edges is huge, it may take
     * a lot of time to compute.
     *
     * This method cannot be used for const instance because
     * triangle decomposition will change the cache in the instance.
     *
     * Distance between p and nearest point is stored in distance.
     */
    virtual Eigen::Vector3f nearestPoint(const Eigen::Vector3f& p,
                                         double& distance);
    virtual size_t getNumVertices();
    virtual size_t getFarestPointIndex(const Eigen::Vector3f& O);
    virtual Eigen::Vector3f directionAtPoint(size_t i);
    virtual Eigen::Vector3f getVertex(size_t i);
    virtual PointIndexPair getNeighborIndex(size_t index);
    virtual Vertices getVertices() { return vertices_; };
    virtual double area();
    virtual bool isPossibleToRemoveTriangleAtIndex(
      size_t index,
      const Eigen::Vector3f& direction);
    virtual PtrPair separatePolygon(size_t index);
    /**
     * @brief
     * return true if p is inside of polygon.
     * p should be in global coordinates.
     */
    virtual bool isInside(const Eigen::Vector3f& p);
    size_t previousIndex(size_t i);
    size_t nextIndex(size_t i);
    
    static Polygon fromROSMsg(const geometry_msgs::Polygon& polygon);
    static Polygon::Ptr fromROSMsgPtr(const geometry_msgs::Polygon& polygon);
    static Polygon createPolygonWithSkip(const Vertices& vertices);

    /**
     * @brief
     * convert jsk_recognition_msgs::PolygonArray
     * to std::vector<Polygon::Ptr>.
     * It requires offset transformation in the 2nd argument.
     */
    static std::vector<Polygon::Ptr> fromROSMsg(const jsk_recognition_msgs::PolygonArray& msg,
                                                const Eigen::Affine3f& offset = Eigen::Affine3f::Identity());
    
    /**
     * @brief
     * transform Polygon with given transform.
     * cached triangles is cleared.
     */
    virtual void transformBy(const Eigen::Affine3d& transform);

    /**
     * @brief
     * transform Polygon with given transform.
     * cached triangles is cleared.
     */
    virtual void transformBy(const Eigen::Affine3f& transform);
    
    /**
     * @brief
     * generate mask image of the polygon.
     * if all the points are outside of field-of-view, 
     * returns false.
     */
    virtual bool maskImage(const jsk_recognition_utils::CameraDepthSensor& model,
                           cv::Mat& image) const;
    
    /**
     * @brief
     * draw line of polygons on image.
     */
    virtual void drawLineToImage(const jsk_recognition_utils::CameraDepthSensor& model,
                                 cv::Mat& image,
                                 const cv::Scalar& color,
                                 const int line_width = 1) const;
    virtual bool isConvex();
    virtual Eigen::Vector3f centroid();
    template<class PointT> void boundariesToPointCloud(
      pcl::PointCloud<PointT>& output) {
      output.points.resize(vertices_.size());
      for (size_t i = 0; i < vertices_.size(); i++) {
        Eigen::Vector3f v = vertices_[i];
        PointT p;
        p.x = v[0]; p.y = v[1]; p.z = v[2];
        output.points[i] = p;
      }
      output.height = 1;
      output.width = output.points.size();
    }
    
  protected:
    Vertices vertices_;
    std::vector<Polygon::Ptr> cached_triangles_;
  private:
    
  };
}

#endif
