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

#ifndef JSK_RECOGNITION_UTILS_GEO_GRID_PLANE_H_
#define JSK_RECOGNITION_UTILS_GEO_GRID_PLANE_H_

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/point_types.h>
#include <boost/tuple/tuple.hpp>
#include <vector>
#include <set>

#include "jsk_recognition_utils/geo/convex_polygon.h"
#include "jsk_recognition_utils/geo/cube.h"

#include <jsk_recognition_msgs/SimpleOccupancyGrid.h>

namespace jsk_recognition_utils
{
  /**
   * @brief
   * Grid based representation of planar region.
   *
   * Each cell represents a square region as belows:
   *        +--------+
   *        |        |
   *        |   +    |
   *        |        |
   *        +--------+
   *
   * The width and height of the cell is equivalent to resolution_,
   * and the value of cells_ represents a center point.
   * (i, j) means rectanglar region of (x, y) which satisfies followings:
   * i * resolution - 0.5 * resolution <= x < i * resolution + 0.5 * resolution
   * j * resolution - 0.5 * resolution <= y < j * resolution + 0.5 * resolution
   * 
   *
   */
  class GridPlane
  {
  public:
    typedef boost::shared_ptr<GridPlane> Ptr;
    typedef boost::tuple<int, int> IndexPair;
    typedef std::set<IndexPair> IndexPairSet;
    GridPlane(ConvexPolygon::Ptr plane, const double resolution);
    virtual ~GridPlane();
    virtual GridPlane::Ptr clone(); // shallow copy
    virtual size_t fillCellsFromPointCloud(
      pcl::PointCloud<pcl::PointNormal>::Ptr& cloud,
      double distance_threshold);
    virtual size_t fillCellsFromPointCloud(
      pcl::PointCloud<pcl::PointNormal>::Ptr& cloud,
      double distance_threshold,
      std::set<int>& non_plane_indices);
    virtual size_t fillCellsFromPointCloud(
      pcl::PointCloud<pcl::PointNormal>::Ptr& cloud,
      double distance_threshold,
      double normal_threshold,
      std::set<int>& non_plane_indices);
    virtual void fillCellsFromCube(Cube& cube);
    virtual double getResolution() { return resolution_; }
    virtual jsk_recognition_msgs::SimpleOccupancyGrid toROSMsg();
    /**
     * @brief
     * Construct GridPlane object from
     * jsk_recognition_msgs::SimpleOccupancyGrid.
     */
    static GridPlane fromROSMsg(
      const jsk_recognition_msgs::SimpleOccupancyGrid& rosmsg,
      const Eigen::Affine3f& offset);
    virtual bool isOccupied(const IndexPair& pair);
    
    /**
     * @brief
     * p should be local coordinate
     */
    virtual bool isOccupied(const Eigen::Vector3f& p);

    /**
     * @brief
     * p should be global coordinate
     */
    virtual bool isOccupiedGlobal(const Eigen::Vector3f& p);
    
    /**
     * @brief
     * Project 3-D point to GridPlane::IndexPair.
     * p should be represented in local coordinates.
     */
    virtual IndexPair projectLocalPointAsIndexPair(const Eigen::Vector3f& p);

    /**
     * @brief
     * Unproject GridPlane::IndexPair to 3-D local point.
     */
    virtual Eigen::Vector3f unprojectIndexPairAsLocalPoint(const IndexPair& pair);

    /**
     * @brief
     * Unproject GridPlane::IndexPair to 3-D global point.
     */
    virtual Eigen::Vector3f unprojectIndexPairAsGlobalPoint(const IndexPair& pair);

    /**
     * @brief
     * Add IndexPair to this instance.
     */
    virtual void addIndexPair(IndexPair pair);

    /**
     * @brief
     * Erode grid cells with specified number of pixels
     */
    virtual GridPlane::Ptr erode(int num);

    /**
     * @brief
     * return ConvexPolygon pointer of this instance.
     */
    virtual ConvexPolygon::Ptr getPolygon() { return convex_; }
    
    /**
     * @brief
     * Dilate grid cells with specified number of pixels
     */
    virtual GridPlane::Ptr dilate(int num);
  protected:
    ConvexPolygon::Ptr convex_;
    IndexPairSet cells_;
    double resolution_;
  private:
    
  };

}

#endif
