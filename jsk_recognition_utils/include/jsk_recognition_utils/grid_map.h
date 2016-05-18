// -*- mode: C++ -*-
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
#ifndef JSK_RECOGNITION_UTILS_GRID_MAP_H_
#define JSK_RECOGNITION_UTILS_GRID_MAP_H_
#include <jsk_recognition_msgs/SparseOccupancyGrid.h>

#include "jsk_recognition_utils/grid_index.h"
#include "jsk_recognition_utils/grid_line.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <map>
#include <set>
#include <Eigen/Geometry>
#include <boost/tuple/tuple.hpp>

#include "jsk_recognition_utils/geo_util.h"
#include <opencv2/opencv.hpp>

namespace jsk_recognition_utils
{

   // infinity range, might be slow...
  class GridMap
  {
  public:
    typedef boost::shared_ptr<GridMap> Ptr;
    typedef std::set<int> RowIndices;
    typedef std::map<int, RowIndices> Columns;
    typedef Columns::iterator ColumnIterator;
    typedef std::set<int>::iterator RowIterator;
    GridMap(double resolution, const std::vector<float>& coefficients);
    virtual ~GridMap();
    virtual void registerPoint(const pcl::PointXYZRGB& point);
    virtual std::vector<GridIndex::Ptr> registerLine(const pcl::PointXYZRGB& from, const pcl::PointXYZRGB& to);
    virtual void removeIndex(const GridIndex::Ptr& index);
    virtual void registerPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    virtual GridIndex::Ptr registerIndex(const GridIndex::Ptr& index);
    virtual GridIndex::Ptr registerIndex(const int x, const int y);
    virtual void pointToIndex(const pcl::PointXYZRGB& point, GridIndex::Ptr index);
    virtual void pointToIndex(const Eigen::Vector3f& point, GridIndex::Ptr index);
    virtual void indicesToPointCloud(const std::vector<GridIndex::Ptr>& indices,
                                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    virtual bool getValue(const GridIndex::Ptr& index);
    virtual bool getValue(const GridIndex& index);
    virtual bool getValue(const int x, const int y);
    virtual void gridToPoint(GridIndex::Ptr index, Eigen::Vector3f& pos);
    virtual void gridToPoint(const GridIndex& index, Eigen::Vector3f& pos);
    virtual void gridToPoint2(const GridIndex& index, Eigen::Vector3f& pos);
    virtual void fillRegion(const Eigen::Vector3f& start, std::vector<GridIndex::Ptr>& output);
    virtual void fillRegion(const GridIndex::Ptr start, std::vector<GridIndex::Ptr>& output);
    // toMsg does not fill header, be carefull
    virtual void originPose(Eigen::Affine3f& output);
    virtual void originPose(Eigen::Affine3d& output);
    virtual void toMsg(jsk_recognition_msgs::SparseOccupancyGrid& grid);
    virtual Plane toPlane();
    virtual Plane::Ptr toPlanePtr();
    virtual void vote();
    virtual unsigned int getVoteNum();
    virtual void setGeneration(unsigned int generation);
    virtual unsigned int getGeneration();
    virtual std::vector<float> getCoefficients();
    virtual bool isBinsOccupied(const Eigen::Vector3f& p);
    virtual int normalizedWidth();
    virtual int normalizedHeight();
    virtual boost::tuple<int, int> minMaxX();
    virtual boost::tuple<int, int> minMaxY();
    virtual int widthOffset();
    virtual int heightOffset();
    virtual int normalizedIndex(int width_offset, int height_offset,
                                int step,
                                int elem_size,
                                int original_x, int original_y);
    virtual cv::Mat toImage();
    virtual bool check4Neighbor(int x, int y);
    virtual ConvexPolygon::Ptr toConvexPolygon();
    virtual pcl::PointCloud<pcl::PointXYZ>::Ptr toPointCloud();
    virtual void decrease(int i);
    virtual void add(GridMap& other);
  protected:
    virtual void decreaseOne();
    
    double resolution_;
    Eigen::Vector3f O_;
    
    // plane parameter
    Eigen::Vector3f normal_;
    double d_;
    
    Eigen::Vector3f ex_, ey_;
    
    std::vector<GridLine::Ptr> lines_;
    Columns data_;
    unsigned int vote_;
    unsigned int generation_;
  private:
  };
  
}

#endif
