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
#define BOOST_PARAMETER_MAX_ARITY 7
#include "jsk_recognition_utils/grid_map.h"
#include <boost/make_shared.hpp>
#include <Eigen/Core>
#include "jsk_recognition_utils/geo_util.h"
#include <eigen_conversions/eigen_msg.h>
#include <nodelet/nodelet.h>
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <pcl/surface/convex_hull.h>
#include <jsk_topic_tools/log_utils.h>
//#define DEBUG_GRID_MAP

namespace jsk_recognition_utils
{
  GridMap::GridMap(double resolution, const std::vector<float>& coefficients):
    resolution_(resolution), vote_(0)
  {
    normal_[0] = -coefficients[0];
    normal_[1] = -coefficients[1];
    normal_[2] = -coefficients[2];
    d_ = -coefficients[3];
    if (normal_.norm() != 1.0) {
      d_ = d_ / normal_.norm();
      normal_.normalize();
    }
    O_ = - d_ * normal_;
    // decide ex_ and ey_
    Eigen::Vector3f u(1, 0, 0);
    if (normal_ == u) {
      u[0] = 0; u[1] = 1; u[2] = 0;
    }
    ey_ = normal_.cross(u).normalized();
    ex_ = ey_.cross(normal_).normalized();
  }
  
  GridMap::~GridMap()
  {
    
  }

  GridIndex::Ptr GridMap::registerIndex(const int x, const int y)
  {
    ColumnIterator it = data_.find(x);
    if (it != data_.end()) {
      (it->second).insert(y);
    }
    else {
      RowIndices new_row;
      new_row.insert(y);
      data_[x] = new_row;
    }
    GridIndex::Ptr ret(new GridIndex(x, y));
    return ret;
  }
  
  GridIndex::Ptr GridMap::registerIndex(const GridIndex::Ptr& index)
  {
    return registerIndex(index->x, index->y);
  }
  
  void GridMap::registerPoint(const pcl::PointXYZRGB& point)
  {
    GridIndex::Ptr index (new GridIndex());
    pointToIndex(point, index);
    // check duplication
    registerIndex(index);
  }
  
  std::vector<GridIndex::Ptr> GridMap::registerLine(const pcl::PointXYZRGB& from,
                                                 const pcl::PointXYZRGB& to)
  {
#ifdef DEBUG_GRID_MAP
    std::cout << "newline" << std::endl;
#endif
    std::vector<GridIndex::Ptr> added_indices;
    //GridLine::Ptr new_line (new GridLine(from, to));
    //lines_.push_back(new_line);
    // count up all the grids which the line penetrates
    
    // 1. convert to y = ax + b style equation.
    // 2. move x from the start index to the end index and count up the y range
    // if it cannot be convert to y = ax + b style, it means the equation
    // is represented as x = c style.
    double from_x = from.getVector3fMap().dot(ex_) / resolution_;
    double from_y = from.getVector3fMap().dot(ey_) / resolution_;
    double to_x = to.getVector3fMap().dot(ex_) / resolution_;
    double to_y = to.getVector3fMap().dot(ey_) / resolution_;
#ifdef DEBUG_GRID_MAP
    std::cout << "registering (" << (int)from_x << ", " << (int)from_y << ")" << std::endl;
    std::cout << "registering (" << (int)to_x << ", " << (int)to_y << ")" << std::endl;
#endif
    added_indices.push_back(registerIndex(from_x, from_y));
    added_indices.push_back(registerIndex(to_x, to_y));
    if (from_x != to_x) {
      double a = (to_y - from_y) / (to_x - from_x);
      double b = - a * from_x + from_y;
#ifdef DEBUG_GRID_MAP
      std::cout << "a: " << a << std::endl;
#endif
      if (a == 0.0) {
#ifdef DEBUG_GRID_MAP
        std::cout << "parallel to x" << std::endl;
#endif
        int from_int_x = (int)from_x;
        int to_int_x = (int)to_x;
        int int_y = (int)from_y;
        if (from_int_x > to_int_x) {
          std::swap(from_int_x, to_int_x);
        }
        for (int ix = from_int_x; ix < to_int_x; ++ix) {
          added_indices.push_back(registerIndex(ix, int_y));
#ifdef DEBUG_GRID_MAP
          std::cout << "registering (" << ix << ", " << int_y << ")" << std::endl;
#endif
        }
      }
      else if (fabs(a) < 1.0) {
#ifdef DEBUG_GRID_MAP
        std::cout << "based on x" << std::endl;
#endif
        int from_int_x = (int)from_x;
        int to_int_x = (int)to_x;
        if (from_int_x > to_int_x) {
          std::swap(from_int_x, to_int_x);
        }
        
        for (int ix = from_int_x; ix < to_int_x; ++ix) {
          double y = a * ix + b;
          added_indices.push_back(registerIndex(ix, (int)y));
#ifdef DEBUG_GRID_MAP
          std::cout << "registering (" << ix << ", " << (int)y << ")" << std::endl;
#endif
        }
      }
      else {
#ifdef DEBUG_GRID_MAP
        std::cout << "based on y" << std::endl;
#endif
        int from_int_y = (int)from_y;
        int to_int_y = (int)to_y;
        if (from_int_y > to_int_y) {
          std::swap(from_int_y, to_int_y);
        }
        
        for (int iy = from_int_y; iy < to_int_y; ++iy) {
          double x = iy / a - b / a;
          added_indices.push_back(registerIndex((int)x, iy));
#ifdef DEBUG_GRID_MAP
          std::cout << "registering (" << (int)x << ", " << iy << ")" << std::endl;
#endif
        }
      }
      
    }
    else {
#ifdef DEBUG_GRID_MAP
      std::cout << "parallel to y" << std::endl;
#endif
      // the line is parallel to y
      int from_int_y = (int)from_y;
      int to_int_y = (int)to_y;
      int int_x = (int)from_x;
      if (from_int_y > to_int_y) {
        std::swap(from_int_y, to_int_y);
      }
      for (int iy = from_int_y; iy < to_int_y; ++iy) {
        added_indices.push_back(registerIndex(int_x, iy));
#ifdef DEBUG_GRID_MAP
        std::cout << "registering (" << int_x << ", " << (int)iy << ")" << std::endl;
#endif
      }
    }
    return added_indices;
  }

  void GridMap::registerPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
  {
    for (size_t i = 0; i < cloud->points.size(); i++) {
      registerPoint(cloud->points[i]);
      //JSK_ROS_INFO("registered point: [%f, %f, %f]", cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
    }
  }
  
  void GridMap::pointToIndex(const pcl::PointXYZRGB& point, GridIndex::Ptr index)
  {
    pointToIndex(point.getVector3fMap(), index);
  }

  void GridMap::pointToIndex(const Eigen::Vector3f& p, GridIndex::Ptr index)
  {
    index->x = (p - O_).dot(ex_) / resolution_;
    index->y = (p - O_).dot(ey_) / resolution_;
  }

  void GridMap::gridToPoint(GridIndex::Ptr index, Eigen::Vector3f& pos)
  {
    gridToPoint(*index, pos);
  }

  void GridMap::gridToPoint(const GridIndex& index, Eigen::Vector3f& pos)
  {
    //pos = resolution_ * (index.x * ex_ + index.y * ey_) + O_;
    pos = resolution_ * ((index.x + 0.5) * ex_ + (index.y + 0.5) * ey_) + O_;
  }

  void GridMap::gridToPoint2(const GridIndex& index, Eigen::Vector3f& pos)
  {
    //pos = resolution_ * ((index.x - 0.5) * ex_ + (index.y - 0.5) * ey_) + O_;
    pos = resolution_ * ((index.x - 0.0) * ex_ + (index.y - 0.0) * ey_) + O_;
  }

  
  bool GridMap::getValue(const int x, const int y)
  {
    // check line
    // for (size_t i = 0; i < lines_.size(); i++) {
    //   GridLine::Ptr line = lines_[i];
    //   Eigen::Vector3f A, B, C, D;
    //   gridToPoint2(GridIndex(x, y), A);
    //   gridToPoint2(GridIndex(x + 1, y), B);
    //   gridToPoint2(GridIndex(x + 1, y + 1), C);
    //   gridToPoint2(GridIndex(x, y + 1), D);
    //   bool penetrate = line->penetrateGrid(A, B, C, D);
    //   if (penetrate) {
    //   //   // printf("(%lf, %lf, %lf) - (%lf, %lf, %lf) penetrate (%d, %d)\n",
    //   //   //        line->from[0],line->from[1],line->from[2],
    //   //   //        line->to[0],line->to[1],line->to[2],
    //   //   //        x, y);
    //   //   //std::cout << "penetrate"
    //     return true;
    //   }
    // }

    ColumnIterator it = data_.find(x);
    if (it == data_.end()) {
      return false;
    }
    else {
      RowIndices c = it->second;
      if (c.find(y) == c.end()) {
        return false;
      }
      else {
        return true;
      }
    }
  }
  
  bool GridMap::getValue(const GridIndex& index)
  {
    return getValue(index.x, index.y);
  }
  
  bool GridMap::getValue(const GridIndex::Ptr& index)
  {
    return getValue(*index);
  }

  void GridMap::fillRegion(const GridIndex::Ptr start, std::vector<GridIndex::Ptr>& output)
  {
#ifdef DEBUG_GRID_MAP
    std::cout << "filling " << start->x << ", " << start->y << std::endl;
#endif
    output.push_back(start);
    registerIndex(start);
#ifdef DEBUG_GRID_MAP
    if (abs(start->x) > 100 || abs(start->y) > 100) {
      //exit(1);
      std::cout << "force to quit" << std::endl;
      for (size_t i = 0; i < lines_.size(); i++) {
        GridLine::Ptr line = lines_[i];
        Eigen::Vector3f from = line->from;
        Eigen::Vector3f to = line->to;
#ifdef DEBUG_GRID_MAP
        std::cout << "line[" << i << "]: "
                  << "[" << from[0] << ", " << from[1] << ", " << from[2] << "] -- "
                  << "[" << to[0] << ", " << to[1] << ", " << to[2] << std::endl;
#endif
      }
      return;                   // force to quit
    }
#endif
    GridIndex U(start->x, start->y + 1),
              D(start->x, start->y - 1),
              R(start->x + 1, start->y),
              L(start->x - 1, start->y);
    
    if (!getValue(U)) {
      fillRegion(boost::make_shared<GridIndex>(U), output);
    }
    if (!getValue(L)) {
      fillRegion(boost::make_shared<GridIndex>(L), output);
    }
    if (!getValue(R)) {
      fillRegion(boost::make_shared<GridIndex>(R), output);
    }
    if (!getValue(D)) {
      fillRegion(boost::make_shared<GridIndex>(D), output);
    }
    
  }
  
  void GridMap::fillRegion(const Eigen::Vector3f& start, std::vector<GridIndex::Ptr>& output)
  {
    GridIndex::Ptr start_index (new GridIndex);
    pointToIndex(start, start_index);
    fillRegion(start_index, output);
  }
  
  void GridMap::indicesToPointCloud(const std::vector<GridIndex::Ptr>& indices,
                                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
  {
    for (size_t i = 0; i < indices.size(); i++) {
      GridIndex::Ptr index = indices[i];
      Eigen::Vector3f point;
      pcl::PointXYZRGB new_point;
      gridToPoint(index, point);
      new_point.x = point[0];
      new_point.y = point[1];
      new_point.z = point[2];
      cloud->points.push_back(new_point);
    }
  }

  void GridMap::originPose(Eigen::Affine3f& output)
  {
    Eigen::Matrix3f rot_mat;
    rot_mat.col(0) = Eigen::Vector3f(ex_[0], ex_[1], ex_[2]);
    rot_mat.col(1) = Eigen::Vector3f(ey_[0], ey_[1], ey_[2]);
    rot_mat.col(2) = Eigen::Vector3f(normal_[0], normal_[1], normal_[2]);
    JSK_ROS_DEBUG("O: [%f, %f, %f]", O_[0], O_[1], O_[2]);
    JSK_ROS_DEBUG("ex: [%f, %f, %f]", ex_[0], ex_[1], ex_[2]);
    JSK_ROS_DEBUG("ey: [%f, %f, %f]", ey_[0], ey_[1], ey_[2]);
    JSK_ROS_DEBUG("normal: [%f, %f, %f]", normal_[0], normal_[1], normal_[2]);
    output = Eigen::Translation3f(O_) * Eigen::Quaternionf(rot_mat);
  }
  
  void GridMap::originPose(Eigen::Affine3d& output)
  {
    Eigen::Affine3f float_affine;
    originPose(float_affine);
    convertEigenAffine3(float_affine, output);
  }
  
  void GridMap::toMsg(jsk_recognition_msgs::SparseOccupancyGrid& grid)
  {
    grid.resolution = resolution_;
    // compute origin POSE from O and normal_, d_
    Eigen::Affine3d plane_pose;
    originPose(plane_pose);
    tf::poseEigenToMsg(plane_pose, grid.origin_pose);
    for (ColumnIterator it = data_.begin();
         it != data_.end();
         it++) {
      int column_index = it->first;
      RowIndices row_indices = it->second;
      jsk_recognition_msgs::SparseOccupancyGridColumn ros_column;
      ros_column.column_index = column_index;
      for (RowIterator rit = row_indices.begin();
           rit != row_indices.end();
           rit++) {
        jsk_recognition_msgs::SparseOccupancyGridCell cell;
        cell.row_index = *rit;
        cell.value = 1.0;
        ros_column.cells.push_back(cell);
      }
      grid.columns.push_back(ros_column);
    }
  }

  Plane GridMap::toPlane()
  {
    return Plane(normal_, d_);
  }

  Plane::Ptr GridMap::toPlanePtr()
  {
    Plane::Ptr ret (new Plane(normal_, d_));
    return ret;
  }


  std::vector<float> GridMap::getCoefficients()
  {
    std::vector<float> output;
    output.push_back(normal_[0]);
    output.push_back(normal_[1]);
    output.push_back(normal_[2]);
    output.push_back(d_);
    return output;
  }

  void GridMap::vote()
  {
    ++vote_;
  }

  unsigned int GridMap::getVoteNum()
  {
    return vote_;
  }

  void GridMap::setGeneration(unsigned int generation) {
    generation_ = generation;
  }

  unsigned int GridMap::getGeneration()
  {
    return generation_;
  }
  
  void GridMap::removeIndex(const GridIndex::Ptr& index)
  {
    int x = index->x;
    int y = index->y;
    ColumnIterator it = data_.find(x);
    if (it != data_.end()) {
      RowIterator rit = (it->second).find(y);
      if (rit != it->second.end()) {
        it->second.erase(rit);
      }
    }
  }
  bool GridMap::isBinsOccupied(const Eigen::Vector3f& p)
  {
    GridIndex::Ptr ret (new GridIndex());
    pointToIndex(p, ret);
    //JSK_ROS_INFO("checking (%d, %d)", ret->x, ret->y);
    return getValue(ret);
  }

  boost::tuple<int, int> GridMap::minMaxX()
  {
    int min_x = INT_MAX;
    int max_x = - INT_MAX;
    for (ColumnIterator it = data_.begin();
         it != data_.end();
         ++it) {
      int x = it->first;
      if (min_x > x) {
        min_x = x;
      }
      if (max_x < x) {
        max_x = x;
      }
    }
    return boost::make_tuple<int, int>(min_x, max_x);
  }

  boost::tuple<int, int> GridMap::minMaxY()
  {
    int min_y = INT_MAX;
    int max_y = - INT_MAX;
    for (ColumnIterator it = data_.begin();
         it != data_.end();
         ++it) {
      RowIndices row_indices = it->second;
      for (RowIterator rit = row_indices.begin();
           rit != row_indices.end();
           rit++) {
        int y = *rit;
        if (min_y > y) {
          min_y = y;
        }
        if (max_y < y) {
          max_y = y;
        }
      }
    }
    return boost::make_tuple<int, int>(min_y, max_y);
  }
  
  int GridMap::normalizedWidth()
  {
    boost::tuple<int, int> min_max_x = minMaxX();
    return min_max_x.get<1>() - min_max_x.get<0>();
  }

  int GridMap::normalizedHeight()
  {
    boost::tuple<int, int> min_max_y = minMaxY();
    return min_max_y.get<1>() - min_max_y.get<0>();
  }

  int GridMap::widthOffset()
  {
    boost::tuple<int, int> min_max_x = minMaxX();
    int min_x = min_max_x.get<0>();
    return - min_x;
  }

  int GridMap::heightOffset()
  {
    boost::tuple<int, int> min_max_y = minMaxY();
    int min_y = min_max_y.get<0>();
    return - min_y;
  }

  int GridMap::normalizedIndex(int width_offset, int height_offset,
                               int step,
                               int elem_size,
                               int original_x, int original_y)
  {
    int x = original_x + width_offset;
    int y = original_y + height_offset;
    return y * step + x * elem_size;
  }
  
  
  cv::Mat GridMap::toImage()
  {
    // initialize with black
    int width = normalizedWidth();
    int height = normalizedHeight();
    int width_offset = widthOffset();
    int height_offset = heightOffset();
    cv::Mat m = cv::Mat(width, height, CV_8UC1) * 0;
    // for all index
    for (ColumnIterator it = data_.begin();
         it != data_.end();
         ++it) {
      for (RowIterator rit = it->second.begin();
           rit != it->second.end();
           ++rit) {
        m.data[normalizedIndex(width_offset, height_offset,
                               m.step, m.elemSize(),
                               it->first, *rit)] = 255;
      }
    }
    
    return m;
  }

  bool GridMap::check4Neighbor(int x, int y) {
    if (getValue(x + 1, y) &&
        getValue(x + 1, y + 1) &&
        getValue(x - 1, y) &&
        getValue(x - 1, y - 1)) {
      return true;
    }
    else {
      return false;
    }
  }
  
  void GridMap::decreaseOne()
  {
    //Columns new_data;
    GridMap::Ptr new_map (new GridMap(resolution_, getCoefficients()));
    for (ColumnIterator it = data_.begin();
         it != data_.end();
         it++) {
      RowIndices row_indices = it->second;
      int x = it->first;
      for (RowIterator rit = row_indices.begin();
           rit != row_indices.end();
           rit++) {
        int y = *rit;
        if (check4Neighbor(x, y)) {
          new_map->registerIndex(x, y);
        }
      }
    }
    data_ = new_map->data_;
  }
  
  void GridMap::decrease(int i)
  {
    for (int ii = 0; ii < i; ii++) {
      decreaseOne();
    }
  }

  void GridMap::add(GridMap& other)
  {
    for (ColumnIterator it = other.data_.begin();
         it != other.data_.end();
         it++) {
      RowIndices row_indices = it->second;
      int x = it->first;
      for (RowIterator rit = row_indices.begin();
           rit != row_indices.end();
           rit++) {
        int y = *rit;
        Eigen::Vector3f pos;
        GridIndex index(x, y);
        other.gridToPoint(index, pos);
        pcl::PointXYZRGB p;
        pointFromVectorToXYZ<Eigen::Vector3f, pcl::PointXYZRGB>(pos, p);
        registerPoint(p);
      }
    }
  }
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr GridMap::toPointCloud()
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    for (ColumnIterator it = data_.begin();
         it != data_.end();
         it++) {
      RowIndices row_indices = it->second;
      int x = it->first;
      for (RowIterator rit = row_indices.begin();
           rit != row_indices.end();
           rit++) {
        int y = *rit;
        Eigen::Vector3f pos;
        GridIndex index(x, y);
        gridToPoint(index, pos);
        pcl::PointXYZ p;
        pointFromVectorToXYZ<Eigen::Vector3f, pcl::PointXYZ>(pos, p);
        cloud->points.push_back(p);
      }
    }
    return cloud;
  }

  ConvexPolygon::Ptr GridMap::toConvexPolygon()
  {
    // 1. build pointcloud
    // 2. compute convex hull
    // 3. return it as ConvexPolygon
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = toPointCloud();
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud(cloud);
    chull.setDimension(2);
    pcl::PointCloud<pcl::PointXYZ> chull_output;
    chull.reconstruct(chull_output);
    // convex chull_output to Vertices
    Vertices vs;
    for (size_t i = 0; i < chull_output.points.size(); i++) {
      Eigen::Vector3f v = chull_output.points[i].getVector3fMap();
      vs.push_back(v);
    }
    return ConvexPolygon::Ptr(new ConvexPolygon(vs));
  }

}
