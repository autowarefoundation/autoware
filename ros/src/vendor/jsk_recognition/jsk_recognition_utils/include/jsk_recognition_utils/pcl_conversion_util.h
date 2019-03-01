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

#ifndef JSK_RECOGNITION_UTILS_PCL_CONVERSION_UTIL_H_
#define JSK_RECOGNITION_UTILS_PCL_CONVERSION_UTIL_H_
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/Point32.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/range_image/range_image_planar.h>
#include <visualization_msgs/Marker.h>
#if ROS_VERSION_MINIMUM(1, 10, 0)
// hydro and later
typedef pcl_msgs::PointIndices PCLIndicesMsg;
typedef pcl_msgs::ModelCoefficients PCLModelCoefficientMsg;
#else
// groovy
typedef pcl::PointIndices PCLIndicesMsg;
typedef pcl::ModelCoefficients PCLModelCoefficientMsg;
#endif

#include <opencv2/opencv.hpp>

namespace jsk_recognition_utils
{

  /** @brief
   * Convert pcl::RangeImage to cv::Mat. Distance is normalized
   * to 0-1 and colorized.
   *
   * @param range_image instance of pcl::RangeImage
   * @param mat instance of cv::Mat, converted cv::Mat is set into
   *        this argument.
   */
  void rangeImageToCvMat(const pcl::RangeImage& range_image,
                         cv::Mat& mat);
  
  template<class FromT, class ToT>
  void pointFromXYZToVector(const FromT& msg,
                            ToT& p)
  {
    p[0] = msg.x; p[1] = msg.y; p[2] = msg.z;
  }

  template<class FromT, class ToT>
  void pointFromVectorToXYZ(const FromT& p,
                            ToT& msg)
  {
    msg.x = p[0]; msg.y = p[1]; msg.z = p[2];
  }

  template<class FromT, class ToT>
  void pointFromXYZToXYZ(const FromT& from,
                         ToT& to)
  {
    to.x = from.x; to.y = from.y; to.z = from.z;
  }

  template<class FromT, class ToT>
  void pointFromVectorToVector(const FromT& from,
                               ToT& to)
  {
    to[0] = from[0]; to[1] = from[1]; to[2] = from[2];
  }

  template<class FromT, class ToT>
  void convertMatrix4(const FromT& from,
                      ToT& to)
  {
    for (size_t i = 0; i < 4; i++) {
      for (size_t j = 0; j < 4; j++) {
        to(i, j) = from(i, j);
      }
    }
  }

  void convertEigenAffine3(const Eigen::Affine3d& from,
                           Eigen::Affine3f& to);
  void convertEigenAffine3(const Eigen::Affine3f& from,
                           Eigen::Affine3d& to);

  template <class PointT>
  void markerMsgToPointCloud(const visualization_msgs::Marker& input_marker,
                             int sample_nums,
                             pcl::PointCloud<PointT>& output_cloud)
  {
    std::vector<double> cumulative_areas;
    double total_area = 0;

    if (input_marker.points.size() != input_marker.colors.size()){
      ROS_ERROR("Color and Points nums is different in markerMsgToPointCloud");
      return;
    }

    //Gether the triangle areas
    for (int i = 0; i < (int)input_marker.points.size()/3; i++){
      geometry_msgs::Point p0_1;
      p0_1.x = input_marker.points[i*3].x - input_marker.points[i*3+2].x;
      p0_1.y = input_marker.points[i*3].y - input_marker.points[i*3+2].y;
      p0_1.z = input_marker.points[i*3].z - input_marker.points[i*3+2].z;
      geometry_msgs::Point p1_2;
      p1_2.x = input_marker.points[i*3+1].x - input_marker.points[i*3+2].x;
      p1_2.y = input_marker.points[i*3+1].y - input_marker.points[i*3+2].y;
      p1_2.z = input_marker.points[i*3+1].z - input_marker.points[i*3+2].z;
      geometry_msgs::Point outer_product;
      outer_product.x = p0_1.y * p1_2.z - p0_1.z * p1_2.y;
      outer_product.y = p0_1.z * p1_2.x - p0_1.x * p1_2.z;
      outer_product.z = p0_1.x * p1_2.y - p0_1.y * p1_2.x;
      double tmp_triangle_area = abs(sqrt( pow(outer_product.x*1000, 2) + pow(outer_product.y*1000, 2) + pow(outer_product.z*1000, 2)))/2;
      total_area += tmp_triangle_area;
      cumulative_areas.push_back(total_area);
    }

    //Gether Random sampling points in propotion to area size
    for(int i = 0; i < sample_nums; i++){
      double r = rand() * (1.0  / (RAND_MAX + 1.0)) * total_area;
      std::vector<double>::iterator low = std::lower_bound (cumulative_areas.begin (), cumulative_areas.end (), r);
      int index = int(low - cumulative_areas.begin ());

      //Get Target Triangle
      PointT p;
      std_msgs::ColorRGBA color;
      geometry_msgs::Point p0 = input_marker.points[index*3];
      geometry_msgs::Point p1 = input_marker.points[index*3+1];
      geometry_msgs::Point p2 = input_marker.points[index*3+2];
      std_msgs::ColorRGBA c0= input_marker.colors[index*3];
      std_msgs::ColorRGBA c1 = input_marker.colors[index*3+1];
      std_msgs::ColorRGBA c2 = input_marker.colors[index*3+2];
      r = rand() * (1.0  / (RAND_MAX + 1.0));

      geometry_msgs::Point point_on_p1_p2;
      point_on_p1_p2.x = p1.x*r + p2.x*(1.0 - r);
      point_on_p1_p2.y = p1.y*r + p2.y*(1.0 - r);
      point_on_p1_p2.z = p1.z*r + p2.z*(1.0 - r);

      color.r = c1.r*r + c2.r*(1.0 - r);
      color.g = c1.g*r + c2.g*(1.0 - r);
      color.b = c1.b*r + c2.b*(1.0 - r);

      r = sqrt(rand() * (1.0  / (RAND_MAX + 1.0)));
      geometry_msgs::Point target;
      target.x = point_on_p1_p2.x*r + p0.x*(1.0 - r);
      target.y = point_on_p1_p2.y*r + p0.y*(1.0 - r);
      target.z = point_on_p1_p2.z*r + p0.z*(1.0 - r);
      color.r = color.r*r + c0.r*(1.0 - r);
      color.g = color.g*r + c0.g*(1.0 - r);
      color.b = color.b*r + c0.b*(1.0 - r);
      p.x = target.x;
      p.y = target.y;
      p.z = target.z;
      p.r = color.r * 256;
      p.g = color.g * 256;
      p.b = color.b * 256;
      
      output_cloud.points.push_back(p);
    }
    output_cloud.width = sample_nums;
    output_cloud.height = 1;
  }

  inline bool isValidPoint(const pcl::PointXYZ& p)
  {
    return !std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z);
  }

  template <class PointT>
  inline bool isValidPoint(const PointT& p)
  {
    return !std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z);
  }

}
// extend pcl_conversions package's toPCL and fromPCL functions
namespace pcl_conversions
{  
  std::vector<pcl::PointIndices::Ptr>
  convertToPCLPointIndices(const std::vector<PCLIndicesMsg>& cluster_indices);

  std::vector<pcl::ModelCoefficients::Ptr>
  convertToPCLModelCoefficients(
    const std::vector<PCLModelCoefficientMsg>& coefficients);
  
  std::vector<PCLIndicesMsg>
  convertToROSPointIndices(
    const std::vector<pcl::PointIndices::Ptr> cluster_indices,
    const std_msgs::Header& header);

  std::vector<PCLIndicesMsg>
  convertToROSPointIndices(
    const std::vector<pcl::PointIndices> cluster_indices,
    const std_msgs::Header& header);

  std::vector<PCLModelCoefficientMsg>
  convertToROSModelCoefficients(
    const std::vector<pcl::ModelCoefficients::Ptr>& coefficients,
    const std_msgs::Header& header);

}

namespace tf
{
  // for eigen float
  void poseMsgToEigen(const geometry_msgs::Pose& msg, Eigen::Affine3f& eigen);
  void poseEigenToMsg(Eigen::Affine3f& eigen, geometry_msgs::Pose& msg);
  void transformMsgToEigen(const geometry_msgs::Transform& msg, Eigen::Affine3f& eigen);
  void transformEigenToMsg(Eigen::Affine3f& eigen, geometry_msgs::Transform& msg);
  void transformTFToEigen(const tf::Transform& t, Eigen::Affine3f& eigen);
  void transformEigenToTF(Eigen::Affine3f& eigen , tf::Transform& t);
  void vectorTFToEigen(const tf::Vector3& t, Eigen::Vector3f& e);
  void vectorEigenToTF(const Eigen::Vector3f& e, tf::Vector3& t);
}

#endif
