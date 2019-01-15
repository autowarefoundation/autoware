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

#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <pcl/visualization/common/float_image_utils.h>

namespace jsk_recognition_utils
{

  void rangeImageToCvMat(const pcl::RangeImage& range_image,
                         cv::Mat& image)
  {
    float min_range, max_range;
    range_image.getMinMaxRanges(min_range, max_range);
    float min_max_range = max_range - min_range;

    image = cv::Mat(range_image.height, range_image.width, CV_8UC3);
    unsigned char r,g,b;
    for (int y=0; y < range_image.height; y++) {
      for (int x=0; x<range_image.width; x++) {
        pcl::PointWithRange rangePt = range_image.getPoint(x,y);
        if (!pcl_isfinite(rangePt.range)) {
          pcl::visualization::FloatImageUtils::getColorForFloat(
            rangePt.range, r, g, b);
        }
        else {
          float value = (rangePt.range - min_range) / min_max_range;
          pcl::visualization::FloatImageUtils::getColorForFloat(
            value, r, g, b);
        }
        image.at<cv::Vec3b>(y,x)[0] = b;
        image.at<cv::Vec3b>(y,x)[1] = g;
        image.at<cv::Vec3b>(y,x)[2] = r;
      }
    }
    return;
  }
  
  void convertEigenAffine3(const Eigen::Affine3d& from,
                           Eigen::Affine3f& to)
  {
    Eigen::Matrix4d from_mat = from.matrix();
    Eigen::Matrix4f to_mat;
    convertMatrix4<Eigen::Matrix4d, Eigen::Matrix4f>(from_mat, to_mat);
    to = Eigen::Affine3f(to_mat);
  }
  
  void convertEigenAffine3(const Eigen::Affine3f& from,
                           Eigen::Affine3d& to)
  {
    Eigen::Matrix4f from_mat = from.matrix();
    Eigen::Matrix4d to_mat;
    convertMatrix4<Eigen::Matrix4f, Eigen::Matrix4d>(from_mat, to_mat);
    to = Eigen::Affine3d(to_mat);
  }
}

namespace pcl_conversions
{
  std::vector<pcl::PointIndices::Ptr>
  convertToPCLPointIndices(
    const std::vector<PCLIndicesMsg>& cluster_indices)
  {
    std::vector<pcl::PointIndices::Ptr> ret;
    for (size_t i = 0; i < cluster_indices.size(); i++) {
      std::vector<int> indices = cluster_indices[i].indices;
      pcl::PointIndices::Ptr pcl_indices (new pcl::PointIndices);
      pcl_indices->indices = indices;
      ret.push_back(pcl_indices);
    }
    return ret;
  }

  std::vector<pcl::ModelCoefficients::Ptr>
  convertToPCLModelCoefficients(
    const std::vector<PCLModelCoefficientMsg>& coefficients)
  {
    std::vector<pcl::ModelCoefficients::Ptr> ret;
    for (size_t i = 0; i < coefficients.size(); i++) {
      pcl::ModelCoefficients::Ptr pcl_coefficients (new pcl::ModelCoefficients);
      pcl_coefficients->values = coefficients[i].values;
      ret.push_back(pcl_coefficients);
    }
    return ret;
  }

  std::vector<PCLIndicesMsg>
  convertToROSPointIndices(
    const std::vector<pcl::PointIndices::Ptr> cluster_indices,
    const std_msgs::Header& header)
  {
    std::vector<PCLIndicesMsg> ret;
    for (size_t i = 0; i < cluster_indices.size(); i++) {
      PCLIndicesMsg ros_msg;
      ros_msg.header = header;
      ros_msg.indices = cluster_indices[i]->indices;
      ret.push_back(ros_msg);
    }
    return ret;
  }

  std::vector<PCLIndicesMsg>
  convertToROSPointIndices(
    const std::vector<pcl::PointIndices> cluster_indices,
    const std_msgs::Header& header)
  {
    std::vector<PCLIndicesMsg> ret;
    for (size_t i = 0; i < cluster_indices.size(); i++) {
      PCLIndicesMsg ros_msg;
      ros_msg.header = header;
      ros_msg.indices = cluster_indices[i].indices;
      ret.push_back(ros_msg);
    }
    return ret;
  }

  std::vector<PCLModelCoefficientMsg>
  convertToROSModelCoefficients(
    const std::vector<pcl::ModelCoefficients::Ptr>& coefficients,
    const std_msgs::Header& header)
  {
    std::vector<PCLModelCoefficientMsg> ret;
    for (size_t i = 0; i < coefficients.size(); i++) {
      PCLModelCoefficientMsg ros_msg;
      ros_msg.header = header;
      ros_msg.values = coefficients[i]->values;
      ret.push_back(ros_msg);
    }
    return ret;
  }
  
}


namespace tf
{
  void poseMsgToEigen(const geometry_msgs::Pose& msg, Eigen::Affine3f& eigen)
  {
    Eigen::Affine3d eigen_d;
    poseMsgToEigen(msg, eigen_d);
    jsk_recognition_utils::convertEigenAffine3(eigen_d, eigen);
  }
  
  void poseEigenToMsg(Eigen::Affine3f& eigen, geometry_msgs::Pose& msg)
  {
    Eigen::Affine3d eigen_d;
    jsk_recognition_utils::convertEigenAffine3(eigen, eigen_d);
    poseEigenToMsg(eigen_d, msg);
  }

  void transformMsgToEigen(const geometry_msgs::Transform& msg, Eigen::Affine3f& eigen)
  {
    Eigen::Affine3d eigen_d;
    transformMsgToEigen(msg, eigen_d);
    jsk_recognition_utils::convertEigenAffine3(eigen_d, eigen);
  }
  
  void transformEigenToMsg(Eigen::Affine3f& eigen, geometry_msgs::Transform& msg)
  {
    Eigen::Affine3d eigen_d;
    jsk_recognition_utils::convertEigenAffine3(eigen_d, eigen);
    transformEigenToMsg(eigen_d, msg);
  }

  void transformTFToEigen(const tf::Transform& t, Eigen::Affine3f& eigen)
  {
    Eigen::Affine3d eigen_d;
    transformTFToEigen(t, eigen_d);
    jsk_recognition_utils::convertEigenAffine3(eigen_d, eigen);
  }

  void transformEigenToTF(Eigen::Affine3f& eigen , tf::Transform& t)
  {
    Eigen::Affine3d eigen_d;
    jsk_recognition_utils::convertEigenAffine3(eigen, eigen_d);
    transformEigenToTF(eigen_d, t);
  }

  void vectorTFToEigen(const tf::Vector3& t, Eigen::Vector3f& e)
  {
    Eigen::Vector3d d;
    tf::vectorTFToEigen(t, d);
    e[0] = d[0];
    e[1] = d[1];
    e[2] = d[2];
  }
  
  void vectorEigenToTF(const Eigen::Vector3f& e, tf::Vector3& t)
  {
    Eigen::Vector3d d(e[0], e[1], e[2]);
    tf::vectorEigenToTF(d, t);
  }
}
