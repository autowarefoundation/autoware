/*
 *  Copyright (c) 2017, Tier IV, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "lidar_localizer/util/data_structs.h"

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

Eigen::Matrix4f convertToEigenMatrix4f(const Pose& pose)
{
    const Eigen::Translation3f translation(pose.x, pose.y, pose.z);
    const Eigen::AngleAxisf rotation_x(pose.roll, Eigen::Vector3f::UnitX());
    const Eigen::AngleAxisf rotation_y(pose.pitch, Eigen::Vector3f::UnitY());
    const Eigen::AngleAxisf rotation_z(pose.yaw, Eigen::Vector3f::UnitZ());
    const Eigen::Matrix4f m = (translation * rotation_z * rotation_y * rotation_x).matrix();
    return m;
}

Pose convertToPose(const Eigen::Matrix4f& m)
{
  Pose pose;
  pose.x = m(0, 3);
  pose.y = m(1, 3);
  pose.z = m(2, 3);

  //reference to tf::getEulerYPR()
  if (std::fabs(m(2,0)) >= 1)
  {
    pose.yaw = 0;
    if (m(2,0) < 0)
    {
      pose.pitch = M_PI / 2.0;
      pose.roll = std::atan2(m(0,1),m(0,2));
    }
    else
    {
      pose.pitch = -M_PI / 2.0;
      pose.roll = std::atan2(-m(0,1),-m(0,2));
    }
  }
  else
  {
    pose.pitch = -std::asin(m(2,0));
    pose.roll  = std::atan2(m(2,1)/std::cos(pose.pitch),
                            m(2,2)/std::cos(pose.pitch));
    pose.yaw   = std::atan2(m(1,0)/std::cos(pose.pitch),
                            m(0,0)/std::cos(pose.pitch));
  }

  return pose;
}

Pose transformToPose(const Pose& pose, const Eigen::Matrix4f& m)
{
  Eigen::Matrix4f eigen_pose = convertToEigenMatrix4f(pose);
  Eigen::Matrix4f trans_pose = eigen_pose * m;

  return convertToPose(trans_pose);
}

Pose convertPoseIntoRelativeCoordinate(const Pose& target_pose, const Pose& reference_pose)
{
    Eigen::Matrix4f target_eigen = convertToEigenMatrix4f(target_pose);
    Eigen::Matrix4f reference_eigen = convertToEigenMatrix4f(reference_pose);

    Eigen::Matrix4f trans_eigen = reference_eigen.inverse() * target_eigen;

    return convertToPose(trans_eigen);
}

template <class PointType>
void addPointCloud(const boost::shared_ptr< pcl::PointCloud<PointType> >& input_ptr, const boost::shared_ptr< pcl::PointCloud<PointType> >& output_ptr)
{
    const auto need_points_size = output_ptr->points.size()+input_ptr->points.size();
    output_ptr->width = need_points_size;
    output_ptr->height = 1;
    std::cout << __func__ << " " << output_ptr->points.size() << std::endl;
    if(output_ptr->points.capacity() < need_points_size) {
        const auto reverse_size = need_points_size*2;
        output_ptr->points.reserve(reverse_size);
        std::cout << __func__ << " " << output_ptr->points.capacity() << std::endl;
    }
    *output_ptr += *input_ptr;
}
template void addPointCloud(const boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> >& input_ptr, const boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> >& output_ptr);
template void addPointCloud(const boost::shared_ptr< pcl::PointCloud<pcl::PointXYZI> >& input_ptr, const boost::shared_ptr< pcl::PointCloud<pcl::PointXYZI> >& output_ptr);

template <class PointType>
void passThroughPointCloud(const boost::shared_ptr< pcl::PointCloud<PointType> > &input_point_cloud_ptr, const boost::shared_ptr< pcl::PointCloud<PointType> >& output_point_cloud_ptr, const double x, const double y, const double width)
{
    output_point_cloud_ptr->points.reserve(output_point_cloud_ptr->width);
    for(const auto& point : input_point_cloud_ptr->points)
    {
        if(  point.x >= x && point.x <= x+width
          && point.y >= y && point.y <= y+width) {
              output_point_cloud_ptr->points.push_back(point);
        }
    }
    output_point_cloud_ptr->width = output_point_cloud_ptr->points.size();
    output_point_cloud_ptr->height = 1;
}
template void passThroughPointCloud(const boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> > &input_point_cloud_ptr, const boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> >& output_point_cloud_ptr, const double x, const double y, const double width);
template void passThroughPointCloud(const boost::shared_ptr< pcl::PointCloud<pcl::PointXYZI> > &input_point_cloud_ptr, const boost::shared_ptr< pcl::PointCloud<pcl::PointXYZI> >& output_point_cloud_ptr, const double x, const double y, const double width);


template <class PointType>
void limitPointCloudRange(const boost::shared_ptr< pcl::PointCloud<PointType> >& input_ptr, const boost::shared_ptr< pcl::PointCloud<PointType> >& output_ptr, const double min_range_meter, const double max_range_meter)
{
    double r = 0;
    for (const auto& p : input_ptr->points) {
      r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
      if (r > min_range_meter && r < max_range_meter) {
        output_ptr->push_back(p);
      }
    }
}
template void limitPointCloudRange(const boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> >& input_ptr, const boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> >& output_ptr, const double min_range_meter, const double max_range_meter);
template void limitPointCloudRange(const boost::shared_ptr< pcl::PointCloud<pcl::PointXYZI> >& input_ptr, const boost::shared_ptr< pcl::PointCloud<pcl::PointXYZI> >& output_ptr, const double min_range_meter, const double max_range_meter);

template <class PointType>
double calcMaxX(const boost::shared_ptr< pcl::PointCloud<PointType> > &cloud)
{
    if (cloud->empty())
        return 0;
    double height = std::max_element(cloud->begin(), cloud->end(), [](const PointType & lhs, const PointType & rhs)
    {
        return lhs.x < rhs.x;
    })->x;
    return height;
}
template double calcMaxX(const boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> > &cloud);
template double calcMaxX(const boost::shared_ptr< pcl::PointCloud<pcl::PointXYZI> > &cloud);

template <class PointType>
double calcMinX(const boost::shared_ptr< pcl::PointCloud<PointType> > &cloud)
{
    if (cloud->empty())
        return 0;
    double height = std::min_element(cloud->begin(), cloud->end(), [](const PointType & lhs, const PointType & rhs)
    {
        return lhs.x < rhs.x;
    })->x;
    return height;
}
template double calcMinX(const boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> > &cloud);
template double calcMinX(const boost::shared_ptr< pcl::PointCloud<pcl::PointXYZI> > &cloud);

template <class PointType>
double calcMaxY(const boost::shared_ptr< pcl::PointCloud<PointType> > &cloud)
{
    if (cloud->empty())
        return 0;
    double height = std::max_element(cloud->begin(), cloud->end(), [](const PointType & lhs, const PointType & rhs)
    {
        return lhs.y < rhs.y;
    })->y;
    return height;
}
template double calcMaxY(const boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> > &cloud);
template double calcMaxY(const boost::shared_ptr< pcl::PointCloud<pcl::PointXYZI> > &cloud);

template <class PointType>
double calcMinY(const boost::shared_ptr< pcl::PointCloud<PointType> > &cloud)
{
    if (cloud->empty())
        return 0;
    double height = std::min_element(cloud->begin(), cloud->end(), [](const PointType & lhs, const PointType & rhs)
    {
        return lhs.y < rhs.y;
    })->y;
    return height;
}
template double calcMinY(const boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> > &cloud);
template double calcMinY(const boost::shared_ptr< pcl::PointCloud<pcl::PointXYZI> > &cloud);
