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

#ifndef UTIL_FUNCTIONS_H
#define UTIL_FUNCTIONS_H

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include "data_structs.h"

Eigen::Matrix4f convertToEigenMatrix4f(const Pose& pose);
Pose convertToPose(const Eigen::Matrix4f& m);
Pose transformToPose(const Pose& pose, const Eigen::Matrix4f& m);

template <class PointType>
void addPointCloud(const boost::shared_ptr< pcl::PointCloud<PointType> >& input_ptr, const boost::shared_ptr< pcl::PointCloud<PointType> >& output_ptr)
{
    const auto need_points_size = output_ptr->points.size()+input_ptr->points.size();
    if(output_ptr->points.capacity() < need_points_size) {
        output_ptr->points.reserve(need_points_size*2);
    }
    *output_ptr += *input_ptr;
}

template <class PointType>
void addPointCloud(const boost::shared_ptr< pcl::PointCloud<PointType> const>& input_ptr, const boost::shared_ptr< pcl::PointCloud<PointType> >& output_ptr)
{
    const auto need_points_size = output_ptr->points.size()+input_ptr->points.size();
    if(output_ptr->points.capacity() < need_points_size) {
        output_ptr->points.reserve(need_points_size*2);
    }
    *output_ptr += *input_ptr;
}

template <class PointType>
void addPointCloud(const boost::shared_ptr< pcl::PointCloud<PointType> const>& input_ptr, const boost::shared_ptr< pcl::PointCloud<PointType> >& output_ptr, const Pose& pose)
{
    boost::shared_ptr< pcl::PointCloud<PointType> > transformed_input_ptr(new pcl::PointCloud<PointType>);
    const auto eigen_pose = convertToEigenMatrix4f(pose);
    pcl::transformPointCloud(*input_ptr, *transformed_input_ptr, eigen_pose);
    const auto need_points_size = output_ptr->points.size()+transformed_input_ptr->points.size();
    if(output_ptr->points.capacity() < need_points_size) {
        output_ptr->points.reserve(need_points_size*2);
    }
    *output_ptr += *transformed_input_ptr;
}

template <class PointType>
void passThroughPointCloud(const boost::shared_ptr< pcl::PointCloud<PointType> const> &input_point_cloud_ptr, const boost::shared_ptr< pcl::PointCloud<PointType> >& output_point_cloud_ptr, const double x, const double y, const double width)
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

template <class PointType>
void limitPointCloudRange(boost::shared_ptr< pcl::PointCloud<PointType> >& input_ptr, boost::shared_ptr< pcl::PointCloud<PointType> >& output_ptr, const double min_range_meter, const double max_range_meter)
{
    double r = 0;
    for (const auto& p : input_ptr->points) {
      r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
      if (r > min_range_meter && r < max_range_meter) {
        output_ptr->push_back(p);
      }
    }
}

template <class PointType>
void limitPointCloudRange(const boost::shared_ptr< pcl::PointCloud<PointType> const>& input_ptr, boost::shared_ptr< pcl::PointCloud<PointType> >& output_ptr, const double min_range_meter, const double max_range_meter)
{
    double r = 0;
    for (const auto& p : input_ptr->points) {
      r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
      if (r > min_range_meter && r < max_range_meter) {
        output_ptr->push_back(p);
      }
    }
}

//TODO use std::minmax_element
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


#endif
