/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 *
 * $Id$
 *
 */

#ifndef FAST_PCL_FILTERS_IMPL_FILTER_H_
#define FAST_PCL_FILTERS_IMPL_FILTER_H_

#include <pcl/pcl_macros.h>
//#include <pcl/filters/filter.h>
#include "fast_pcl/filters/filter.h"

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::removeNaNFromPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                              pcl::PointCloud<PointT> &cloud_out,
                              std::vector<int> &index)
{
  // If the clouds are not the same, prepare the output
  if (&cloud_in != &cloud_out)
  {
    cloud_out.header = cloud_in.header;
    cloud_out.points.resize (cloud_in.points.size ());
  }
  // Reserve enough space for the indices
  index.resize (cloud_in.points.size ());
  size_t j = 0;

  // If the data is dense, we don't need to check for NaN
  if (cloud_in.is_dense)
  {
    // Simply copy the data
    cloud_out = cloud_in;
    for (j = 0; j < cloud_out.points.size (); ++j)
      index[j] = static_cast<int>(j);
  }
  else
  {
    for (size_t i = 0; i < cloud_in.points.size (); ++i)
    {
      if (!pcl_isfinite (cloud_in.points[i].x) || 
          !pcl_isfinite (cloud_in.points[i].y) || 
          !pcl_isfinite (cloud_in.points[i].z))
        continue;
      cloud_out.points[j] = cloud_in.points[i];
      index[j] = static_cast<int>(i);
      j++;
    }
    if (j != cloud_in.points.size ())
    {
      // Resize to the correct size
      cloud_out.points.resize (j);
      index.resize (j);
    }

    cloud_out.height = 1;
    cloud_out.width  = static_cast<uint32_t>(j);

    // Removing bad points => dense (note: 'dense' doesn't mean 'organized')
    cloud_out.is_dense = true;
  }
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::removeNaNNormalsFromPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                                     pcl::PointCloud<PointT> &cloud_out,
                                     std::vector<int> &index)
{
  // If the clouds are not the same, prepare the output
  if (&cloud_in != &cloud_out)
  {
    cloud_out.header = cloud_in.header;
    cloud_out.points.resize (cloud_in.points.size ());
  }
  // Reserve enough space for the indices
  index.resize (cloud_in.points.size ());
  size_t j = 0;

  for (size_t i = 0; i < cloud_in.points.size (); ++i)
  {
    if (!pcl_isfinite (cloud_in.points[i].normal_x) || 
        !pcl_isfinite (cloud_in.points[i].normal_y) || 
        !pcl_isfinite (cloud_in.points[i].normal_z))
      continue;
    cloud_out.points[j] = cloud_in.points[i];
    index[j] = static_cast<int>(i);
    j++;
  }
  if (j != cloud_in.points.size ())
  {
    // Resize to the correct size
    cloud_out.points.resize (j);
    index.resize (j);
  }

  cloud_out.height = 1;
  cloud_out.width  = static_cast<uint32_t>(j);
}


#define PCL_INSTANTIATE_removeNaNFromPointCloud(T) template PCL_EXPORTS void pcl::removeNaNFromPointCloud<T>(const pcl::PointCloud<T>&, pcl::PointCloud<T>&, std::vector<int>&);
#define PCL_INSTANTIATE_removeNaNNormalsFromPointCloud(T) template PCL_EXPORTS void pcl::removeNaNNormalsFromPointCloud<T>(const pcl::PointCloud<T>&, pcl::PointCloud<T>&, std::vector<int>&);

#endif    // FAST_PCL_FILTERS_IMPL_FILTER_H_
