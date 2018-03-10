/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
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

// #include <pcl/filters/impl/filter.hpp>
#include "fast_pcl/filters/impl/filter.hpp"
#include <pcl/PCLPointCloud2.h>

///////////////////////////////////////////////////////////////////////////////////////////
/** \brief Base method for feature estimation for all points given in <setInputCloud (), setIndices ()> using
 * the surface in setSearchSurface () and the spatial locator in setSearchMethod ()
 * \param output the resultant filtered point cloud dataset
 */
void
pcl::Filter<pcl::PCLPointCloud2>::filter (PCLPointCloud2 &output)
{
  if (!initCompute ())
    return;

  if (input_.get () == &output)  // cloud_in = cloud_out
  {
    pcl::PCLPointCloud2 output_temp;
    applyFilter (output_temp);
    output_temp.fields = input_->fields;
    output_temp.header = input_->header;
    pcl::copyPointCloud (output_temp, output);
  }
  else
  {
    output.fields = input_->fields;
    output.header = input_->header;
    applyFilter (output);
  }

  // Apply the actual filter
  applyFilter (output);

  deinitCompute ();
}

#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>

// Instantiations of specific point types
PCL_INSTANTIATE(removeNaNFromPointCloud, PCL_XYZ_POINT_TYPES)
PCL_INSTANTIATE(removeNaNNormalsFromPointCloud, PCL_NORMAL_POINT_TYPES)

#endif    // PCL_NO_PRECOMPILE

