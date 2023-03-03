// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
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

#include "pointcloud_preprocessor/passthrough_filter/passthrough_uint16.hpp"

//////////////////////////////////////////////////////////////////////////
void pcl::PassThroughUInt16<pcl::PCLPointCloud2>::applyFilter(PCLPointCloud2 & output)
{
  if (!input_) {
    PCL_ERROR("[pcl::%s::applyFilter] Input dataset not given!\n", getClassName().c_str());
    output.width = output.height = 0;
    output.data.clear();
    return;
  }

  // If fields x/y/z are not present, we cannot filter
  if (x_idx_ == -1 || y_idx_ == -1 || z_idx_ == -1) {
    PCL_ERROR(
      "[pcl::%s::applyFilter] Input dataset doesn't have x-y-z coordinates!\n",
      getClassName().c_str());
    output.width = output.height = 0;
    output.data.clear();
    return;
  }

  int nr_points = input_->width * input_->height;

  // Check if we're going to keep the organized structure of the cloud or not
  if (keep_organized_) {
    if (filter_field_name_.empty()) {
      // Silly - if no filtering is actually done, and we want to keep the data organized,
      // just copy everything. Any optimizations that can be done here???
      output = *input_;
      return;
    }

    output.width = input_->width;
    output.height = input_->height;
    // Check what the user value is: if !finite, set is_dense to false, true otherwise
    if (!std::isfinite(user_filter_value_)) {
      output.is_dense = false;
    } else {
      output.is_dense = input_->is_dense;
    }
  } else {
    // Copy the header (and thus the frame_id) + allocate enough space for points
    output.height = 1;  // filtering breaks the organized structure
    // Because we're doing explicit checks for isfinite, is_dense = true
    output.is_dense = true;
  }
  output.row_step = input_->row_step;
  output.point_step = input_->point_step;
  output.is_bigendian = input_->is_bigendian;
  output.data.resize(input_->data.size());

  removed_indices_->resize(input_->data.size());

  int nr_p = 0;
  int nr_removed_p = 0;
  // Create the first xyz_offset
  Eigen::Array4i xyz_offset(
    input_->fields[x_idx_].offset, input_->fields[y_idx_].offset, input_->fields[z_idx_].offset, 0);

  Eigen::Vector4f pt = Eigen::Vector4f::Zero();
  // If we don't want to process the entire cloud,
  // but rather filter points far away from the viewpoint first...
  if (!filter_field_name_.empty()) {
    // Get the distance field index
    int distance_idx = pcl::getFieldIndex(*input_, filter_field_name_);
    if (distance_idx == -1) {
      PCL_WARN(
        "[pcl::%s::applyFilter] Invalid filter field name. Index is %d.\n", getClassName().c_str(),
        distance_idx);
      output.width = output.height = 0;
      output.data.clear();
      return;
    }

    // @todo fixme
    if (input_->fields[distance_idx].datatype != pcl::PCLPointField::UINT16) {
      PCL_ERROR(
        "[pcl::%s::downsample] Distance filtering requested, but distances are not uint16 in the "
        "dataset! Only UINT16 distances are supported right now.\n",
        getClassName().c_str());
      output.width = output.height = 0;
      output.data.clear();
      return;
    }

    // cspell: ignore badpt
    std::uint16_t badpt = user_filter_value_;
    // Check whether we need to store filtered valued in place
    if (keep_organized_) {
      std::uint16_t distance_value = 0;
      // Go over all points
      for (int cp = 0; cp < nr_points; ++cp, xyz_offset += input_->point_step) {
        // Copy all the fields
        memcpy(
          &output.data[cp * output.point_step], &input_->data[cp * output.point_step],
          output.point_step);

        // Get the distance value
        memcpy(
          &distance_value,
          &input_->data[cp * input_->point_step + input_->fields[distance_idx].offset],
          sizeof(std::uint16_t));

        if (filter_limit_negative_) {
          // Use a threshold for cutting out points which inside the interval
          if ((distance_value < filter_limit_max_) && (distance_value > filter_limit_min_)) {
            // Unoptimized memcpy functions: assume fields x, y, z are in random order
            memcpy(&output.data[xyz_offset[0]], &badpt, sizeof(std::uint16_t));
            memcpy(&output.data[xyz_offset[1]], &badpt, sizeof(std::uint16_t));
            memcpy(&output.data[xyz_offset[2]], &badpt, sizeof(std::uint16_t));
            continue;
          } else {
            if (extract_removed_indices_) {
              (*removed_indices_)[nr_removed_p++] = cp;
            }
          }
        } else {
          // Use a threshold for cutting out points which are too close/far away
          if ((distance_value > filter_limit_max_) || (distance_value < filter_limit_min_)) {
            // Unoptimized memcpy functions: assume fields x, y, z are in random order
            memcpy(&output.data[xyz_offset[0]], &badpt, sizeof(std::uint16_t));
            memcpy(&output.data[xyz_offset[1]], &badpt, sizeof(std::uint16_t));
            memcpy(&output.data[xyz_offset[2]], &badpt, sizeof(std::uint16_t));
            continue;
          } else {
            if (extract_removed_indices_) {
              (*removed_indices_)[nr_removed_p++] = cp;
            }
          }
        }
      }
    } else {  // Remove filtered points
      // Go over all points
      std::uint16_t distance_value = 0;
      for (int cp = 0; cp < nr_points; ++cp, xyz_offset += input_->point_step) {
        // Get the distance value
        memcpy(
          &distance_value,
          &input_->data[cp * input_->point_step + input_->fields[distance_idx].offset],
          sizeof(std::uint16_t));

        // Remove NAN/INF/-INF values. We expect passthrough to output clean valid data.
        if (!std::isfinite(distance_value)) {
          if (extract_removed_indices_) {
            (*removed_indices_)[nr_removed_p++] = cp;
          }
          continue;
        }

        if (filter_limit_negative_) {
          // Use a threshold for cutting out points which inside the interval
          if (distance_value < filter_limit_max_ && distance_value > filter_limit_min_) {
            if (extract_removed_indices_) {
              (*removed_indices_)[nr_removed_p] = cp;
              nr_removed_p++;
            }
            continue;
          }
        } else {
          // Use a threshold for cutting out points which are too close/far away
          if (distance_value > filter_limit_max_ || distance_value < filter_limit_min_) {
            if (extract_removed_indices_) {
              (*removed_indices_)[nr_removed_p] = cp;
              nr_removed_p++;
            }
            continue;
          }
        }

        // Unoptimized memcpy functions: assume fields x, y, z are in random order
        memcpy(&pt[0], &input_->data[xyz_offset[0]], sizeof(std::uint16_t));
        memcpy(&pt[1], &input_->data[xyz_offset[1]], sizeof(std::uint16_t));
        memcpy(&pt[2], &input_->data[xyz_offset[2]], sizeof(std::uint16_t));

        // Check if the point is invalid
        if (!std::isfinite(pt[0]) || !std::isfinite(pt[1]) || !std::isfinite(pt[2])) {
          if (extract_removed_indices_) {
            (*removed_indices_)[nr_removed_p] = cp;
            nr_removed_p++;
          }
          continue;
        }

        // Copy all the fields
        memcpy(
          &output.data[nr_p * output.point_step], &input_->data[cp * output.point_step],
          output.point_step);
        nr_p++;
      }
      output.width = nr_p;
    }       // !keep_organized_
  } else {  // No distance filtering, process all data.
            // No need to check for is_organized here as we did it above
    for (int cp = 0; cp < nr_points; ++cp, xyz_offset += input_->point_step) {
      // Unoptimized memcpy functions: assume fields x, y, z are in random order
      memcpy(&pt[0], &input_->data[xyz_offset[0]], sizeof(std::uint16_t));
      memcpy(&pt[1], &input_->data[xyz_offset[1]], sizeof(std::uint16_t));
      memcpy(&pt[2], &input_->data[xyz_offset[2]], sizeof(std::uint16_t));

      // Check if the point is invalid
      if (!std::isfinite(pt[0]) || !std::isfinite(pt[1]) || !std::isfinite(pt[2])) {
        if (extract_removed_indices_) {
          (*removed_indices_)[nr_removed_p] = cp;
          nr_removed_p++;
        }
        continue;
      }

      // Copy all the fields
      memcpy(
        &output.data[nr_p * output.point_step], &input_->data[cp * output.point_step],
        output.point_step);
      nr_p++;
    }
    output.width = nr_p;
  }

  output.row_step = output.point_step * output.width;
  output.data.resize(output.width * output.height * output.point_step);

  removed_indices_->resize(nr_removed_p);
}

#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>

#include <pcl/point_types.h>

// Instantiations of specific point types
PCL_INSTANTIATE(PassThroughUInt16, PCL_XYZ_POINT_TYPES)

#endif  // PCL_NO_PRECOMPILE
