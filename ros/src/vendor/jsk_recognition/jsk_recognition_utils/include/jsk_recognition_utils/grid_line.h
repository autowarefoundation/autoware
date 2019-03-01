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
#ifndef JSK_RECOGNITION_UTILS_GRID_LINE_H_
#define JSK_RECOGNITION_UTILS_GRID_LINE_H_
#include <pcl/point_types.h>
#include <boost/shared_ptr.hpp>
#include "jsk_recognition_utils/grid_index.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace jsk_recognition_utils
{
  class GridLine
  {
  public:
    typedef boost::shared_ptr<GridLine> Ptr;
    GridLine(const pcl::PointXYZRGB a, const pcl::PointXYZRGB b);
    virtual ~GridLine();
    virtual bool penetrateGrid(const pcl::PointXYZRGB& A,
                               const pcl::PointXYZRGB& B,
                               const pcl::PointXYZRGB& C,
                               const pcl::PointXYZRGB& D);
    virtual bool penetrateGrid(const Eigen::Vector3f A,
                               const Eigen::Vector3f B,
                               const Eigen::Vector3f C,
                               const Eigen::Vector3f D)
    {
      // std::cout << "checking "
      //           << "[" << from[0] << ", " << from[1] << ", " << from[2] << "]"
      //           << "--"
      //           << "[" << to[0] << ", " << to[1] << ", " << to[2] << "]"
      //           << std::endl;
      // std::cout << " penetrates "
      //           << "[" << A[0] << ", " << A[1] << ", " << A[2] << "], "
      //           << "[" << B[0] << ", " << B[1] << ", " << B[2] << "], "
      //           << "[" << C[0] << ", " << C[1] << ", " << C[2] << "], "
      //           << "[" << D[0] << ", " << D[1] << ", " << D[2] << "]" << std::endl;
      Eigen::Vector3f Across = (A - from).cross(d_);
      Eigen::Vector3f Bcross = (B - from).cross(d_);
      Eigen::Vector3f Ccross = (C - from).cross(d_);
      Eigen::Vector3f Dcross = (D - from).cross(d_);
      bool ab_direction = Across.dot(Bcross) < 0;
      bool ac_direction = Across.dot(Ccross) < 0;
      bool ad_direction = Across.dot(Dcross) < 0;
      bool bc_direction = Bcross.dot(Ccross) < 0;
      if (Across.norm() == 0 || 
          Bcross.norm() == 0 || 
          Ccross.norm() == 0 || 
          Dcross.norm() == 0) {
        return true;
      }
      else if ((ab_direction == ac_direction) && 
               (ab_direction == ad_direction)
               && (ab_direction == bc_direction)) {
        return false;
      }
      else {
        return true;
      }
    }
    
    const Eigen::Vector3f from;
    const Eigen::Vector3f to;
  protected:
    const Eigen::Vector3f d_;
  };
}

#endif
