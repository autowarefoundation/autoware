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

#include "jsk_recognition_utils/grid_line.h"

namespace jsk_recognition_utils
{
  GridLine::GridLine(const pcl::PointXYZRGB a, const pcl::PointXYZRGB b)
    : from(a.getVector3fMap()), to(b.getVector3fMap()), d_(from - to)
  {
    d_.normalized();
  }
  
  GridLine::~GridLine()
  {

  }

  // bool GridLine::penetrateGrid(const Eigen::Vector3f A,
  //                              const Eigen::Vector3f B,
  //                              const Eigen::Vector3f C,
  //                              const Eigen::Vector3f D)
  // {
  //   Eigen::Vector3f Across = (A - from).cross(d_);
  //   Eigen::Vector3f Bcross = (B - from).cross(d_);
  //   Eigen::Vector3f Ccross = (C - from).cross(d_);
  //   Eigen::Vector3f Dcross = (D - from).cross(d_);
  //   bool ab_direction = Across.dot(Bcross) < 0;
  //   bool ac_direction = Across.dot(Ccross) < 0;
  //   bool ad_direction = Across.dot(Dcross) < 0;
  //   if ((ab_direction == ac_direction) && (ab_direction == ad_direction)) {
  //     return false;
  //   }
  //   else {
  //     return true;
  //   }
  // }
  
  bool GridLine::penetrateGrid(const pcl::PointXYZRGB& A,
                               const pcl::PointXYZRGB& B,
                               const pcl::PointXYZRGB& C,
                               const pcl::PointXYZRGB& D)
  {
    return penetrateGrid(A.getVector3fMap(),
                         B.getVector3fMap(),
                         C.getVector3fMap(),
                         D.getVector3fMap());
  }
}
