// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
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

#include "jsk_recognition_utils/pcl_ros_util.h"
#include <pcl_conversions/pcl_conversions.h>

namespace jsk_recognition_utils
{
  void publishPointIndices(
    ros::Publisher& pub, const pcl::PointIndices& indices, const std_msgs::Header& header)
  {
    pcl_msgs::PointIndices msg;
    //pcl_conversions::moveFromPCL does not support const indices
    msg.indices = indices.indices;
    msg.header = header;
    pub.publish(msg);
  }

  bool isSameFrameId(const std::string& a, const std::string& b)
  {
    // we can ignore the first /
    std::string aa;
    if (a.length() > 0 && a[0] == '/') {
      aa = a.substr(1, a.length() - 1);
    }
    else {
      aa = a;
    }
    std::string bb;
    if (b.length() > 0 && b[0] == '/') {
      bb = b.substr(1, b.length() - 1);
    }
    else {
      bb = b;
    }
    return aa == bb;
  }
  
  bool isSameFrameId(const std_msgs::Header& a, const std_msgs::Header& b)
  {
    return isSameFrameId(a.frame_id, b.frame_id);
  }
  
  bool hasField(const std::string& field_name, const sensor_msgs::PointCloud2& msg)
  {
    for (size_t i = 0; i < msg.fields.size(); i++) {
      sensor_msgs::PointField field = msg.fields[i];
      if (field.name == field_name) {
        return true;
      }
    }
    return false;
  }
  
}
