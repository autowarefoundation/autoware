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

#include "util_functions.h"

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
