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

#include "lidar_localizer/pose_linear_interpolator/pose_linear_interpolator.h"

PoseStamped interpolatePose(const PoseStamped& pose_a, const PoseStamped& pose_b, const double time_stamp)
{
    if(pose_a.stamp == 0 || pose_b.stamp == 0 || time_stamp == 0) {
        return PoseStamped();
    }

    Velocity v(pose_a, pose_b);
    const double dt = time_stamp - pose_b.stamp;

    PoseStamped p;
    p.pose.x = pose_b.pose.x + v.linear.x * dt;
    p.pose.y = pose_b.pose.y + v.linear.y * dt;
    p.pose.z = pose_b.pose.z;
    p.pose.roll = pose_b.pose.roll;
    p.pose.pitch = pose_b.pose.pitch;
    p.pose.yaw = pose_b.pose.yaw + v.angular.z * dt;
    p.stamp = time_stamp;
    return p;
}


PoseLinearInterpolator::PoseLinearInterpolator()
{
}

void PoseLinearInterpolator::clearPoseStamped()
{
    pose_.clear();
    prev_pose_.clear();
}

bool PoseLinearInterpolator::isNotSetPoseStamped() const
{
    return (pose_ == PoseStamped() && prev_pose_ == PoseStamped());
}
void PoseLinearInterpolator::pushbackPoseStamped(const PoseStamped& pose)
{
    prev_pose_ = pose_;
    pose_ = pose;
}

PoseStamped PoseLinearInterpolator::getInterpolatePose(const double time_stamp) const
{
    return interpolatePose(prev_pose_, pose_, time_stamp);
}
