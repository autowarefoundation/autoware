/*
 *  Copyright (c) 2015, Nagoya University
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

#include <ros/ros.h>
#include <autoware_msgs/CurvatureCommandStamped.h>

class KalmanFilter
{
private:
  double x_, p_, k_, Q_, R_;
public:
  KalmanFilter(double Q = 1e-2, double R = 3e-1)
  : x_(1e-0), p_(1e-2), k_(1e-0)
  { Q_ = Q; R_ = R; }
  void init(double x0) { x_ = x0; }
  void predict() {
    x_ = x_;
    p_ = p_ + Q_;
  }
  double update(const double z)
  {
    k_ = p_ / (p_ + R_);
    x_ = x_ + k_ * (z - x_);
    p_ = (1.0 - k_) * p_;
    return x_;
  }
};

class CurvatureCmdFilter
{
private:
  ros::NodeHandle nh_, private_nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  KalmanFilter kf_;

public:
  CurvatureCmdFilter() : nh_(),  private_nh_("~"), kf_()
  {
    sub_ = nh_.subscribe("curvature_cmd_raw", 1, &CurvatureCmdFilter::curvatureCmdCallback, this);
    pub_ = nh_.advertise<autoware_msgs::CurvatureCommandStamped>("curvature_cmd", 10);
  }

  void curvatureCmdCallback(const autoware_msgs::CurvatureCommandStamped::ConstPtr &msg)
  {
    autoware_msgs::CurvatureCommandStamped cmd;
    cmd.header = msg->header;
    kf_.predict();
    cmd.cmd.linear_velocity = kf_.update(msg->cmd.linear_velocity);
    cmd.cmd.curvature = msg->cmd.curvature;
    pub_.publish(cmd);
  }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "curvature_cmd_filter");
    CurvatureCmdFilter curvature_cmd_filter;
    ros::spin();
    return 0;
}
