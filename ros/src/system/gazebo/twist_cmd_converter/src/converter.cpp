/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 * this
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
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <boost/thread.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>

using namespace std;

class TwistCmdConverter {
private:
  boost::mutex m_mutex_;
  // ROS
  ros::NodeHandle nh_;
  ros::Subscriber sub_twist_;
  ros::Publisher pub_twist_;

  int queue_size_;
  string twist_in_, twist_out_;

public:
  TwistCmdConverter()
      : nh_("~"), queue_size_(100), twist_in_("/twist_in"),
        twist_out_("/twist_out") {
    // Subscribe to the cloud topic using both the old message format and the
    // new
    sub_twist_ = nh_.subscribe(twist_in_, queue_size_,
                               &TwistCmdConverter::cloud_cb_twist, this);

    pub_twist_ = nh_.advertise<geometry_msgs::Twist>(twist_out_, queue_size_);
    nh_.resolveName(twist_in_).c_str();
    nh_.resolveName(twist_out_).c_str();
  }

  void cloud_cb_twist(const geometry_msgs::TwistStampedConstPtr &msg) {
    if (pub_twist_.getNumSubscribers() <= 0) {
      // ROS_DEBUG ("[point_cloud_converter] Got a PointCloud with %d points on
      // %s, but no subscribers.", (int)msg->points.size (), nh_.resolveName
      // (points_in_).c_str ());
      return;
    }

    geometry_msgs::Twist output;
    output = msg->twist;
    pub_twist_.publish(output);
  }
};

/* ---[ */
int main(int argc, char **argv) {
  // ROS init
  ros::init(argc, argv, "twist_cmd_converter",
            ros::init_options::AnonymousName);

  TwistCmdConverter p;
  ros::spin();

  return (0);
}
