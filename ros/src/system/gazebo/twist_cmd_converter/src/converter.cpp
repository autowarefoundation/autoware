/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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
