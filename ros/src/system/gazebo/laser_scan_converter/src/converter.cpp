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
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;

class LaserScanConverter {
private:
  boost::mutex m_mutex_;
  // ROS
  ros::NodeHandle nh_;
  ros::Subscriber sub_points_;
  ros::Publisher pub_points_;

  int queue_size_;
  string laser_in_, laser_out_;

public:
  LaserScanConverter()
      : nh_("~"), queue_size_(100), laser_in_("/points_in"),
        laser_out_("/points_out") {
    // Subscribe to the cloud topic using both the old message format and the
    // new
    sub_points_ = nh_.subscribe(laser_in_, queue_size_,
                                &LaserScanConverter::cloud_cb_points, this);

    pub_points_ =
        nh_.advertise<sensor_msgs::LaserScan>(laser_out_, queue_size_);
    nh_.resolveName(laser_in_).c_str();
    nh_.resolveName(laser_out_).c_str();
  }

  void cloud_cb_points(const sensor_msgs::LaserScanConstPtr &msg) {
    if (pub_points_.getNumSubscribers() <= 0) {
      // ROS_DEBUG ("[point_cloud_converter] Got a PointCloud with %d points on
      // %s, but no subscribers.", (int)msg->points.size (), nh_.resolveName
      // (points_in_).c_str ());
      return;
    }

    sensor_msgs::LaserScan output;
    output = *msg;
    output.ranges.clear();
    output.intensities.clear();
    for (int i; i < (int)msg->ranges.size(); ++i) {
      if (msg->ranges.at(i) > 40.0) {
        output.ranges.push_back(0);
        output.intensities.push_back(0);
        continue;
      }
      output.ranges.push_back(msg->ranges.at(i));
      output.intensities.push_back(msg->intensities.at(i));
    }
    pub_points_.publish(output);
  }
};

/* ---[ */
int main(int argc, char **argv) {
  // ROS init
  ros::init(argc, argv, "laser_scan_converter",
            ros::init_options::AnonymousName);

  LaserScanConverter p;
  ros::spin();

  return (0);
}
