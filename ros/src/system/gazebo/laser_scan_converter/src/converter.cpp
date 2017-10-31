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
