/*
// *  Copyright (c) 2018, TierIV, Inc.
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

// ROS includes
#include <ros/ros.h>

// C++ includes
#include <iostream>
#include <fstream>
#include <vector>
#include <tf/transform_datatypes.h>

#include "autoware_msgs/LaneArray.h"

namespace waypoint_maker
{
// Constructor
class WaypointWriter
{
private:
  ros::NodeHandle nh_, private_nh_;
  ros::Subscriber larray_sub_;
  std::string lane_csv_;
public:
  WaypointWriter() : private_nh_("~")
  {
    initPubSub();
  }
  // Destructor
  ~WaypointWriter()
  {
  }

  double mps2kmph(double velocity_mps)
  {
    return (velocity_mps * 60 * 60) / 1000;
  }
  const std::string addFileSuffix(std::string file_path, std::string suffix)
  {
    std::string output_file_path, tmp;
    std::string directory_path, filename, extension;

    tmp = file_path;
    const std::string::size_type idx_slash = tmp.find_last_of("/");
    if (idx_slash != std::string::npos)
    {
      tmp.erase(0, idx_slash);
    }
    const std::string::size_type idx_dot = tmp.find_last_of(".");
    const std::string::size_type idx_dot_allpath = file_path.find_last_of(".");
    if (idx_dot != std::string::npos && idx_dot != tmp.size() - 1)
    {
      file_path.erase(idx_dot_allpath, file_path.size() - 1);
    }
    file_path += suffix + ".csv";
    return file_path;
  }
  void initPubSub()
  {
    private_nh_.param<std::string>("lane_csv", lane_csv_, "/tmp/driving_lane.csv");
    // setup publisher
    larray_sub_ = nh_.subscribe("/lane_waypoints_array", 1, &WaypointWriter::LArrayCallback, this);
  }
  void LArrayCallback(const autoware_msgs::LaneArray::ConstPtr& larray)
  {
    if (larray->lanes.empty())
    {
      return;
    }
    std::vector<std::string> dst_multi_file_path(larray->lanes.size(), lane_csv_);
    for (auto& el : dst_multi_file_path)
    {
      el = addFileSuffix(el, std::to_string(&el - &dst_multi_file_path[0]));
    }
    saveLaneArray(dst_multi_file_path, *larray);
  }

  void saveLaneArray(const std::vector<std::string>& paths,
                                         const autoware_msgs::LaneArray& lane_array)
  {
    for (const auto& file_path : paths)
    {
      const unsigned long idx = &file_path - &paths[0];
      std::ofstream ofs(file_path.c_str());
      ofs << "x,y,z,yaw,velocity,change_flag,steering_flag,accel_flag,stop_flag,event_flag" << std::endl;
      for (const auto& el : lane_array.lanes[idx].waypoints)
      {
        ofs << std::fixed << std::setprecision(4) << el.pose.pose.position.x << "," << el.pose.pose.position.y << ","
            << el.pose.pose.position.z << "," << tf::getYaw(el.pose.pose.orientation) << ","
            << mps2kmph(el.twist.twist.linear.x) << "," << (int)el.change_flag << "," << (int)el.wpstate.steering_state
            << "," << (int)el.wpstate.accel_state << "," << (int)el.wpstate.stopline_state/*stop_state*/ << ","
            << (int)el.wpstate.event_state << std::endl;
      }
    }
  }
};

}  // waypoint_maker

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_writer");
  waypoint_maker::WaypointWriter ww;
  ros::spin();
  return 0;
}
