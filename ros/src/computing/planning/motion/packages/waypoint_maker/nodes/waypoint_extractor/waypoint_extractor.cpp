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
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <autoware_msgs/LaneArray.h>
#include <iostream>
#include <fstream>
#include <vector>

namespace waypoint_maker
{
// Constructor
class WaypointExtractor
{
private:
  ros::NodeHandle nh_, private_nh_;
  ros::Subscriber larray_sub_;
  std::string lane_csv_, lane_topic_;
  autoware_msgs::LaneArray lane_;
public:
  WaypointExtractor() : private_nh_("~")
  {
    init();
  }
  // Destructor
  ~WaypointExtractor()
  {
    deinit();
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
    const std::string::size_type idx_slash(tmp.find_last_of("/"));
    if (idx_slash != std::string::npos)
    {
      tmp.erase(0, idx_slash);
    }
    const std::string::size_type idx_dot(tmp.find_last_of("."));
    const std::string::size_type idx_dot_allpath(file_path.find_last_of("."));
    if (idx_dot != std::string::npos && idx_dot != tmp.size() - 1)
    {
      file_path.erase(idx_dot_allpath, file_path.size() - 1);
    }
    file_path += suffix + ".csv";
    return file_path;
  }

  void init()
  {
    private_nh_.param<std::string>("lane_csv", lane_csv_, "/tmp/driving_lane.csv");
    private_nh_.param<std::string>("lane_topic", lane_topic_, "/lane_waypoints_array");
    // setup publisher
    larray_sub_ = nh_.subscribe(lane_topic_, 1, &WaypointExtractor::LaneArrayCallback, this);
  }

  void deinit()
  {
    if (lane_.lanes.empty())
    {
      return;
    }
    std::vector<std::string> dst_multi_file_path(lane_.lanes.size(), lane_csv_);
    if (lane_.lanes.size() > 1)
    {
      for (auto& el : dst_multi_file_path)
      {
        el = addFileSuffix(el, std::to_string(&el - &dst_multi_file_path[0]));
      }
    }
    saveLaneArray(dst_multi_file_path, lane_);
  }

  void LaneArrayCallback(const autoware_msgs::LaneArray::ConstPtr& larray)
  {
    if (larray->lanes.empty())
    {
      return;
    }
    lane_ = *larray;
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
        const geometry_msgs::Point p = el.pose.pose.position;
        const double yaw = tf::getYaw(el.pose.pose.orientation);
        const double vel = mps2kmph(el.twist.twist.linear.x);
        const int states[] =
        {
          el.change_flag, el.wpstate.steering_state, el.wpstate.accel_state,
          el.wpstate.stop_state, el.wpstate.event_state
        };
        ofs << std::fixed << std::setprecision(4);
        ofs << p.x << "," << p.y << "," << p.z << "," << yaw << "," << vel;
        for (int i = 0; i < 5; ofs << "," << states[i++]){}
        ofs << std::endl;
      }
    }
  }
};

}  // waypoint_maker

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_extractor");
  waypoint_maker::WaypointExtractor we;
  ros::spin();
  return 0;
}
