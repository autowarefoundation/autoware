/*
 * Copyright 2017-2019 Autoware Foundation. All rights reserved.
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

#include <cmath>
#include <thread>

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_msgs/VehicleCmd.h>

#include "g30esli_interface_util.h"
#include "can_utils/cansend.h"
#include "can_utils/cansend_util.h"
#include "can_utils/ymc_can.h"

namespace
{
// ros param
int g_mode;
int g_loop_rate;
int g_stop_time_sec;
double g_wheel_base;
std::string g_device;

// ros publisher
ros::Publisher g_current_twist_pub;

// variables
uint16_t g_target_velocity_ui16;
int16_t g_steering_angle_deg_i16;
double g_current_vel_kmph = 0.0;
bool g_terminate_thread = false;
double g_steering_offset_deg = 0.0;

// cansend tool
mycansend::CanSender g_cansender;

void vehicle_cmd_callback(const autoware_msgs::VehicleCmdConstPtr& msg)
{
  // TODO: use steer angle, shift, turn signal
  double target_velocity = msg->twist_cmd.twist.linear.x * 3.6;  // [m/s] -> [km/h]
  double target_steering_angle_deg = ymc::computeTargetSteeringAngleDegree(msg->twist_cmd.twist.angular.z,
                                                                           msg->twist_cmd.twist.linear.x, g_wheel_base);
  target_steering_angle_deg += g_steering_offset_deg;

  // factor
  target_velocity *= 10.0;
  target_steering_angle_deg *= 10.0;

  g_target_velocity_ui16 = target_velocity;
  g_steering_angle_deg_i16 = target_steering_angle_deg * -1.0;
}

void current_vel_callback(const geometry_msgs::TwistStampedConstPtr& msg)
{
  g_current_vel_kmph = msg->twist.linear.x * 3.6;
}

// receive input from keyboard
// change the mode to manual mode or auto drive mode
void changeMode()
{
  while (!g_terminate_thread)
  {
    if (ymc::kbhit())
    {
      char c = getchar();
      if (c == ' ')
      {
        g_mode = 3;
      }
      else if (c == 's')
      {
        g_mode = 8;
      }
    }
    usleep(20000);  // sleep 20msec
  }
}

// read can data from the vehicle
void readCanData(FILE* fp)
{
  char buf[64];
  while (fgets(buf, sizeof(buf), fp) != NULL && !g_terminate_thread)
  {
    std::string raw_data = std::string(buf);
    std::cout << "received data: " << raw_data << std::endl;

    // check if the data is valid
    if (raw_data.size() > 0)
    {
      // split data to strings
      // data format is like this
      // can0  200   [8]  08 00 00 00 01 00 01 29
      std::vector<std::string> parsed_data = ymc::splitString(raw_data);

      // delete first 3 elements
      std::vector<std::string> data = parsed_data;
      data.erase(data.begin(), data.begin() + 3);

      int id = std::stoi(parsed_data.at(1), nullptr, 10);
      double _current_vel_mps = ymc::translateCanData(id, data, &g_mode);
      if (_current_vel_mps != RET_NO_PUBLISH)
      {
        geometry_msgs::TwistStamped ts;
        ts.header.frame_id = "base_link";
        ts.header.stamp = ros::Time::now();
        ts.twist.linear.x = _current_vel_mps;
        g_current_twist_pub.publish(ts);
      }
    }
  }
}

}  // namespace

int main(int argc, char* argv[])
{
  // ROS initialization
  ros::init(argc, argv, "g30esli_interface");
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");

  private_nh.param<double>("wheel_base", g_wheel_base, 2.4);
  private_nh.param<int>("mode", g_mode, 8);
  private_nh.param<std::string>("device", g_device, "can0");
  private_nh.param<int>("loop_rate", g_loop_rate, 100);
  private_nh.param<int>("stop_time_sec", g_stop_time_sec, 1);
  private_nh.param<double>("steering_offset_deg", g_steering_offset_deg, 0.0);

  // init cansend tool
  g_cansender.init(g_device);

  // subscriber
  ros::Subscriber vehicle_cmd_sub = n.subscribe<autoware_msgs::VehicleCmd>("vehicle_cmd", 1, vehicle_cmd_callback);
  ros::Subscriber current_vel_sub =
      n.subscribe<geometry_msgs::TwistStamped>("current_velocity", 1, current_vel_callback);

  // publisher
  g_current_twist_pub = n.advertise<geometry_msgs::TwistStamped>("ymc_current_twist", 10);

  // read can data from candump
  FILE* fp = popen("candump can0", "r");

  // create threads
  std::thread t1(changeMode);
  std::thread t2(readCanData, fp);

  ros::Rate loop_rate = g_loop_rate;
  bool stopping_flag = true;
  while (ros::ok())
  {
    // data
    unsigned char mode = static_cast<unsigned char>(g_mode);
    unsigned char shift = 0;
    uint16_t target_velocity_ui16 = g_target_velocity_ui16;
    int16_t steering_angle_deg_i16 = g_steering_angle_deg_i16;
    static unsigned char brake = 1;
    static unsigned char heart_beat = 0;

    // change to STOP MODE...
    if (target_velocity_ui16 == 0 || g_mode == 3)
      stopping_flag = true;

    // STOPPING
    static int stop_count = 0;
    if (stopping_flag && g_mode == 8)
    {
      // TODO: change steering angle according to current velocity ?
      target_velocity_ui16 = 0;
      brake = 1;

      // vehicle is stopping or not
      double stopping_threshold = 1.0;
      g_current_vel_kmph < stopping_threshold ? stop_count++ : stop_count = 0;

      std::cout << "stop count: " << stop_count << std::endl;
      // vehicle has stopped, so we can restart
      if (stop_count > g_loop_rate * g_stop_time_sec)
      {
        brake = 0;
        stop_count = 0;
        stopping_flag = false;
      }
    }

    // Insert data to 8 byte array
    unsigned char data[8] = {};
    ymc::setCanData(data, mode, shift, target_velocity_ui16, steering_angle_deg_i16, brake, heart_beat);

    // send can data
    std::string send_id("200");
    size_t data_size = sizeof(data) / sizeof(data[0]);
    char can_cmd[256];
    std::strcpy(can_cmd, mycansend::makeCmdArgument(data, data_size, send_id).c_str());
    g_cansender.send(can_cmd);

    // wait for callbacks
    ros::spinOnce();
    loop_rate.sleep();

    heart_beat += 1;
  }

  g_terminate_thread = true;
  t1.join();
  t2.join();

  pclose(fp);

  return 0;
}
