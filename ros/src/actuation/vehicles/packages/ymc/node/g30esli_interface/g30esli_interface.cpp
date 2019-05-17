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

#include "g30esli_interface.h"

G30esliInterface::G30esliInterface() : nh_(), private_nh_("~")
{
  // rosparam
  private_nh_.param<std::string>("device", device_, "can0");
  private_nh_.param<bool>("use_ds4", use_ds4_, false);
  private_nh_.param<double>("steering_offset_deg", steering_offset_deg_, 0.0);
  private_nh_.param<double>("command_timeout", command_timeout_, 1000);
  private_nh_.param<double>("brake_threshold", brake_threshold_, 0.1);

  // engaged at startup
  private_nh_.param<bool>("engaged", engage_, true);

  // subscriber
  vehicle_cmd_sub_ =
      nh_.subscribe<autoware_msgs::VehicleCmd>("vehicle_cmd", 1, &G30esliInterface::vehicleCmdCallback, this);
  engage_sub_ = nh_.subscribe<std_msgs::Bool>("vehicle/engage", 1, &G30esliInterface::engageCallback, this);

  if (!use_ds4_)
  {
    mode_ = MODE::AUTO;  // use vehicle_cmd at startup
  }
  else
  {
    ds4_sub_ = nh_.subscribe<ds4_msgs::DS4>("ds4", 1, &G30esliInterface::ds4Callback, this);
    mode_ = MODE::JOYSTICK;  // use ds4 at startup
  }

  // publisher
  vehicle_status_pub_ = nh_.advertise<autoware_msgs::VehicleStatus>("vehicle_status", 10);
  current_twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("vehicle/twist", 10);
  battery_pub_ = nh_.advertise<std_msgs::Float32>("vehicle/battery", 10);

  terminate_thread_ = false;
}

G30esliInterface::~G30esliInterface()
{
  terminate_thread_ = true;

  thread_read_status_->join();
  thread_read_keyboard_->join();
  thread_publish_status_->join();

  delete thread_read_status_;
  delete thread_read_keyboard_;
  delete thread_publish_status_;
}

// generate command by autoware
void G30esliInterface::vehicleCmdCallback(const autoware_msgs::VehicleCmdConstPtr& msg)
{
  g30esli_ros_.updateAutoCommand(*msg, engage_, steering_offset_deg_, brake_threshold_);
}

// engaging by topic
void G30esliInterface::engageCallback(const std_msgs::BoolConstPtr& msg)
{
  bool engaging = msg->data;
  engage_mutex_.lock();
  if (!engage_ && engaging)
  {
    engage_ = true;
    engage_start_ = ros::Time::now();
    ROS_INFO("TOPIC: Engaged");
  }
  else if (engage_ && !engaging)
  {
    engage_ = false;
    ROS_WARN("TOPIC: Disengaged");
  }
  engage_mutex_.unlock();
}

// generate command by ds4 joystick
void G30esliInterface::ds4Callback(const ds4_msgs::DS4ConstPtr& msg)
{
  g30esli_ros_.updateJoystickCommand(*msg, engage_, steering_offset_deg_);

  // joystick override = use JOYSTICK mode
  if ((msg->square || msg->cross) && mode_ != MODE::JOYSTICK)
  {
    ROS_INFO("JOYSTICK: Joystick Manual Mode");
    mode_ = MODE::JOYSTICK;
  }

  // autonomous drive mode = use AUTO mode
  if (msg->ps && mode_ == MODE::JOYSTICK)
  {
    ROS_INFO("JOYSTICK: Autonomous Driving Mode");
    mode_ = MODE::AUTO;
  }

  // engage vehicle
  engage_mutex_.lock();
  if (msg->option && !engage_)
  {
    engage_ = true;
    engage_start_ = ros::Time::now();
    ROS_INFO("JOYSTICK: Engaged");
  }
  if (msg->share && engage_)
  {
    engage_ = false;
    ROS_WARN("JOYSTICK: Disengaged");
  }
  engage_mutex_.unlock();
}

// read vehicle status
void G30esliInterface::readStatus()
{
  while (!terminate_thread_)
  {
    g30esli_ros_.receiveStatus(steering_offset_deg_);

    // accel/brake override, switch to manual mode
    engage_mutex_.lock();
    double dt = (ros::Time::now() - engage_start_).toSec();
    if (g30esli_ros_.checkOverride() && engage_ && (dt > 0.3))  // 300ms wait
    {
      engage_ = false;
      ROS_WARN("OVERRIDE: Disengaged");
    }
    engage_mutex_.unlock();
  }
}

// receive input from keyboard
// change the mode to manual mode or auto drive mode
void G30esliInterface::readKeyboard()
{
  ros::Rate rate = ros::Rate(100);
  while (!terminate_thread_)
  {
    if (kbhit())
    {
      char c = getchar();
      if (c == 's')
      {
        engage_mutex_.lock();
        if (!engage_)
        {
          engage_ = true;
          engage_start_ = ros::Time::now();
          ROS_INFO("KEYBOARD: Engaged");
        }
        engage_mutex_.unlock();

        if (mode_ == MODE::JOYSTICK)
        {
          mode_ = MODE::AUTO;
          ROS_INFO("KEYBOARD: Autonomous Driving Mode");
        }
      }
      else if (c == ' ')
      {
        engage_mutex_.lock();
        if (engage_)
        {
          engage_ = false;
          ROS_WARN("KEYBOARD: Disengaged");
        }
        engage_mutex_.unlock();

        if (mode_ == MODE::JOYSTICK)
        {
          mode_ = MODE::AUTO;
          ROS_INFO("KEYBOARD: Autonomous Driving Mode");
        }
      }
    }
    rate.sleep();
  }
}

// publish vehicle status
void G30esliInterface::publishStatus()
{
  ros::Rate rate = ros::Rate(100);
  std_msgs::Float32 battery;
  while (!terminate_thread_)
  {
    battery.data = g30esli_ros_.getBatteryCharge();
    vehicle_status_pub_.publish(g30esli_ros_.getVehicleStatus());
    current_twist_pub_.publish(g30esli_ros_.getCurrentTwist());
    battery_pub_.publish(battery);
    rate.sleep();
  }
}

// main loop
void G30esliInterface::run()
{
  // open can device
  if (!g30esli_ros_.openDevice(device_))
  {
    ROS_ERROR("Cannot open device: %s", device_.c_str());
    return;
  }

  // start threads
  thread_read_status_ = new std::thread(&G30esliInterface::readStatus, this);
  thread_read_keyboard_ = new std::thread(&G30esliInterface::readKeyboard, this);
  thread_publish_status_ = new std::thread(&G30esliInterface::publishStatus, this);

  ros::Rate rate(100);

  while (ros::ok())
  {
    const MODE& mode = mode_;

    // update subscribers
    ros::spinOnce();

    // check command
    if (g30esli_ros_.checkTimeout(mode, command_timeout_))
    {
      // semi-emergency stop if timeout occured
      ROS_ERROR("Emergency stop by timeout...");
      g30esli_ros_.emergencyStop(mode);
    }

    // reset speed command when restarting
    g30esli_ros_.checkRestart(mode);

    // send command
    g30esli_ros_.sendCommand(mode);

    // update heart beat
    g30esli_ros_.updateAliveCounter();

    // debug
    ROS_DEBUG("\n%s", g30esli_ros_.dumpDebug(mode).c_str());

    // loop sleep
    rate.sleep();
  }
}

// detect hit of keyboard
bool G30esliInterface::kbhit()
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return true;
  }

  return false;
}
