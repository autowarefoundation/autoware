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

// ROS includes
#include <ros/ros.h>

#include "nmea2tfpose_core.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nmea2tfpose");
  gnss_localizer::Nmea2TFPoseNode ntpn;
  ntpn.run();

  return 0;
}
