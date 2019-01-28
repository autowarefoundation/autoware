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
#include "autoware_msgs/Lane.h"
#include <iostream>

static ros::Publisher _pub;

void callback(const autoware_msgs::Lane &msg)
{
    _pub.publish(msg);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_select");

    ros::NodeHandle nh;
    ros::Subscriber twist_sub = nh.subscribe("temporal_waypoints", 1, callback);
    _pub = nh.advertise<autoware_msgs::Lane>("final_waypoints", 1000,true);

    ros::spin();



    return 0;
}
