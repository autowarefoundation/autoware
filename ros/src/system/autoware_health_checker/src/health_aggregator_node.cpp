/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
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
 *
 *
 * v1.0 Masaya Kataoka
 */

#include <ros/ros.h>

#include <autoware_health_checker/health_aggregator.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "health_aggregator");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  HealthAggregator agg(nh, pnh);
  agg.run();
  ros::spin();
  return 0;
}