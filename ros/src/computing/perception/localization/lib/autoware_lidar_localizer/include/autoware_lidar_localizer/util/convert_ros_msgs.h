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

#ifndef CONVERT_ROS_MSGS_H
#define CONVERT_ROS_MSGS_H

#include <tf/tf.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <jsk_recognition_msgs/HistogramWithRange.h>

#include "data_structs.h"

geometry_msgs::PoseStamped convertToROSMsg(const std_msgs::Header &header,
                                           const Pose &pose);
geometry_msgs::PoseStamped
convertToROSMsg(const std_msgs::Header &header, const Pose &pose,
                const tf::Transform &local_transform);

geometry_msgs::PoseWithCovarianceStamped convertToROSMsg(const std_msgs::Header &header,
                                                         const Pose &pose,
                                                         const std::array<double, 36> cov_array);

geometry_msgs::TwistStamped convertToROSMsg(const std_msgs::Header &header,
                                            const Velocity &velocity);

jsk_recognition_msgs::HistogramWithRange convertToROSMsg(const std_msgs::Header &header,
                                                         const std::vector<HistogramWithRangeBin> &histogram_bin_array);

Pose convertFromROSMsg(const geometry_msgs::Pose &msg);
Pose convertFromROSMsg(const geometry_msgs::PoseStamped &msg);
Pose convertFromROSMsg(const geometry_msgs::PoseWithCovarianceStamped &msg);

Velocity convertFromROSMsg(const geometry_msgs::Twist &msg);
Velocity convertFromROSMsg(const geometry_msgs::TwistStamped &msg);

#endif
