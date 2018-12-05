/*
 *  Copyright (c) 2018, Nagoya University
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

#ifndef OBJECT_TRACKING_VISUALIZEDETECTEDOBJECTS_H
#define OBJECT_TRACKING_VISUALIZEDETECTEDOBJECTS_H

#include <ros/ros.h>
#include <std_msgs/Header.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <pcl/io/io.h>

#include <vector>
#include <string>

#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"

class VisualizeDetectedObjects
{
private:
  const double vis_arrow_height_;
  const double vis_id_height_;
  double ignore_velocity_thres_;
  double visualize_arrow_velocity_thres_;
  std::string input_topic_;
  std::string pointcloud_frame_;

  ros::NodeHandle node_handle_;
  ros::Subscriber sub_object_array_;

  ros::Publisher pub_arrow_;
  ros::Publisher pub_id_;

  void visMarkers(const autoware_msgs::DetectedObjectArray& input);
  void callBack(const autoware_msgs::DetectedObjectArray& input);

public:
  VisualizeDetectedObjects();
};

#endif  // OBJECT_TRACKING_VISUALIZEDETECTEDOBJECTS_H
