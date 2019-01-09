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
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include "autoware_msgs/PointsImage.h"
#include "autoware_msgs/ProjectionMatrix.h"
//#include "autoware_msgs/CameraExtrinsic.h"

#include <include/points_image/points_image.hpp>

#define CAMERAEXTRINSICMAT "CameraExtrinsicMat"
#define CAMERAMAT "CameraMat"
#define DISTCOEFF "DistCoeff"
#define IMAGESIZE "ImageSize"

static cv::Mat cameraExtrinsicMat;
static cv::Mat cameraMat;
static cv::Mat distCoeff;
static cv::Size imageSize;

static ros::Publisher pub;

static void projection_callback(const autoware_msgs::ProjectionMatrix& msg)
{
  cameraExtrinsicMat = cv::Mat(4, 4, CV_64F);
  for (int row = 0; row < 4; row++)
  {
    for (int col = 0; col < 4; col++)
    {
      cameraExtrinsicMat.at<double>(row, col) = msg.projection_matrix[row * 4 + col];
    }
  }
  resetMatrix();
}

static void intrinsic_callback(const sensor_msgs::CameraInfo& msg)
{
  imageSize.height = msg.height;
  imageSize.width = msg.width;

  cameraMat = cv::Mat(3, 3, CV_64F);
  for (int row = 0; row < 3; row++)
  {
    for (int col = 0; col < 3; col++)
    {
      cameraMat.at<double>(row, col) = msg.K[row * 3 + col];
    }
  }

  distCoeff = cv::Mat(1, 5, CV_64F);
  for (int col = 0; col < 5; col++)
  {
    distCoeff.at<double>(col) = msg.D[col];
  }
  resetMatrix();
}

static void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  if (cameraExtrinsicMat.empty() || cameraMat.empty() || distCoeff.empty() || imageSize.height == 0 ||
      imageSize.width == 0)
  {
    ROS_INFO("[points2image]Looks like camera_info or projection_matrix are not being published.. Please check that "
             "both are running..");
    return;
  }

  autoware_msgs::PointsImage pub_msg = pointcloud2_to_image(msg, cameraExtrinsicMat, cameraMat, distCoeff, imageSize);
  pub.publish(pub_msg);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "points2image");
  ros::NodeHandle n;

  ros::NodeHandle private_nh("~");

  std::string camera_info_topic_str;
  std::string projection_matrix_topic;
  std::string pub_topic_str = "/points_image";

  private_nh.param<std::string>("projection_matrix_topic", projection_matrix_topic, "/projection_matrix");
  private_nh.param<std::string>("camera_info_topic", camera_info_topic_str, "/camera_info");

  std::string name_space_str = ros::this_node::getNamespace();

  if (name_space_str != "/")
  {
    if (name_space_str.substr(0, 2) == "//")
    {
      /* if name space obtained by ros::this::node::getNamespace()
         starts with "//", delete one of them */
      name_space_str.erase(name_space_str.begin());
    }
    pub_topic_str = name_space_str + pub_topic_str;
    projection_matrix_topic = name_space_str + projection_matrix_topic;
    camera_info_topic_str = name_space_str + camera_info_topic_str;
  }

  std::string points_topic;
  if (private_nh.getParam("points_node", points_topic))
  {
    ROS_INFO("[points2image]Setting points node to %s", points_topic.c_str());
  }
  else
  {
    ROS_INFO("[points2image]No points node received, defaulting to points_raw, you can use _points_node:=YOUR_TOPIC");
    points_topic = "/points_raw";
  }

  ROS_INFO("[points2image]Publishing to... %s", pub_topic_str.c_str());
  pub = n.advertise<autoware_msgs::PointsImage>(pub_topic_str, 10);

  ros::Subscriber sub = n.subscribe(points_topic, 1, callback);

  ROS_INFO("[points2image]Subscribing to... %s", projection_matrix_topic.c_str());
  ros::Subscriber projection = n.subscribe(projection_matrix_topic, 1, projection_callback);
  ROS_INFO("[points2image]Subscribing to... %s", camera_info_topic_str.c_str());
  ros::Subscriber intrinsic = n.subscribe(camera_info_topic_str, 1, intrinsic_callback);

  ros::spin();
  return 0;
}
