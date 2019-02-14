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
}

static void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  if (cameraExtrinsicMat.empty() || cameraMat.empty() || distCoeff.empty() || imageSize.height == 0 ||
      imageSize.width == 0)
  {
    ROS_INFO("Looks like /camera/camera_info or /projection_matrix are not being published.. Please check that both "
             "are running..");
    return;
  }
  autoware_msgs::PointsImage pub_msg = pointcloud2_to_image(msg, cameraExtrinsicMat, cameraMat, distCoeff, imageSize);
  pub.publish(pub_msg);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "vscan2image");
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");
  std::string cameraInfo_topic_name;
  private_nh.param<std::string>("camera_info_topic", cameraInfo_topic_name, "/camera/camera_info");
  std::string projectionMat_topic_name;
  private_nh.param<std::string>("projection_matrix_topic", projectionMat_topic_name, "/projection_matrix");

  /*if(argc < 2){
    std::cout<<"Need calibration filename as the first parameter.";
    return 0;
  }

  cv::FileStorage fs(argv[1], cv::FileStorage::READ);
  if(!fs.isOpened()){
    std::cout<<"Invalid calibration filename.";
    return 0;
  }

  fs[CAMERAEXTRINSICMAT] >> cameraExtrinsicMat;
  fs[CAMERAMAT] >> cameraMat;
  fs[DISTCOEFF] >> distCoeff;
    fs[IMAGESIZE] >> imageSize;*/
  // imageSize.width = IMAGE_WIDTH;
  // imageSize.height = IMAGE_HEIGHT;

  pub = n.advertise<autoware_msgs::PointsImage>("vscan_image", 10);
  ros::Subscriber sub = n.subscribe("vscan_points", 1, callback);
  ros::Subscriber projection = n.subscribe(projectionMat_topic_name, 1, projection_callback);
  ros::Subscriber intrinsic = n.subscribe(cameraInfo_topic_name, 1, intrinsic_callback);

  ros::spin();
  return 0;
}
