/*
 *  Copyright (c) 2015, Nagoya University
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

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include "autoware_msgs/PointsImage.h"
#include "autoware_msgs/projection_matrix.h"
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

static void projection_callback(const autoware_msgs::projection_matrix& msg)
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
