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
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_cv/grid_map_cv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>

namespace
{

double g_max_distance;

void distanceTransform(grid_map::GridMap& map)
{
  cv::Mat original_image;
  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(map, "original", CV_8UC1, original_image);

  // binalize
  cv::Mat binary_image;
  cv::threshold(original_image, binary_image, 20, 100, cv::THRESH_BINARY_INV);

  // distance transform
  // 3 fast
  // 5 slow but accurate
  cv::Mat dt_image;
  cv::distanceTransform(binary_image, dt_image, CV_DIST_L2, 5);

  // Convert to int...
  cv::Mat dt_int_image(dt_image.size(), CV_8UC1);
  cv::Mat dt_int_inv_image(dt_image.size(), CV_8UC1);

  // max distance for cost propagation
  double max_dist = g_max_distance; // meter
  double resolution = map.getResolution();

  for (int y = 0; y < dt_image.rows; y++) {
    for (int x = 0; x < dt_image.cols; x++) {
      // actual distance [meter]
      double dist = dt_image.at<float>(y, x) * resolution;
      if (dist > max_dist)
        dist = max_dist;

      // Make value range 0 ~ 255
      int round_dist = dist / max_dist * 255;
      int inv_round_dist = 255 - round_dist;

      dt_int_image.at<unsigned char>(y, x)     = round_dist;
      dt_int_inv_image.at<unsigned char>(y, x) = inv_round_dist;
    }
  }

  // visualize the image
  /*
  cv::namedWindow("Original", 0);
  cv::namedWindow("Binary", 0);
  cv::namedWindow("Distance", 0);
  cv::namedWindow("DistanceInt", 0);
  cv::namedWindow("DistanceInv", 0);

  cv::imshow("Original", original_image);
  cv::imshow("Binary", binary_image);
  cv::imshow("Distance", dt_image);
  cv::imshow("DistanceInt", dt_int_image);
  cv::imshow("DistanceInv", dt_int_inv_image);
  cv::waitKey(40);
  */

  // convert to ROS msg
  grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(dt_int_inv_image, "distance_transform", map, 0, 100);
  grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(dt_int_inv_image, "dist_elevation", map, 0, 1.5);
}

void convertOccupancyGrid(const nav_msgs::OccupancyGridConstPtr& msg, const ros::Publisher& pub)
{
  // timer start
  auto start = std::chrono::system_clock::now();

  ROS_INFO("Subscribed Occupancy Grid Map");

  // convert ROS OccupancyGrid to GridMap
  grid_map::GridMap map({"original", "distance_transform", "elevation", "dist_elevation"});
  grid_map::GridMapRosConverter::fromOccupancyGrid(*msg, "original", map);

  //map.setFrameId("occupancy_grid");

  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
           map.getLength().x(), map.getLength().y(),
           map.getSize()(0), map.getSize()(1));

  // Create elevation points
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it)
  {
    map.at("elevation", *it) = map.at("original", *it) / 100.0;
  }

  // Process for distance transform
  distanceTransform(map);

  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(map, message);
  pub.publish(message);
  ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());

  // timer end
  auto end = std::chrono::system_clock::now();
  auto usec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
  std::cout << "time: " << usec / 1000.0 << " [msec]" << std::endl;
}

} // namespace

int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "dist_transform");
  ros::NodeHandle nh("~");
  ros::NodeHandle private_nh("~");

  std::string map_topic;
  private_nh.param<std::string>("map_topic", map_topic, "/realtime_cost_map");
  private_nh.param<double>("max_distance", g_max_distance, 3.0);

  ros::Publisher grid_map_pub = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

  ros::Subscriber ogm_sub = nh.subscribe<nav_msgs::OccupancyGrid>(map_topic, 1, std::bind(convertOccupancyGrid, std::placeholders::_1, grid_map_pub));

  ros::spin();

  return 0;
}
