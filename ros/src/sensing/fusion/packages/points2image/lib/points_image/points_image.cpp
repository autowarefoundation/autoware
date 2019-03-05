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

#include <vector>
#include <include/points_image/points_image.hpp>
#include <stdint.h>
#include <iostream>

static cv::Mat invRt, invTt;
static bool init_matrix = false;

void resetMatrix()
{
  init_matrix = false;
}

void initMatrix(const cv::Mat& cameraExtrinsicMat)
{
  invRt = cameraExtrinsicMat(cv::Rect(0, 0, 3, 3));
  cv::Mat invT = -invRt.t() * (cameraExtrinsicMat(cv::Rect(3, 0, 1, 3)));
  invTt = invT.t();
  init_matrix = true;
}

autoware_msgs::PointsImage pointcloud2_to_image(const sensor_msgs::PointCloud2ConstPtr& pointcloud2,
                                                const cv::Mat& cameraExtrinsicMat, const cv::Mat& cameraMat,
                                                const cv::Mat& distCoeff, const cv::Size& imageSize)
{
  int w = imageSize.width;
  int h = imageSize.height;

  autoware_msgs::PointsImage msg;

  msg.header = pointcloud2->header;

  msg.intensity.assign(w * h, 0);
  msg.distance.assign(w * h, 0);
  msg.min_height.assign(w * h, 0);
  msg.max_height.assign(w * h, 0);

  uintptr_t cp = (uintptr_t)pointcloud2->data.data();

  msg.max_y = -1;
  msg.min_y = h;

  msg.image_height = imageSize.height;
  msg.image_width = imageSize.width;
  if (!init_matrix)
  {
    initMatrix(cameraExtrinsicMat);
  }

  cv::Mat point(1, 3, CV_64F);
  cv::Point2d imagepoint;
  for (uint32_t y = 0; y < pointcloud2->height; ++y)
  {
    for (uint32_t x = 0; x < pointcloud2->width; ++x)
    {
      float* fp = (float*)(cp + (x + y * pointcloud2->width) * pointcloud2->point_step);
      double intensity = fp[4];
      for (int i = 0; i < 3; i++)
      {
        point.at<double>(i) = invTt.at<double>(i);
        for (int j = 0; j < 3; j++)
        {
          point.at<double>(i) += double(fp[j]) * invRt.at<double>(j, i);
        }
      }

      if (point.at<double>(2) <= 1)
      {
        continue;
      }

      double tmpx = point.at<double>(0) / point.at<double>(2);
      double tmpy = point.at<double>(1) / point.at<double>(2);
      double r2 = tmpx * tmpx + tmpy * tmpy;
      double tmpdist =
          1 + distCoeff.at<double>(0) * r2 + distCoeff.at<double>(1) * r2 * r2 + distCoeff.at<double>(4) * r2 * r2 * r2;

      imagepoint.x =
          tmpx * tmpdist + 2 * distCoeff.at<double>(2) * tmpx * tmpy + distCoeff.at<double>(3) * (r2 + 2 * tmpx * tmpx);
      imagepoint.y =
          tmpy * tmpdist + distCoeff.at<double>(2) * (r2 + 2 * tmpy * tmpy) + 2 * distCoeff.at<double>(3) * tmpx * tmpy;
      imagepoint.x = cameraMat.at<double>(0, 0) * imagepoint.x + cameraMat.at<double>(0, 2);
      imagepoint.y = cameraMat.at<double>(1, 1) * imagepoint.y + cameraMat.at<double>(1, 2);

      int px = int(imagepoint.x + 0.5);
      int py = int(imagepoint.y + 0.5);
      if (0 <= px && px < w && 0 <= py && py < h)
      {
        int pid = py * w + px;
        if (msg.distance[pid] == 0 || msg.distance[pid] > point.at<double>(2))
        {
          msg.distance[pid] = float(point.at<double>(2) * 100);
          msg.intensity[pid] = float(intensity);

          msg.max_y = py > msg.max_y ? py : msg.max_y;
          msg.min_y = py < msg.min_y ? py : msg.min_y;
        }
        if (0 == y && pointcloud2->height == 2)  // process simultaneously min and max during the first layer
        {
          float* fp2 = (float*)(cp + (x + (y + 1) * pointcloud2->width) * pointcloud2->point_step);
          msg.min_height[pid] = fp[2];
          msg.max_height[pid] = fp2[2];
        }
        else
        {
          msg.min_height[pid] = -1.25;
          msg.max_height[pid] = 0;
        }
      }
    }
  }

  return msg;
}

/*autoware_msgs::CameraExtrinsic
pointcloud2_to_3d_calibration(const sensor_msgs::PointCloud2ConstPtr& pointcloud2,
            const cv::Mat& cameraExtrinsicMat)
{
  autoware_msgs::CameraExtrinsic msg;
  std::vector<double> cali;
  for (int y = 0; y < msg.ysize ; ++y) {
    for (int x = 0; x < msg.xsize ; ++x){
      cali.push_back(cameraExtrinsicMat.at<double>(y,x));
    }
  }

  msg.calibration = cali;
  return msg;
}
 */
