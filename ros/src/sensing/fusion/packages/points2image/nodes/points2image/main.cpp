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

#include <QCoreApplication>

#include <iostream>
#include <qstring.h>

#include <opencv2/opencv.hpp>
#include <sensor_msgs/PointCloud2.h>

#include <rosinterface.h>

#define CAMERAEXTRINSICMAT "CameraExtrinsicMat"
#define CAMERAMAT "CameraMat"
#define DISTCOEFF "DistCoeff"
#define IMAGESIZE "ImageSize"

struct POINT2IMAGE
{
  uchar intensity;
  ushort distance;
};

struct POINTS2IMAGE
{
  std_msgs::Header header;
  int height;
  int width;
  std::vector<POINT2IMAGE> points;
};

typedef boost::shared_ptr<POINTS2IMAGE> POINTS2IMAGEPtr;

static POINTS2IMAGEPtr points_to_image(sensor_msgs::PointCloud2ConstPtr velodyneData, cv::Mat cameraExtrinsicMat,
                                       cv::Mat cameraMat, cv::Mat distCoeff, cv::Size imageSize)
{
  POINTS2IMAGEPtr result(new POINTS2IMAGE);
  result->header = velodyneData->header;
  result->height = imageSize.height;
  result->width = imageSize.width;
  POINT2IMAGE tmpdata;
  tmpdata.intensity = 0;
  tmpdata.distance = 0;
  result->points.resize(result->height * result->width, tmpdata);

  cv::Mat invR = cameraExtrinsicMat(cv::Rect(0, 0, 3, 3)).t();
  cv::Mat invT = -invR * (cameraExtrinsicMat(cv::Rect(3, 0, 1, 3)));

  int n = velodyneData->height;
  int m = velodyneData->width;
  char* data = (char*)(velodyneData->data.data());
  for (int i = 0; i < n; i++)
  {
    for (int j = 0; j < m; j++)
    {
      int id = i * m + j;
      float* dataptr = (float*)(data + id * velodyneData->point_step);

      cv::Mat point(1, 3, CV_64F);
      point.at<double>(0) = double(dataptr[0]);
      point.at<double>(1) = double(dataptr[1]);
      point.at<double>(2) = double(dataptr[2]);
      point = point * invR.t() + invT.t();

      if (point.at<double>(2) <= 0)
      {
        continue;
      }

      double tmpx = point.at<double>(0) / point.at<double>(2);
      double tmpy = point.at<double>(1) / point.at<double>(2);
      double r2 = tmpx * tmpx + tmpy * tmpy;
      double tmpdist =
          1 + distCoeff.at<double>(0) * r2 + distCoeff.at<double>(1) * r2 * r2 + distCoeff.at<double>(4) * r2 * r2 * r2;

      cv::Point2d imagepoint;
      imagepoint.x =
          tmpx * tmpdist + 2 * distCoeff.at<double>(2) * tmpx * tmpy + distCoeff.at<double>(3) * (r2 + 2 * tmpx * tmpx);
      imagepoint.y =
          tmpy * tmpdist + distCoeff.at<double>(2) * (r2 + 2 * tmpy * tmpy) + 2 * distCoeff.at<double>(3) * tmpx * tmpy;
      imagepoint.x = cameraMat.at<double>(0, 0) * imagepoint.x + cameraMat.at<double>(0, 2);
      imagepoint.y = cameraMat.at<double>(1, 1) * imagepoint.y + cameraMat.at<double>(1, 2);
      if (imagepoint.x >= 0 && imagepoint.x < result->width && imagepoint.y >= 0 && imagepoint.y < result->height)
      {
        int px = int(imagepoint.x + 0.5);
        int py = int(imagepoint.y + 0.5);
        int pid = py * result->width + px;
        if (result->points[pid].distance == 0 || (result->points[pid].distance / 100.0) > point.at<double>(2))
        {
          result->points[pid].distance = ushort(point.at<double>(2) * 100 + 0.5);
          result->points[pid].intensity = uchar(dataptr[4]);
        }
      }
    }
  }
  return result;
}

int main(int argc, char* argv[])
{
  QCoreApplication a(argc, argv);

  if (argc < 2)
  {
    std::cout << "Need calibration filename as the first parameter.";
    return 0;
  }

  cv::Mat cameraextrinsicmat;
  cv::Mat cameramat;
  cv::Mat distcoeff;
  cv::Size imagesize;

  cv::FileStorage fs(QString(argv[1]).toStdString(), cv::FileStorage::READ);
  if (!fs.isOpened())
  {
    std::cout << "Invalid calibration filename.";
    return 0;
  }

  fs[CAMERAEXTRINSICMAT] >> cameraextrinsicmat;
  fs[CAMERAMAT] >> cameramat;
  fs[DISTCOEFF] >> distcoeff;
  fs[IMAGESIZE] >> imagesize;

  ROSSub<sensor_msgs::PointCloud2ConstPtr>* velodyne =
      new ROSSub<sensor_msgs::PointCloud2ConstPtr>("points_raw", 1000, 10, "points2image");
  //   ROSPub<POINTS2IMAGEPtr> * publisher=new ROSPub<POINTS2IMAGEPtr>("points_image",1000,"points2image");

  while (ros::ok())
  {
    sensor_msgs::PointCloud2ConstPtr msg = velodyne->getMessage();
    if (msg == NULL)
    {
      continue;
    }
    POINTS2IMAGEPtr points2image = points_to_image(msg, cameraextrinsicmat, cameramat, distcoeff, imagesize);
    //       publisher->sendMessage(points2image);
  }
  return a.exec();
}
