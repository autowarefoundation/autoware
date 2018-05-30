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
