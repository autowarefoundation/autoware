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

#ifndef _POINTS_IMAGE_H_
#define _POINTS_IMAGE_H_

#include <opencv2/opencv.hpp>
#include <sensor_msgs/PointCloud2.h>
#include "autoware_msgs/PointsImage.h"

void resetMatrix();
autoware_msgs::PointsImage pointcloud2_to_image(const sensor_msgs::PointCloud2ConstPtr& pointclound2,
                                                const cv::Mat& cameraExtrinsicMat, const cv::Mat& cameraMat,
                                                const cv::Mat& distCoeff, const cv::Size& imageSize);

/*points2image::CameraExtrinsic
pointcloud2_to_3d_calibration(const sensor_msgs::PointCloud2ConstPtr& pointclound2,
            const cv::Mat& cameraExtrinsicMat);
 */
#endif /* _POINTS_IMAGE_H_ */
