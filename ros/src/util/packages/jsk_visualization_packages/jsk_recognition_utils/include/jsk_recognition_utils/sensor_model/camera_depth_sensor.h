// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef JSK_RECOGNITION_UTILS_CAMERA_DEPTH_SENSOR_H_
#define JSK_RECOGNITION_UTILS_CAMERA_DEPTH_SENSOR_H_

#include "jsk_recognition_utils/sensor_model/pointcloud_sensor_model.h"
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>

namespace jsk_recognition_utils
{
  class CameraDepthSensor: public PointCloudSensorModel
  {
  public:
    typedef boost::shared_ptr<CameraDepthSensor> Ptr;
    CameraDepthSensor() {}
    
    virtual void setCameraInfo(const sensor_msgs::CameraInfo& info)
    {
      camera_info_ = info;
      model_.fromCameraInfo(info);
    }
    
    /**
     * @brief
     * get instance of image_geometry::PinholeCameraModel.
     */
    virtual image_geometry::PinholeCameraModel getPinholeCameraModel() const
    {
      return model_;
    }

    /**
     * @brief
     * return true if point p is inside of field-of-view.
     */
    virtual bool isInside(const cv::Point& p) const
    {
      return (p.x >= 0 && p.x < camera_info_.width &&
              p.y >= 0 && p.y < camera_info_.height);
    }

    /**
     * @brief
     * Return the expected number of points according to distance and area.
     * it is calculated according to:
     * f^2\frac{s}{r^2}
     **/
    virtual double expectedPointCloudNum(double distance, double area) const
    {
      double f = model_.fx();
      return f*f / (distance*distance) * area;
    }

    /**
     * @brief
     * return an image from internal camera parameter.
     */
    virtual cv::Mat image(const int type) const
    {
      return cv::Mat::zeros(camera_info_.height, camera_info_.width, type);
    }

    /**
     * @brief
     * width of camera in pixels.
     */
    virtual int width() const
    {
      return camera_info_.width;
    }

    /**
     * @brief
     * height of camera in pixels.
     */
    virtual int height() const
    {
      return camera_info_.height;
    }

    /**
     * @brief
     * force point to be inside of field of view.
     */
    virtual cv::Point limit2DPoint(const cv::Point& p) const
    {
      return cv::Point(std::min(std::max(p.x, 0), width()),
                       std::min(std::max(p.y, 0), height()));
    }

  protected:
    image_geometry::PinholeCameraModel model_;
    sensor_msgs::CameraInfo camera_info_;
  private:
    
  };
}

#endif
