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

#ifndef JSK_RECOGNITION_UTILS_SPINDLE_LASER_SENSOR_H_
#define JSK_RECOGNITION_UTILS_SPINDLE_LASER_SENSOR_H_

#include "jsk_recognition_utils/sensor_model/pointcloud_sensor_model.h"

namespace jsk_recognition_utils
{
  class SpindleLaserSensor: public PointCloudSensorModel
  {
  public:
    typedef boost::shared_ptr<SpindleLaserSensor> Ptr;
    
    SpindleLaserSensor(const double min_angle, const double max_angle,
                       const double laser_freq,
                       const size_t point_sample):
      min_angle_(min_angle), max_angle_(max_angle),
      laser_freq_(laser_freq),
      point_sample_(point_sample) { }
    
    virtual void setSpindleVelocity(const double velocity)
    {
      spindle_velocity_ = spindle_velocity;
    }

    /**
     * @brief
     * Return the expected number of points according to distance and area.
     * it is calculated according to:
     * \frac{N}{2 \pi \Delta \phi}\frac{1}{r^2}s
     * \Delta \phi = \frac{2 \pi}{\omega}
     */
    virtual double expectedPointCloudNum(double distance, double area) const
    {
      assert(spindle_velocity_ != 0.0);
      double dphi = 2.0 * M_PI / spindle_velocity_;
      return point_sample_ * laser_freq_ / (2.0 * dphi) / (distance * distance) * area;
    }
    
  protected:
    
    double spindle_velocity_;
    double min_angle_;
    double max_angle_;
    size_t point_sample_;
  private:
    
  };
}

#endif 
