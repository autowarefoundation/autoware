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


#ifndef POSEDETECTION_MSGS_FEATURE0D_TO_IMAGE_H_
#define POSEDETECTION_MSGS_FEATURE0D_TO_IMAGE_H_

#include <ros/node_handle.h>
#include <sensor_msgs/Image.h>
#include <posedetection_msgs/ImageFeature0D.h>

#include <opencv/cv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/shared_ptr.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

namespace posedetection_msgs
{
  cv::Mat draw_features(const cv::Mat src,
                        const std::vector<float> positions,
                        const std::vector<float> scales,
                        const std::vector<float> orientations)
  {
    cv::Mat dst;
    src.copyTo(dst);
    for(size_t i = 0; i < positions.size()/2; ++i) {
      float scale = i < scales.size() ? scales[i] : 10.0;
      cv::Point center = cv::Point(positions[2*i+0], positions[2*i+1]);
      cv::circle(dst, center, scale, CV_RGB(0,255,0));
      if( i < orientations.size() ) {
        // draw line indicating orientation
        cv::Point end_pt = cv::Point(center.x+std::cos(orientations[i])*scale,
            center.y+std::sin(orientations[i])*scale);
        cv::line(dst, center, end_pt, CV_RGB(255,0,0));
      }
    }
    return dst;
  }

  class Feature0DToImage
  {
  public:
    ros::NodeHandle _node;
    ros::Publisher _pub;
    ros::Subscriber _sub_imagefeature;
    typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::Image,
    posedetection_msgs::Feature0D
    > SyncPolicy;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > _sync;
    message_filters::Subscriber<sensor_msgs::Image> _sub_image;
    message_filters::Subscriber<posedetection_msgs::Feature0D> _sub_feature;

    Feature0DToImage();
    virtual ~Feature0DToImage();
    void imagefeature_cb(const posedetection_msgs::ImageFeature0DConstPtr& msg_ptr);
    void imagefeature_cb(const sensor_msgs::ImageConstPtr& image_msg,
                         const posedetection_msgs::Feature0DConstPtr& feature_msg);
  };
}

#endif
