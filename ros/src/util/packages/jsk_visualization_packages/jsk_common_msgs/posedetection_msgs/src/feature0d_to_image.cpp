// vim: set tabstop=4 shiftwidth=4:
// Copyright (C) 2008-2009 Rosen Diankov
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "posedetection_msgs/feature0d_to_image.h"
#include <ros/node_handle.h>
#include <sensor_msgs/Image.h>
#include <posedetection_msgs/ImageFeature0D.h>

#include <opencv2/highgui/highgui.hpp>
#include <boost/shared_ptr.hpp>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/synchronizer.h>

namespace posedetection_msgs
{
    Feature0DToImage::Feature0DToImage()
    {
        ros::NodeHandle local_nh("~");

        _pub = _node.advertise<sensor_msgs::Image>(local_nh.resolveName("output"), 1);
        _sub_image.subscribe(_node, "image", 1);
        _sub_feature.subscribe(_node, "Feature0D", 1);
        _sync = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
        _sync->connectInput(_sub_image, _sub_feature);
        _sync->registerCallback(boost::bind(&Feature0DToImage::imagefeature_cb, this, _1, _2));
        _sub_imagefeature = _node.subscribe("ImageFeature0D", 1, &Feature0DToImage::imagefeature_cb, this);
    }
    Feature0DToImage::~Feature0DToImage() {}

    void Feature0DToImage::imagefeature_cb(const sensor_msgs::ImageConstPtr& image_msg,
                                           const posedetection_msgs::Feature0DConstPtr& feature_msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(image_msg, "bgr8");
            cv::Mat image = draw_features(cv_ptr->image,
                                          feature_msg->positions,
                                          feature_msg->scales,
                                          feature_msg->orientations);
            _pub.publish(cv_bridge::CvImage(cv_ptr->header, "bgr8", image));
        } catch (cv_bridge::Exception& error) {
            ROS_WARN("bad frame");
            return;
        }
    }

    void Feature0DToImage::imagefeature_cb(const posedetection_msgs::ImageFeature0DConstPtr& msg_ptr)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg_ptr->image, "bgr8");
            cv::Mat image = draw_features(cv_ptr->image,
                                          msg_ptr->features.positions,
                                          msg_ptr->features.scales,
                                          msg_ptr->features.orientations);
            _pub.publish(cv_bridge::CvImage(cv_ptr->header, "bgr8", image));
        } catch (cv_bridge::Exception& error) {
            ROS_WARN("bad frame");
            return;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature0d_to_image");
    boost::shared_ptr<posedetection_msgs::Feature0DToImage> node(new posedetection_msgs::Feature0DToImage());
    ros::spin();
    return 0;
}

