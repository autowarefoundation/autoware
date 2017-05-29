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
#include "posedetection_msgs/feature0d_view.h"
#include "posedetection_msgs/feature0d_to_image.h"
#include <ros/node_handle.h>
#include <ros/master.h>
#include <posedetection_msgs/ImageFeature0D.h>

#include <opencv2/highgui/highgui.hpp>
#include <boost/shared_ptr.hpp>

#include <cv_bridge/cv_bridge.h>

namespace posedetection_msgs
{
    Feature0DView::Feature0DView()
    { 
        std::string topic = _node.resolveName("ImageFeature0D");
        ros::NodeHandle local_nh("~");
        local_nh.param("window_name", _window_name, topic);
        bool autosize;
        local_nh.param("autosize", autosize, false);

        _sub = _node.subscribe("ImageFeature0D",1,&Feature0DView::image_cb,this);
        cvNamedWindow(_window_name.c_str(), autosize ? CV_WINDOW_AUTOSIZE : 0);
        cvStartWindowThread();
    }
    Feature0DView::~Feature0DView() {}

    void Feature0DView::image_cb(const posedetection_msgs::ImageFeature0DConstPtr& msg_ptr)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg_ptr->image, "bgr8");
            cv::Mat image = draw_features(cv_ptr->image,
                                          msg_ptr->features.positions,
                                          msg_ptr->features.scales,
                                          msg_ptr->features.orientations);
            cv::imshow(_window_name.c_str(), image);
        }
        catch (cv_bridge::Exception error) {
            ROS_WARN("bad frame");
            return;
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc,argv,"feature0d_view");
    if( !ros::master::check() )
        return 1;
    
    boost::shared_ptr<posedetection_msgs::Feature0DView> node(new posedetection_msgs::Feature0DView());
    ros::spin();
    node.reset();
    return 0;
}
