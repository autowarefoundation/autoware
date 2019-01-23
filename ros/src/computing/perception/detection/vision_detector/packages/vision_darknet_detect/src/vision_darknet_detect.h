/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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
 ********************
 *  v1.0: amc-nu (abrahammonrroy@yahoo.com)
 *
 * yolo3_node.cpp
 *
 *  Created on: April 4th, 2018
 */
#ifndef DARKNET_YOLO3_H
#define DARKNET_YOLO3_H

#define __APP_NAME__ "vision_darknet_detect"

#include <fstream>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>

#include <autoware_config_msgs/ConfigSSD.h>
#include <autoware_msgs/DetectedObject.h>
#include <autoware_msgs/DetectedObjectArray.h>

#include <rect_class_score.h>

#include <opencv2/opencv.hpp>


extern "C"
{
#undef __cplusplus
#include "box.h"
#include "image.h"
#include "network.h"
#include "detection_layer.h"
#include "parser.h"
#include "region_layer.h"
#include "utils.h"
#include "image.h"
#define __cplusplus
}

namespace darknet {
    class Yolo3Detector {
    private:
        double min_confidence_, nms_threshold_;
        network* darknet_network_;
        std::vector<box> darknet_boxes_;
        std::vector<RectClassScore<float> > forward(image &in_darknet_image);
    public:
        Yolo3Detector() {}

        void load(std::string &in_model_file, std::string &in_trained_file, double in_min_confidence,
                  double in_nms_threshold);

        ~Yolo3Detector();

        image convert_image(const sensor_msgs::ImageConstPtr &in_image_msg);

        std::vector<RectClassScore<float> > detect(image &in_darknet_image);

        uint32_t get_network_width();

        uint32_t get_network_height();


    };
}  // namespace darknet

class Yolo3DetectorNode {
    ros::Subscriber                 subscriber_image_raw_;
    ros::Subscriber                 subscriber_yolo_config_;
    ros::Publisher                  publisher_objects_;
    ros::NodeHandle                 node_handle_;

    darknet::Yolo3Detector          yolo_detector_;

    image darknet_image_ = {};

    float                           score_threshold_;
    float                           nms_threshold_;
    double                          image_ratio_;//resize ratio used to fit input image to network input size
    uint32_t                        image_top_bottom_border_;//black strips added to the input image to maintain aspect ratio while resizing it to fit the network input size
    uint32_t                        image_left_right_border_;
    std::vector<cv::Scalar>         colors_;

    std::vector<std::string>        custom_names_;
    bool                            use_coco_names_;


    void                            convert_rect_to_image_obj(std::vector< RectClassScore<float> >& in_objects,
                                      autoware_msgs::DetectedObjectArray& out_message);
    void                            rgbgr_image(image& im);
    image                           convert_ipl_to_image(const sensor_msgs::ImageConstPtr& msg);
    void                            image_callback(const sensor_msgs::ImageConstPtr& in_image_message);
    void                            config_cb(const autoware_config_msgs::ConfigSSD::ConstPtr& param);
    std::vector<std::string>        read_custom_names_file(const std::string& in_path);
public:
    void    Run();
};

#endif  // DARKNET_YOLO3_H
