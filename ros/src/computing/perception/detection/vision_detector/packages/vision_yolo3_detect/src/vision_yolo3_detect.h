/*
 *  Copyright (c) 2018, Nagoya University
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
 ********************
 *  v1.0: amc-nu (abrahammonrroy@yahoo.com)
 *
 * yolo3_node.cpp
 *
 *  Created on: April 4th, 2018
 */
#ifndef DARKNET_YOLO3_H
#define DARKNET_YOLO3_H

#define __APP_NAME__ "vision_yolo3_detect"

#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>

#include <autoware_msgs/ConfigSsd.h>
#include <autoware_msgs/image_obj.h>

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

namespace Yolo3
{
    enum YoloDetectorClasses//using coco for default cfg and weights
    {
        PERSON, BICYCLE, CAR, MOTORBIKE, AEROPLANE, BUS, TRAIN, TRUCK, BOAT, TRAFFIC_LIGHT,
        FIRE_HYDRANT, STOP_SIGN, PARKING_METER, BENCH, BIRD, CAT, DOG, HORSE, SHEEP, COW,
        ELEPHANT, BEAR, ZEBRA, GIRAFFE, BACKPACK, UMBRELLA, HANDBAG, TIE, SUITCASE, FRISBEE,
        SKIS, SNOWBOARD, SPORTS_BALL, KITE, BASEBALL_BAT, BASEBALL_GLOVE, SKATEBOARD, SURFBOARD, TENNIS_RACKET, BOTTLE,
        WINE_GLASS, CUP, FORK, KNIFE, SPOON, BOWL, BANANA, APPLE, SANDWICH, ORANGE,
        BROCCOLI, CARROT, HOT_DOG, PIZZA, DONUT, CAKE, CHAIR, SOFA, POTTEDPLANT, BED,
        DININGTABLE, TOILET, TVMONITOR, LAPTOP, MOUSE, REMOTE, KEYBOARD, CELL_PHONE, MICROWAVE, OVEN,
        TOASTER, SINK, REFRIGERATOR, BOOK, CLOCK, VASE, SCISSORS, TEDDY_BEAR, HAIR_DRIER, TOOTHBRUSH,
    };
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
    ros::Subscriber subscriber_image_raw_;
    ros::Subscriber subscriber_yolo_config_;
    ros::Publisher publisher_car_objects_;
    ros::Publisher publisher_person_objects_;
    ros::NodeHandle node_handle_;

    darknet::Yolo3Detector yolo_detector_;

    image darknet_image_ = {};

    float score_threshold_;
    float nms_threshold_;
    double image_ratio_;//resdize ratio used to fit input image to network input size
    uint32_t image_top_bottom_border_;//black strips added to the input image to maintain aspect ratio while resizing it to fit the network input size
    uint32_t image_left_right_border_;

    void convert_rect_to_image_obj(std::vector< RectClassScore<float> >& in_objects, autoware_msgs::image_obj& out_message, std::string in_class);
    void rgbgr_image(image& im);
    image convert_ipl_to_image(const sensor_msgs::ImageConstPtr& msg);
    void image_callback(const sensor_msgs::ImageConstPtr& in_image_message);
    void config_cb(const autoware_msgs::ConfigSsd::ConstPtr& param);
public:
    void Run();
};

#endif  // DARKNET_YOLO3_H
