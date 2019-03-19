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
#include <string>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "autoware_config_msgs/ConfigSSD.h"
#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"

#include <rect_class_score.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#if (CV_MAJOR_VERSION <= 2)

#include <opencv2/contrib/contrib.hpp>

#else
#include "gencolors.cpp"
#endif

#include "vision_ssd_detect.h"

class ROSSSDApp
{
  ros::Subscriber subscriber_image_raw_;
  ros::Subscriber subscriber_ssd_config_;
  ros::Publisher publisher_detected_objects_;
  ros::NodeHandle node_handle_;

  cv::Scalar pixel_mean_;
  std::vector<cv::Scalar> colors_;

  //Caffe based Object Detection ConvNet
  SSDDetector* ssd_detector_;

  //The minimum score required to filter the detected objects by the ConvNet
  float score_threshold_;

  //If GPU is enabled, stores the GPU Device to use
  unsigned int gpu_device_id_;

  //Sets whether or not use GPU acceleration
  bool use_gpu_;

  //vector of indices of the classes to search for
  std::vector<unsigned int> detect_classes_;

  void convert_rect_to_detected_object(std::vector< RectClassScore<float> >& in_objects,
                                       autoware_msgs::DetectedObjectArray &out_message,
                                       cv::Mat& in_image)
  {
    for (unsigned int i = 0; i < in_objects.size(); ++i)
    {
      if (in_objects[i].score >= score_threshold_)
      {
        autoware_msgs::DetectedObject obj;
        obj.header = out_message.header;
        obj.label = in_objects[i].GetClassString();
        obj.score = in_objects[i].score;

        obj.color.r = colors_[in_objects[i].class_type].val[0];
        obj.color.g = colors_[in_objects[i].class_type].val[1];
        obj.color.b = colors_[in_objects[i].class_type].val[2];
        obj.color.a = 1.0f;

        obj.image_frame = out_message.header.frame_id;
        obj.x = in_objects[i].x;
        obj.y = in_objects[i].y;
        obj.width = in_objects[i].w;
        obj.height = in_objects[i].h;
        obj.valid = true;

        //obj.roi_image = in_image(cv::Rect(obj.x, obj.y, obj.width, obj.height));

        out_message.objects.push_back(obj);
      }
    }
  }

  void image_callback(const sensor_msgs::Image &image_source)
  {
    //Receive Image, convert it to OpenCV Mat
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_source,
                                                         "bgr8");//toCvCopy(image_source, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_image->image;

    //Detect Object in image
    std::vector<RectClassScore < float> > detections;
    //cv::TickMeter timer; timer.start();
    //std::cout << "score:" << score_threshold_ << " slices:" << image_slices_ << " slices overlap:" << slices_overlap_ << "nms" << group_threshold_ << std::endl;
    detections = ssd_detector_->Detect(image);
    //timer.stop();
    //std::cout << "Detection took: " << timer.getTimeMilli() << std::endl;

    //Prepare Output message
    //Convert Objects to Message type
    //timer.reset(); timer.start();
    autoware_msgs::DetectedObjectArray output_detected_message;
    output_detected_message.header = image_source.header;
    convert_rect_to_detected_object(detections, output_detected_message, image);

    publisher_detected_objects_.publish(output_detected_message);
  }


  void config_cb(const autoware_config_msgs::ConfigSSD::ConstPtr& param)
  {
    score_threshold_ 	= param->score_threshold;
  }

public:
  void Run()
  {
    //ROS STUFF
    ros::NodeHandle private_node_handle("~");//to receive args

    //RECEIVE IMAGE TOPIC NAME
    std::string image_raw_topic_str;
    if (private_node_handle.getParam("image_raw_node", image_raw_topic_str))
    {
      ROS_INFO("Setting image node to %s", image_raw_topic_str.c_str());
    } else
    {
      ROS_INFO("No image node received, defaulting to /image_raw, you can use _image_raw_node:=YOUR_TOPIC");
      image_raw_topic_str = "/image_raw";
    }

    //RECEIVE CONVNET FILENAMES
    std::string network_definition_file;
    std::string pretrained_model_file;
    if (private_node_handle.getParam("network_definition_file", network_definition_file))
    {
      ROS_INFO("Network Definition File: %s", network_definition_file.c_str());
    } else
    {
      ROS_INFO("No Network Definition File was received. Finishing execution.");
      return;
    }
    if (private_node_handle.getParam("pretrained_model_file", pretrained_model_file))
    {
      ROS_INFO("Pretrained Model File: %s", pretrained_model_file.c_str());
    } else
    {
      ROS_INFO("No Pretrained Model File was received. Finishing execution.");
      return;
    }

    if (private_node_handle.getParam("score_threshold", score_threshold_))
    {
      ROS_INFO("Score Threshold: %f", score_threshold_);
    }

    if (private_node_handle.getParam("use_gpu", use_gpu_))
    {
      ROS_INFO("GPU Mode: %d", use_gpu_);
    }
    int gpu_id;
    if (private_node_handle.getParam("gpu_device_id", gpu_id))
    {
      ROS_INFO("GPU Device ID: %d", gpu_id);
      gpu_device_id_ = (unsigned int) gpu_id;
    }

    //SSD STUFF
    ssd_detector_ = new SSDDetector(network_definition_file, pretrained_model_file, pixel_mean_, use_gpu_, gpu_device_id_);

    if (NULL == ssd_detector_)
    {
      ROS_INFO("Error while creating SSD Object");
      return;
    }
    ROS_INFO("SSD Detector initialized.");

#if (CV_MAJOR_VERSION <= 2)
    cv::generateColors(colors_, 20);
#else
    generateColors(colors_, 20);
#endif

    publisher_detected_objects_ = node_handle_.advertise<autoware_msgs::DetectedObjectArray>(
      "/detection/image_detector/objects", 1);

    ROS_INFO("Subscribing to... %s", image_raw_topic_str.c_str());
    subscriber_image_raw_ = node_handle_.subscribe(image_raw_topic_str, 1, &ROSSSDApp::image_callback, this);

    std::string config_topic("/config");
    config_topic += "/ssd";
    subscriber_ssd_config_ = node_handle_.subscribe(config_topic, 1, &ROSSSDApp::config_cb, this);

    ros::spin();
    ROS_INFO("END SSD");

  }

  ~ROSSSDApp()
  {
    if (NULL != ssd_detector_)
      delete ssd_detector_;
  }

  ROSSSDApp()
  {
    ssd_detector_ 	= NULL;
    score_threshold_= 0.5;
    use_gpu_ 		= false;
    gpu_device_id_ 	= 0;
    pixel_mean_		= cv::Scalar(102.9801, 115.9465, 122.7717);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ssd_unc");

  ROSSSDApp app;

	app.Run();

  return 0;
}
