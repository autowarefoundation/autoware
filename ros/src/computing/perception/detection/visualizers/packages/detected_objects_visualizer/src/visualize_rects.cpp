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
 *
 ********************
 *  v1.0: amc-nu (abrahammonrroy@yahoo.com)
 */

#include "visualize_rects.h"

VisualizeRects::VisualizeRects()
{
  ros::NodeHandle private_nh_("~");

  ros::NodeHandle nh;

  std::string image_src_topic;
  std::string object_src_topic;
  std::string image_out_topic;

  private_nh_.param<std::string>("image_src", image_src_topic, "/image_raw");
  private_nh_.param<std::string>("object_src", object_src_topic, "/detection/image_detector/objects");
  private_nh_.param<std::string>("image_out", image_out_topic, "/image_rects");

  //get namespace from topic
  std::string ros_namespace = image_src_topic;
  std::size_t found_pos = ros_namespace.rfind("/");//find last / from topic name to extract namespace
  std::cout << ros_namespace << std::endl;
  if (found_pos!=std::string::npos)
    ros_namespace.erase(found_pos, ros_namespace.length()-found_pos);
  std::cout << ros_namespace << std::endl;
  image_out_topic = ros_namespace + image_out_topic;

  image_filter_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image>(private_nh_,
                                                                                 image_src_topic,
                                                                                             1);
  ROS_INFO("[%s] image_src: %s", __APP_NAME__, image_src_topic.c_str());
  detection_filter_subscriber_ = new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>(private_nh_,
                                                                                                     object_src_topic,
                                                                                             1);
  ROS_INFO("[%s] object_src: %s", __APP_NAME__, object_src_topic.c_str());

  detections_synchronizer_ =
    new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(10),
                                                   *image_filter_subscriber_,
                                                   *detection_filter_subscriber_);
  detections_synchronizer_->registerCallback(
    boost::bind(&VisualizeRects::SyncedDetectionsCallback, this, _1, _2));


  publisher_image_ = node_handle_.advertise<sensor_msgs::Image>(
    image_out_topic, 1);
  ROS_INFO("[%s] image_out: %s", __APP_NAME__, image_out_topic.c_str());

}

void
VisualizeRects::SyncedDetectionsCallback(
  const sensor_msgs::Image::ConstPtr &in_image_msg,
  const autoware_msgs::DetectedObjectArray::ConstPtr &in_objects)
{
  try
  {
    image_ = cv_bridge::toCvShare(in_image_msg, "bgr8")->image;
    cv::Mat drawn_image;
    drawn_image = ObjectsToRects(image_, in_objects);
    sensor_msgs::ImagePtr drawn_msg = cv_bridge::CvImage(in_image_msg->header, "bgr8", drawn_image).toImageMsg();
    publisher_image_.publish(drawn_msg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("[%s] Could not convert from '%s' to 'bgr8'.", __APP_NAME__, in_image_msg->encoding.c_str());
  }
}

cv::Mat
VisualizeRects::ObjectsToRects(cv::Mat in_image, const autoware_msgs::DetectedObjectArray::ConstPtr &in_objects)
{
  cv::Mat final_image = in_image.clone();
  for (auto const &object: in_objects->objects)
  {
    if (IsObjectValid(object))
    {
      cv::Rect rect;
      rect.x = object.x;
      rect.y = object.y;
      rect.width = object.width;
      rect.height = object.height;

      if (rect.x+rect.width >= in_image.cols)
        rect.width = in_image.cols -rect.x - 1;

      if (rect.y+rect.height >= in_image.rows)
        rect.height = in_image.rows -rect.y - 1;

      //draw rectangle
      cv::rectangle(final_image,
                    rect,
                    cv::Scalar(244,134,66),
                    4,
                    CV_AA);

      //draw label
      std::string label = "";
      if (!object.label.empty() && object.label != "unknown")
      {
        label = object.label;
      }
      int font_face = cv::FONT_HERSHEY_DUPLEX;
      double font_scale = 1.5;
      int thickness = 1;

      int baseline=0;
      cv::Size text_size = cv::getTextSize(label,
                                          font_face,
                                          font_scale,
                                          thickness,
                                          &baseline);
      baseline += thickness;

      cv::Point text_origin(object.x - text_size.height,object.y);

      cv::rectangle(final_image,
                    text_origin + cv::Point(0, baseline),
                    text_origin + cv::Point(text_size.width, -text_size.height),
                    cv::Scalar(0,0,0),
                    CV_FILLED,
                    CV_AA,
                    0);

      cv::putText(final_image,
                  label,
                  text_origin,
                  font_face,
                  font_scale,
                  cv::Scalar::all(255),
                  thickness,
                  CV_AA,
                  false);

    }
  }
  return final_image;
}//ObjectsToBoxes

bool VisualizeRects::IsObjectValid(const autoware_msgs::DetectedObject &in_object)
{
  if (!in_object.valid ||
      in_object.width < 0 ||
      in_object.height < 0 ||
      in_object.x < 0 ||
      in_object.y < 0
    )
  {
    return false;
  }
  return true;
}//end IsObjectValid