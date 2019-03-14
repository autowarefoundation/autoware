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

#include <ros/ros.h>
#include <autoware_msgs/ImageObjTracked.h>
#include <autoware_msgs/ImageObjRanged.h>

class DummyTrack{
public:
  DummyTrack(){
    subscriber_image_obj_ = node_handle_.subscribe("image_obj_ranged", 1, &DummyTrack::detections_callback, this);
		publisher_tracked_objects_ = node_handle_.advertise<autoware_msgs::ImageObjTracked>("image_obj_tracked", 1);
  }
  void run(){
    ros::spin();
  }
private:
  ros::Subscriber 	subscriber_image_obj_;
  ros::Publisher 		publisher_tracked_objects_;//ROS
  ros::NodeHandle 	node_handle_;

  void detections_callback(autoware_msgs::ImageObjRanged image_objects_msg)
  {
    autoware_msgs::ImageObjTracked pub_msg;
    pub_msg.header = image_objects_msg.header;
    pub_msg.type = image_objects_msg.type;
    pub_msg.rect_ranged = image_objects_msg.obj;
    pub_msg.total_num = image_objects_msg.obj.size();
    for (int i = 0; i < pub_msg.total_num; ++i) {
      pub_msg.obj_id.push_back(i);
      pub_msg.real_data.push_back(0);
      pub_msg.lifespan.push_back(45);
    }
    publisher_tracked_objects_.publish(pub_msg);
  }
};

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "dummy_track");
  DummyTrack dummy_track;
  dummy_track.run();

	return 0;
}
