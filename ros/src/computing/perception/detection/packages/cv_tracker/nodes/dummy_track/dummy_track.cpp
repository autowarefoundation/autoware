/*
 *  Copyright (c) 2015, Nagoya University
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
*/

#include <ros/ros.h>
#include <autoware_msgs/image_obj_tracked.h>
#include <autoware_msgs/image_obj_ranged.h>

class DummyTrack{
public:
  DummyTrack(){
    subscriber_image_obj_ = node_handle_.subscribe("image_obj_ranged", 1, &DummyTrack::detections_callback, this);
		publisher_tracked_objects_ = node_handle_.advertise<autoware_msgs::image_obj_tracked>("image_obj_tracked", 1);
  }
  void run(){
    ros::spin();
  }
private:
  ros::Subscriber 	subscriber_image_obj_;
  ros::Publisher 		publisher_tracked_objects_;//ROS
  ros::NodeHandle 	node_handle_;

  void detections_callback(autoware_msgs::image_obj_ranged image_objects_msg)
  {
    autoware_msgs::image_obj_tracked pub_msg;
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
