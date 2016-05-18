#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <jsk_topic_tools/specific_transform_subscriber.h>
#include <dynamic_tf_publisher/SetDynamicTF.h>

void CallSetDynamicTf(std::string parent_frame_id, std::string frame_id, geometry_msgs::Transform transform){
  dynamic_tf_publisher::SetDynamicTF SetTf;
  SetTf.request.freq = 10;
  SetTf.request.cur_tf.header.stamp = ros::Time::now();
  SetTf.request.cur_tf.header.frame_id = parent_frame_id;
  SetTf.request.cur_tf.child_frame_id = frame_id;
  SetTf.request.cur_tf.transform = transform;
  dynamic_tf_publisher_client.call(SetTf);
}


void transformCallback(const tf::tfMessage::ConstPtr& msg){
  for(int i=0; i<msg->transforms.size(); i++){
    CallSetDynamicTf(msg->transforms[i].header.frame_id, msg->transforms[i].child_frame_id, msg->transforms[i].transform);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "specific_transform_subscriber");

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_("~");

  ros::Subscriber sub_ =  pnh_.subscribe<tf::tfMessage>
    ("/specific_transform", 100, transformCallback);

  dynamic_tf_publisher_client = nh_.serviceClient<dynamic_tf_publisher::SetDynamicTF>("set_dynamic_tf");
  ros::service::waitForService("set_dynamic_tf", -1);
  
  ros::spin();
}
