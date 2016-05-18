#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <iostream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jsk_model_marker_interface");
  ros::NodeHandle n;
  ros::NodeHandle pnh_("~");
  tf::TransformListener tfl_;

  ros::Publisher pub_ =  pnh_.advertise<tf::tfMessage> ("/specific_transform", 1);

  std::vector<std::string> parent_frame;
  std::vector<std::string> child_frame;

  {
    XmlRpc::XmlRpcValue param_val;
    pnh_.getParam("parent_frame", param_val);
    if (param_val.getType() ==  XmlRpc::XmlRpcValue::TypeArray) {
      for(int i = 0; i < param_val.size(); i++) {
        std::string st = param_val[i];
        parent_frame.push_back(st);
      }
    } else if (param_val.getType() ==  XmlRpc::XmlRpcValue::TypeString) {
      std::string st = param_val;
      parent_frame.push_back(st);
    }
  }
  {
    XmlRpc::XmlRpcValue param_val;
    pnh_.getParam("child_frame", param_val);
    if (param_val.getType() ==  XmlRpc::XmlRpcValue::TypeArray) {
      for(int i = 0; i < param_val.size(); i++) {
        std::string st = param_val[i];
        child_frame.push_back(st);
      }
    } else if (param_val.getType() ==  XmlRpc::XmlRpcValue::TypeString) {
      std::string st = param_val;
      child_frame.push_back(st);
    }
  }

  if (parent_frame.size() != child_frame.size()) {
    ROS_FATAL("size difference between parent frames and child frames");
  }

  double loop_hz;
  pnh_.param("loop_hz", loop_hz, 1.0 );

  for(int i = 0; i < parent_frame.size(); i++) {
    ROS_INFO_STREAM("parent->child: " << parent_frame[i] << " -> " << child_frame[i]);
  }
  ROS_INFO_STREAM("loop_hz:" << loop_hz);

  ros::Rate rate(loop_hz);

  while (ros::ok())
    {
      tf::tfMessage tf_msg;
      for(int i = 0; i < parent_frame.size(); i++) {
        geometry_msgs::TransformStamped tfs_msg;
        tf::StampedTransform stf;
        try {
          tfl_.lookupTransform(parent_frame[i], child_frame[i], ros::Time(0), stf);
          tf::transformStampedTFToMsg(stf, tfs_msg);
	  tf_msg.transforms.push_back(tfs_msg);
        } catch(tf::TransformException ex) {
          ROS_INFO_STREAM("missing transform: " << parent_frame[i] << " to " << child_frame[i]);
        }
      }
      pub_.publish(tf_msg);

      ros::spinOnce();
      rate.sleep();
    }
}
