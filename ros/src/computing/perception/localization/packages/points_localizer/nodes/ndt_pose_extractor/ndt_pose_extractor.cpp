#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

int main (int argc, char* argv[])
{
  ros::init(argc, argv, "ndt_pose_extractor");

  ros::NodeHandle node;

  ros::Publisher ndt_pose_pub = node.advertise<geometry_msgs::PoseStamped> ("/ndt_pose", 1000);

  tf::TransformListener listener;
  ros::Rate rate(10.0);
  while (ros::ok())
    {
      tf::StampedTransform transform;
      try {
        //listener.lookupTransform("velodyne", "map", ros::Time(0), transform);
        listener.lookupTransform("map", "velodyne", ros::Time(0), transform);
      }
      catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
      }


      geometry_msgs::PoseStamped ndt_pose_msg;
      ndt_pose_msg.header.frame_id = "/map";
      ndt_pose_msg.header.stamp = transform.stamp_;
      ndt_pose_msg.pose.position.x = transform.getOrigin().x();
      ndt_pose_msg.pose.position.y = transform.getOrigin().y();
      ndt_pose_msg.pose.position.z = transform.getOrigin().z();
      ndt_pose_msg.pose.orientation.x = transform.getRotation().x();
      ndt_pose_msg.pose.orientation.y = transform.getRotation().y();
      ndt_pose_msg.pose.orientation.z = transform.getRotation().z();
      ndt_pose_msg.pose.orientation.w = transform.getRotation().w();

      ndt_pose_pub.publish(ndt_pose_msg);
      rate.sleep();
    }



}
