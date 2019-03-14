#include <vector>
#include <iostream>
#include <limits>
#include <chrono>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf2/utils.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/LinkStates.h>

class GazeboTwistCalculator
{
public:
  GazeboTwistCalculator()
  {
    pub_twist_ = nh_.advertise<geometry_msgs::Twist>("/ideal_twist", 1);
    pub_pose_ = nh_.advertise<geometry_msgs::Pose>("/ideal_pose", 1);
    sub_gazebo_status_ = nh_.subscribe("/gazebo/link_states", 1, &GazeboTwistCalculator::callbackGazeboLinkStatus, this);
    timer_ = nh_.createTimer(ros::Duration(0.1), &GazeboTwistCalculator::timerCallback, this);
  };
  ~GazeboTwistCalculator(){};

private:
  ros::NodeHandle nh_, pnh_;
  ros::Publisher pub_twist_, pub_pose_;
  ros::Subscriber sub_gazebo_status_;
  ros::Timer timer_;
  geometry_msgs::Pose pose_;
  geometry_msgs::Twist twist_;

  void callbackGazeboLinkStatus(const gazebo_msgs::LinkStates &msg)
  {
    int num = 146;
    pose_ = msg.pose[num];
    twist_ = msg.twist[num];
  }

  void timerCallback(const ros::TimerEvent &te)
  {

    pub_pose_.publish(pose_);
    double yaw = tf2::getYaw(pose_.orientation);

    geometry_msgs::Twist twist;
    twist.linear.x = twist_.linear.x * std::cos(-yaw) - twist_.linear.y * std::sin(-yaw);
    twist.linear.y = twist_.linear.x * std::sin(-yaw) + twist_.linear.y * std::cos(-yaw);
    twist.angular.z = twist_.angular.z;
    pub_twist_.publish(twist);

    ROS_INFO("a");
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gazebo_twist_calculator");
  GazeboTwistCalculator obj;
  ros::spin();
  return 0;
};
