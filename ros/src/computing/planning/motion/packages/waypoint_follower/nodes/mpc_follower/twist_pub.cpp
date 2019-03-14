#include <vector>
#include <iostream>
#include <limits>
#include <chrono>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>

#include <geometry_msgs/TwistStamped.h>

class TwistPub
{
  public:
    TwistPub()
    {
        pub_twist_ = nh_.advertise<geometry_msgs::TwistStamped>("/twist_raw", 1);
        timer_ = nh_.createTimer(ros::Duration(0.1), &TwistPub::timerCallback, this);
    };
    ~TwistPub(){};

  private:
    ros::NodeHandle nh_, pnh_;
    ros::Publisher pub_twist_;
    ros::Timer timer_;

    void timerCallback(const ros::TimerEvent &te)
    {
        double hz = 0.3;
        double a = 0.2;
        double b = 0.15;
        double t = ros::Time::now().toSec();

        geometry_msgs::TwistStamped twist;
        twist.twist.linear.x = 2.0;
        twist.twist.angular.z = b + a * std::sin(hz * 3.1415 * t);
        pub_twist_.publish(twist);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "twist_pub");
    TwistPub obj;
    ros::spin();
    return 0;
};
