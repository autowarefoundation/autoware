#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <iostream>
#include <sstream>

#include "geo_pos_conv.hh"

double _initial_velocity = 1.4; // m/s
geometry_msgs::Twist _current_velocity;

nav_msgs::Path _current_path;

//車の座標系に変換したwaypoint
geometry_msgs::PoseStamped _transformed_waypoint;

//参照するwaypointの番号
int _next_waypoint = 0;

//グローバル座標系での現在位置
geometry_msgs::PoseStamped _current_pose;

double _lookahead_threshold = 4.0;

const std::string PATH_FRAME = "/path";
const std::string CAR_FRAME = "/base_link";

std::string _current_pose_topic;

void OdometryPoseCallback(const nav_msgs::OdometryConstPtr &msg)
{
    //std::cout << "odometry callback" << std::endl;
    _current_velocity = msg->twist.twist;

    //テスト版位置情報

    if (_current_pose_topic == "odometry") {
        _current_pose.header = msg->header;
        _current_pose.pose = msg->pose.pose;
    }else
        std::cout << "pose is not odometry" << std::endl;

}

 void GNSSCallback(const sensor_msgs::NavSatFixConstPtr &msg)
 {
 //std::cout << "gnss callback" << std::endl;
 if (_current_pose_topic == "gnss") {
 //平面直角座標への変換
        geo_pos_conv geo;
 geo.llh_to_xyz(msg->latitude, msg->longitude, msg->altitude);
  _current_pose.header = msg->header;
  _current_pose.pose.position.x = geo.x();
_current_pose.pose.position.y = geo.y();
 _current_pose.pose.position.z = geo.z();
    }else
        std::cout << "pose is not gnss" << std::endl;

 }

void AmclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    if (_current_pose_topic == "amcl") {
           _current_pose.header = msg->header;
           _current_pose.pose = msg->pose.pose;
       }else
           std::cout << "pose is not amcl" << std::endl;
}

void WayPointCallback(const nav_msgs::PathConstPtr &msg)
{
    //std::cout << "waypoint callback" << std::endl;
    _current_path = *msg;

    //std::cout << "_current_path frame_id = " << _current_path.header.frame_id
    //       << std::endl;
}

//waypoint探索
int GetNextWayPoint(const tf::TransformListener &tfListener)
{
    std::cout << "get nextwaypoint" << std::endl;

    if (_current_path.poses.empty() == false) {

        //車座標系での原点を設定
        tf::Vector3 v1(0, 0, 0);

        std::cout << "waypoint count =" << _current_path.poses.size()
                << std::endl;

        for (int i = _next_waypoint; i < _current_path.poses.size(); i++) {

            std::cout << "calculate waypoint : " << i << std::endl;

            //waypointを車の座標系に変換
            try {
                ros::Time now = ros::Time::now();
                tfListener.waitForTransform(CAR_FRAME,
                        _current_path.header.frame_id, now, ros::Duration(0.1));

                tfListener.transformPose(CAR_FRAME, _current_path.poses[i],
                        _transformed_waypoint);
                std::cout << "current path ("
                        << _current_path.poses[i].pose.position.x << " "
                        << _current_path.poses[i].pose.position.y << " "
                        << _current_path.poses[i].pose.position.z
                        << ") ---> transformed_path : ("
                        << _transformed_waypoint.pose.position.x << " "
                        << _transformed_waypoint.pose.position.y << " "
                        << _transformed_waypoint.pose.position.z << ")"
                        << std::endl;

            } catch (tf::TransformException &ex) {
                ROS_ERROR("%s", ex.what());

            }

            tf::Vector3 v2(_transformed_waypoint.pose.position.x,
                    _transformed_waypoint.pose.position.y,
                    _transformed_waypoint.pose.position.z);

            if (_transformed_waypoint.pose.position.x < 0)
                continue;

            std::cout << "distance = " << tf::tfDistance(v1, v2) << std::endl;

            if (tf::tfDistance(v1, v2) > _lookahead_threshold) {
                return i;
            }

        }
        return 0;

    } else
        std::cout << "nothing waypoint" << std::endl;
    return 0;
}

//waypointまでの直線距離を計算
double GetLookAheadDistance()
{
    //std::cout << "get lookahead distance" << std::endl;

//現在位置
    tf::Vector3 v1(_current_pose.pose.position.x, _current_pose.pose.position.y,
            _current_pose.pose.position.z);

    tf::Vector3 v2(_current_path.poses[_next_waypoint].pose.position.x,
            _current_path.poses[_next_waypoint].pose.position.y,
            _current_path.poses[_next_waypoint].pose.position.z);

    return tf::tfDistance(v1, v2);

}

//waypointまで到達するための速度を計算
geometry_msgs::Twist CalculateCmdTwist()
{
    std::cout << "calculate" << std::endl;
    geometry_msgs::Twist twist;

    double lookahead_distance = GetLookAheadDistance();

    std::cout << "Lookahead Distance = " << lookahead_distance << std::endl;

    double radius = pow(lookahead_distance, 2)
            / (2 * _transformed_waypoint.pose.position.y);

    double angular_velocity;

    if (radius > 0 || radius < 0) {
        angular_velocity = _initial_velocity / radius;
    } else {
        angular_velocity = 0;
    }

    double linear_velocity = _initial_velocity;

    twist.linear.x = linear_velocity;
    twist.angular.z = angular_velocity;

    return twist;

}

int main(int argc, char **argv)
{

    std::cout << "lane follower start" << std::endl;
// set up ros
    ros::init(argc, argv, "lane_follower");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    //setting params
    _current_pose_topic = "odometry";
    private_nh.getParam("current_pose_topic", _current_pose_topic);
    std::cout << "current_pose_topic : " << _current_pose_topic << std::endl;

//publish topic
    ros::Publisher cmd_velocity_publisher = nh.advertise<geometry_msgs::Twist>(
            "cmd_vel", 1000);

//subscribe topic
    ros::Subscriber waypoint_subcscriber = nh.subscribe("lane_waypoint", 1000,
            WayPointCallback);
    ros::Subscriber odometry_subscriber = nh.subscribe("pose", 1000,
            OdometryPoseCallback);

    ros::Subscriber gnss_subscriber = nh.subscribe("fix", 1000, GNSSCallback);

    ros::Subscriber amcl_subscriber = nh.subscribe("amcl_pose", 1000,
            AmclCallback);

    geometry_msgs::Twist twist;
    tf::TransformListener tfListener;

//5Hzでループ
    ros::Rate loop_rate(5);
    while (ros::ok()) {
        ros::spinOnce();

        //waypoint取得
        _next_waypoint = GetNextWayPoint(tfListener);
        std::cout << "nextwaypoint = " << _next_waypoint << std::endl;
        std::cout << "current_pose : (" << _current_pose.pose.position.x << " "
                << _current_pose.pose.position.y << " "
                << _current_pose.pose.position.z << ")" << std::endl;

        if (_next_waypoint > 0) {

            //速度を計算
            twist = CalculateCmdTwist();

        } else {
            twist.linear.x = 0;
            twist.angular.z = 0;
        }

        std::cout << "linear.x =" << twist.linear.x << " angular.z = "
                << twist.angular.z << std::endl << std::endl;
        cmd_velocity_publisher.publish(twist);

        loop_rate.sleep();
    }

    return 0;

}
