#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>

#include <iostream>

#include "geo_pos_conv.hh"

ros::Publisher pose_publisher;

geometry_msgs::PoseStamped _prev_pose;
geometry_msgs::Quaternion _quat;
double yaw;

void GNSSCallback(const sensor_msgs::NavSatFixConstPtr &msg)
{

    geo_pos_conv geo;

    geo.set_plane(7);
    geo.llh_to_xyz(msg->latitude, msg->longitude, msg->altitude);

    static tf::TransformBroadcaster pose_broadcaster;
    tf::Transform pose_transform;
    tf::Quaternion pose_q;

    pose_transform.setOrigin(tf::Vector3(0, 0, 0));
    pose_q.setRPY(0, 0, 0);
    pose_transform.setRotation(pose_q);
    pose_broadcaster.sendTransform(
            tf::StampedTransform(pose_transform, ros::Time::now(), "map",
                    "gps_frame"));

    geometry_msgs::PoseStamped pose;
    pose.header = msg->header;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "gps";
    pose.pose.position.x = geo.y();
    pose.pose.position.y = geo.x();
    pose.pose.position.z = geo.z();

    double distance = sqrt(
            pow(pose.pose.position.y - _prev_pose.pose.position.y, 2)
                    + pow(pose.pose.position.x - _prev_pose.pose.position.x,
                            2));
    std::cout << "distance : " << distance << std::endl;

    if (distance > 0.2) {
        yaw = atan2(pose.pose.position.y - _prev_pose.pose.position.y,
                pose.pose.position.x - _prev_pose.pose.position.x);
        _quat = tf::createQuaternionMsgFromYaw(yaw);
        _prev_pose = pose;
    }

    pose.pose.orientation = _quat;
    pose_publisher.publish(pose);

    //座標変換
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(
            tf::Vector3(pose.pose.position.x, pose.pose.position.y,
                    pose.pose.position.z));
    q.setRPY(0, 0, yaw);
    transform.setRotation(q);
    br.sendTransform(
            tf::StampedTransform(transform, ros::Time::now(), "map",
                    "gps"));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gnss_pos");
    ros::NodeHandle nh;
    pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("gnss_pose",
            1000);
    ros::Subscriber gnss_pose_subscriber = nh.subscribe("fix", 100,
            GNSSCallback);

    ros::spin();
    return 0;
}
