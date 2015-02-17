#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <lane_follower/lane.h>

#include <iostream>
#include <fstream>
#include <vector>

struct pose {
    double x;
    double y;
    double z;
    double velocity_kmh;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_lane_navigator");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ros::Publisher navigation_pub[2];
    navigation_pub[0] = nh.advertise<nav_msgs::Path>("lane_waypoint", 100);
    navigation_pub[1] = nh.advertise<lane_follower::lane>("ruled_waypoint", 100);

    std::vector<pose> Pose;

    std::string filename = "";
    if (private_nh.getParam("filename", filename) == false) {
        std::cout << "error! usage : rosrun lane_follower test_lane_navigator _filename:=\"[path file]\"" << std::endl;
        exit(-1);
    }
    std::cout << "filename : " << filename << std::endl;
    std::ifstream ifs(filename.c_str());
    std::string str;
    while (getline(ifs, str)) {
        std::cout << str << std::endl;
        pose test_pose;
        sscanf(str.c_str(), "%lf,%lf,%lf,%lf", &test_pose.x, &test_pose.y, &test_pose.z, &test_pose.velocity_kmh);
        //std::cout <<test_pose.x << " "<< test_pose.y << " "<<  std::endl;
        Pose.push_back(test_pose);
    }
    ros::Rate loop_rate(1);
    int count = 0;
    while (ros::ok()) {
        count ++;
        std::cout << "loop : " << count << std::endl;
        ros::Time now = ros::Time::now();
        nav_msgs::Path cmd_path;
        cmd_path.header.frame_id = "path";
        cmd_path.header.stamp = now;

        lane_follower::lane lane_cmd;
        lane_cmd.header.frame_id = "path";
        lane_cmd.header.stamp = now;

        for (int i = 0; i < Pose.size(); i++) {

            //Path用
            geometry_msgs::PoseStamped posestamped;
            posestamped.header = cmd_path.header;
            //      geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(
            //      Pose[i].theta);
            //std::cout << Pose[i].x <<  " " << Pose[i].y << std::endl;
            posestamped.pose.position.x = Pose[i].x;
            posestamped.pose.position.y = Pose[i].y;
            posestamped.pose.position.z = Pose[i].z;
            posestamped.pose.orientation.w = 1.0;

            std::cout << posestamped.pose.position.x << " " << posestamped.pose.position.y << " " << posestamped.pose.position.z << std::endl;
            cmd_path.poses.push_back(posestamped);

            //Ruled用
            lane_follower::waypoint waypoint;
            waypoint.pose.header = lane_cmd.header;
            waypoint.twist.header = lane_cmd.header;
            waypoint.pose.pose.position.x = Pose[i].x;
            waypoint.pose.pose.position.y = Pose[i].y;
            waypoint.pose.pose.position.z = Pose[i].z;
            waypoint.pose.pose.orientation.w = 1.0;
            waypoint.twist.twist.linear.x = Pose[i].velocity_kmh /3.6;

            std::cout << waypoint.pose.pose.position.x << " " << waypoint.pose.pose.position.y << " " << waypoint.pose.pose.position.z << " " << waypoint.twist.twist.linear.x << std::endl;
            lane_cmd.waypoints.push_back(waypoint);
        }
        navigation_pub[0].publish(cmd_path);
        navigation_pub[1].publish(lane_cmd);

        ros::spinOnce();
        loop_rate.sleep();

    }

}
