#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

#include <iostream>
#include <fstream>
#include <vector>

struct pose {
    double x;
    double y;
  double z;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_lane_navigator");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ros::Publisher navigation_pub = nh.advertise<nav_msgs::Path>(
            "lane_waypoint", 1000);

    std::vector<pose> Pose;

    std::string filename = "";
    if (private_nh.getParam("filename", filename) == false) {
        std::cout << "error! usage : rosrun lane_follower test_lane_navigator _filename:=\"[path file]\"" << std::endl;
        exit(-1);
    }
    std::cout << "filename : " << filename <<  std::endl;
    std::ifstream ifs(filename.c_str());
    std::string str;
    while (getline(ifs, str)) {
        std::cout << str << std::endl;
        pose test_pose;
        sscanf(str.c_str(), "%lf,%lf,%lf", &test_pose.x, &test_pose.y,&test_pose.z);
        //std::cout <<test_pose.x << " "<< test_pose.y << " "<<  std::endl;
        Pose.push_back(test_pose);
    }
    ros::Rate loop_rate(10);
    while (ros::ok()) {

        nav_msgs::Path cmd_path;
        cmd_path.header.frame_id = "path";
        cmd_path.header.stamp = ros::Time::now();
        for (int i = 0; i < Pose.size(); i++) {

            geometry_msgs::PoseStamped posestamped;
            posestamped.header = cmd_path.header;
            //      geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(
            //      Pose[i].theta);
            //std::cout << Pose[i].x <<  " " << Pose[i].y << std::endl;
            posestamped.pose.position.x = Pose[i].x;
            posestamped.pose.position.y = Pose[i].y;
     posestamped.pose.position.z = Pose[i].z;
            posestamped.pose.orientation.w = 1.0;

            std::cout << posestamped.pose.position.x << " "
                    << posestamped.pose.position.y << " " << posestamped.pose.position.z <<std::endl;
            cmd_path.poses.push_back(posestamped);
        }

        navigation_pub.publish(cmd_path);
        ros::spinOnce();
        loop_rate.sleep();

    }

}
