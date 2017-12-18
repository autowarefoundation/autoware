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

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <chrono>

#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
// #include <velodyne_pointcloud/point_types.h>
// #include <velodyne_pointcloud/rawdata.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <pcl/point_types.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <autoware_msgs/ConfigVelodyneHokuyo.h>

#include <boost/bind.hpp>

static std::string velodyne_topic = "/vh/velodyne_points";
static std::string hokuyo_topic = "/vh/scan";
static std::string output_topic = "velodyne_points";
static bool use_velodyne = true;
static bool use_hokuyo = true;

laser_geometry::LaserProjection projector;
static tf::TransformListener *tf_listener;
static tf::StampedTransform hokuyo_local_transform;
static tf::StampedTransform velodyne_local_transform;

static ros::Publisher output_pub;

static double pose_x = 0.0;
static double pose_y = 0.0;
static double pose_z = 0.0;
static double pose_roll = 0.0;
static double pose_pitch = 0.0;
static double pose_yaw = 0.0;
static double max_range = 30.0;

static ros::Time current_vel_time;
static ros::Time current_hok_time;

static pcl::PointCloud<pcl::PointXYZI> velodyne_points;
static pcl::PointCloud<pcl::PointXYZI> hokuyo_points;
static sensor_msgs::PointCloud2 cloud;

/*
void points_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    if (output_pub.getNumSubscribers() == 0)      // no one listening?
      return;                                     // avoid much work

    
    current_vel_time = msg->header.stamp;

	velodyne_points.points.clear();
    pcl::fromROSMsg(*msg, velodyne_points);
    pcl::PointCloud<pcl::PointXYZI>::Ptr velodyne_points_ptr(new pcl::PointCloud<pcl::PointXYZI>(velodyne_points));
    int vel_points_num = velodyne_points_ptr->size();

    //add the hokuyo points
    current_vel_time = msg->header.stamp;
    if (use_hokuyo && hokuyo_points.points.size()) {
        velodyne_points += hokuyo_points;
    }
    
    velodyne_points.header.frame_id = output_topic;

    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*velodyne_points_ptr, output_msg);

    output_pub.publish(output_msg);
}

void laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    if (output_pub.getNumSubscribers() == 0)      // no one listening?
      return;                                     // avoid much work

    
    tf::TransformListener hokuyo_tf_listener;
    try {
      ros::Time now = ros::Time(0);
      ros::Time scan_time =  msg->header.stamp + ros::Duration().fromSec(msg->ranges.size()*msg->time_increment);
      hokuyo_tf_listener.waitForTransform("/hokuyo", 
                                          "/velodyne", 
                                          now, 
                                          ros::Duration(1.0));
      hokuyo_tf_listener.lookupTransform("hokuyo", "/velodyne", now, hokuyo_local_transform);
    }
    catch (tf::TransformException& ex) {
      ROS_ERROR("%s", ex.what());
    }
  
    //sensor_msgs::PointCloud2 cloud;
    //sensor_msgs::PointCloud2::Ptr cloud_ptr(new sensor_msgs::PointCloud2(cloud));
    projector.transformLaserScanToPointCloud("/velodyne", *msg, cloud, hokuyo_tf_listener,
                                             laser_geometry::channel_option::Intensity|laser_geometry::channel_option::Distance);

    current_hok_time = msg->header.stamp;
    //pcl::PointCloud<pcl::PointXYZI>::Ptr hokuyo_points_ptr(new pcl::PointCloud<pcl::PointXYZI>(hokuyo_points));
    hokuyo_points.points.clear();
    pcl::fromROSMsg(cloud, hokuyo_points);
}
*/

void callback(const sensor_msgs::LaserScan::ConstPtr& laser, const sensor_msgs:: PointCloud2::ConstPtr& ptcloud)
{
    // if (output_pub.getNumSubscribers() == 0)      // no one listening?
    //   return;                                     // avoid much work

    static tf::StampedTransform vh_transform;
    /*
    try {
      ros::Time now = ros::Time::now(); //();
      ros::Time scan_time =  laser->header.stamp + ros::Duration().fromSec(laser->ranges.size()*laser->time_increment);
      tf_listener->waitForTransform(laser->header.frame_id, 
                                    ptcloud->header.frame_id, 
                                    now, 
                                    ros::Duration(2.0));
    //   tf_listener->lookupTransform(laser->header.frame_id,
    //                                ptcloud->header.frame_id, 
    //                                now, 
    //                                vh_transform);
    }
    catch (tf::TransformException& ex) {
      ROS_ERROR("%s", ex.what());
    }
    */
    ros::Time now = ros::Time::now(); //();
    ros::Time scan_time =  laser->header.stamp + ros::Duration().fromSec(laser->ranges.size()*laser->time_increment);
    //ROS_INFO_STREAM("scantime -> " << scan_time);
    if (tf_listener->waitForTransform(laser->header.frame_id, 
                                            ptcloud->header.frame_id, 
                                            ros::Time(0),//scan_time,//now, 
                                            ros::Duration(1.0))) {
        // ROS_INFO("got tf");
        // tf_listener->lookupTransform(laser->header.frame_id,
        //                              ptcloud->header.frame_id, 
        //                              now, 
        //                              vh_transform);
        //ROS_INFO("scan2ptcloud");
        projector.transformLaserScanToPointCloud(ptcloud->header.frame_id, 
                                                 *laser, 
                                                 cloud, 
                                                 *tf_listener,
                                                 laser_geometry::channel_option::Intensity|laser_geometry::channel_option::Distance);
        //add ring field
        // sensor_msgs::PointField field;
        // field.name = "ring";
        // field.count = 1;
        // field.datatype = 4;
        // field.offset = 20;
        // cloud.fields.push_back(field);

        current_hok_time = laser->header.stamp;
        hokuyo_points.points.clear();
        pcl::fromROSMsg(cloud, hokuyo_points);
    }

    current_vel_time = ptcloud->header.stamp;

	velodyne_points.points.clear();
    pcl::fromROSMsg(*ptcloud, velodyne_points);
    pcl::PointCloud<pcl::PointXYZI>::Ptr velodyne_points_ptr(new pcl::PointCloud<pcl::PointXYZI>(velodyne_points));
    int vel_points_num = velodyne_points_ptr->size();

    // for (auto f:ptcloud->fields) {
    //     ROS_INFO_STREAM("vel fields>> name: " << f.name << " count: " << f.count << " offset: " << f.offset << " datatype: " << (int)f.datatype);
    // }

    // for (auto f:cloud.fields) {
    //     ROS_INFO_STREAM("hok fields>> name: " << f.name << " count: " << f.count << " offset: " << f.offset << " datatype: " << (int)f.datatype);
    // }

    //add the hokuyo points
    current_vel_time = ptcloud->header.stamp;
    if (use_hokuyo && hokuyo_points.points.size()) {
        //velodyne_points += hokuyo_points;
        for (int i = 0; i < hokuyo_points.points.size(); i++) {
            if (laser->ranges[i] <= max_range) {
                auto pt = hokuyo_points.points[i];
                pt.intensity = (pt.intensity / 10000) * 255; //Velodyne is in the 0..255 range
                //velodyne_points.points.push_back(pt);
                velodyne_points_ptr->push_back(pt);
                //ROS_INFO_STREAM(pt.x << "," << pt.y << "," << pt.z);
            }
        }
    }

    velodyne_points.header.frame_id = output_topic;

    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*velodyne_points_ptr, output_msg);

    output_pub.publish(output_msg);
}

/*static void param_callback(const autoware_msgs::ConfigVelodyneHokuyo::ConstPtr& input)
{
    pose_x = input->x;
    pose_y = input->y;
    pose_z = input->z;
    pose_roll = input->roll;
    pose_pitch = input->pitch;
    pose_yaw = input->yaw;
}*/

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "velodyne_hokuyo");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.getParam("velodyne_topic", velodyne_topic);
    private_nh.getParam("hokuyo_topic", hokuyo_topic);
    private_nh.getParam("output_topic", output_topic);
    private_nh.getParam("use_velodyne", use_velodyne);
    private_nh.getParam("use_hokuyo", use_hokuyo);
    private_nh.getParam("max_range", max_range);

    tf_listener = new tf::TransformListener();

    //Subscribers
    message_filters::Subscriber<sensor_msgs::LaserScan> hokuyo_sub(nh, hokuyo_topic, 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> velodyne_sub(nh, velodyne_topic, 10);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::PointCloud2> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), hokuyo_sub, velodyne_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));


    //Advertisers
    if (use_hokuyo || use_velodyne) {
        output_pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic, 10);
    }

    //spin
    ros::spin();
}
