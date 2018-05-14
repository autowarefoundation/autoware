
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "velodyne_pointcloud/point_types.h"
#include "velodyne_pointcloud/rawdata.h"
#include <stdio.h>
#include <stdlib.h>
#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

static std::string PARENT_FRAME;
static std::string CHILD_FRAME;
static std::string POINTS_TOPIC;
static int SCAN_NUM;
static std::string OUTPUT_DIR;

static pcl::PointCloud<velodyne_pointcloud::PointXYZIR> map;
static tf::TransformListener *tf_listener;
static std::string filename;

static int added_scan_num = 0;
static int map_id = 0;
static int count = 0;

void points_callback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &input)
{
  count++;

  pcl::PointCloud<pcl::PointXYZI> pcl_out;
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_input (new pcl::PointCloud<pcl::PointXYZI>);
  std_msgs::Header header;
  pcl_conversions::fromPCL(input->header, header);

  if(CHILD_FRAME == "velodyne" && POINTS_TOPIC == "hokuyo_3d/hokuyo_cloud2")
  {
	  /*
    tf::Vector3 v(1.7, 0, 1.5);
    tf::Quaternion q;
    q.setRPY(3.141592, 0.05, -0.05);
    tf::Transform tf_baselink_to_localizer(q, v);
    */
    tf::Vector3 v(-0.5, 0, 0.5);
    tf::Quaternion q;
    q.setRPY(3.141592, 0.05, -0.05);
    tf::Transform tf_baselink_to_localizer(q, v);
    pcl_ros::transformPointCloud(*input, *transformed_input, tf_baselink_to_localizer);
  }

  tf::StampedTransform transform;
  if(input->size() > 0)
  {
    try
    {
      tf_listener->waitForTransform(PARENT_FRAME, CHILD_FRAME, header.stamp, ros::Duration(1));
      tf_listener->lookupTransform(PARENT_FRAME, CHILD_FRAME, header.stamp, transform);
    }
    catch (tf::TransformException ex)
    {
      std::cout << "Transform not found" << std::endl;
      return;
    }

//    if(CHILD_FRAME == "gps" && POINTS_TOPIC == "hokuyo_3d/hokuyo_cloud2")
      if(CHILD_FRAME == "velodyne" && POINTS_TOPIC == "hokuyo_3d/hokuyo_cloud2")
    {
      for (int i = 0; i < (int)transformed_input->size(); i++)
      {
        tf::Point pt(transformed_input->points[i].x, transformed_input->points[i].y, transformed_input->points[i].z);
        tf::Point pt_world = transform * pt;
        pcl::PointXYZI wp;
        double distance = pt.x() * pt.x() + pt.y() * pt.y() + pt.z() * pt.z();
        if (distance < 3 * 3)
          continue;
        wp.x = pt_world.x();
        wp.y = pt_world.y();
        wp.z = pt_world.z();
        wp.intensity = transformed_input->points[i].intensity;

        pcl_out.push_back(wp);
      }
      pcl_out.header = transformed_input->header;
    } else {
      for (int i = 0; i < (int)input->size(); i++)
      {
        tf::Point pt(input->points[i].x, input->points[i].y, input->points[i].z);
        tf::Point pt_world = transform * pt;
        pcl::PointXYZI wp;
        double distance = pt.x() * pt.x() + pt.y() * pt.y() + pt.z() * pt.z();
        if (distance < 3 * 3)
          continue;
        wp.x = pt_world.x();
        wp.y = pt_world.y();
        wp.z = pt_world.z();
        wp.intensity = input->points[i].intensity;

        pcl_out.push_back(wp);
      }
      pcl_out.header = input->header;
    }

    pcl_out.header.frame_id = "map";

    // Set log file name.
    std::ofstream ofs;
    std::string lidar;
    if(POINTS_TOPIC == "points_raw")
    {
      lidar = "velodyne";
    } else if(POINTS_TOPIC == "hokuyo_3d/hokuyo_cloud2"){
      lidar = "hokuyo";
    }
    filename = OUTPUT_DIR + PARENT_FRAME + "-" + CHILD_FRAME + "_" + lidar + "_"+ std::to_string(map_id) + ".csv";
    ofs.open(filename.c_str(), std::ios::app);

    if (!ofs)
    {
      std::cerr << "Could not open " << filename << "." << std::endl;
      exit(1);
    }

    for (int i = 0; i < (int)pcl_out.points.size(); i++)
    {
      ofs << std::fixed << std::setprecision(5) << pcl_out.points[i].x << ","
          << std::fixed << std::setprecision(5) << pcl_out.points[i].y << ","
          << std::fixed << std::setprecision(5) << pcl_out.points[i].z << ","
          << pcl_out.points[i].intensity << std::endl;
    }
    std::cout << "Wrote " << pcl_out.size() << " points to " << filename << "." << std::endl;
    added_scan_num++;
    if(added_scan_num == SCAN_NUM)
    {
      added_scan_num = 0;
      map_id++;
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_mapping");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.getParam("parent_frame", PARENT_FRAME);
  private_nh.getParam("child_frame", CHILD_FRAME);
  private_nh.getParam("points_topic", POINTS_TOPIC);
  private_nh.getParam("scan_num", SCAN_NUM);
  private_nh.getParam("output_dir", OUTPUT_DIR);

  std::cout << "parent_frame: " << PARENT_FRAME << std::endl;
  std::cout << "child_frame: " << CHILD_FRAME << std::endl;
  std::cout << "points_topic: " << POINTS_TOPIC << std::endl;
  std::cout << "scan_num: " << SCAN_NUM << std::endl;
  std::cout << "output_dir: " << OUTPUT_DIR << std::endl;

  tf_listener = new tf::TransformListener();

  ros::Subscriber points_sub = nh.subscribe(POINTS_TOPIC, 10, points_callback);

  ros::spin();

  return 0;
}
