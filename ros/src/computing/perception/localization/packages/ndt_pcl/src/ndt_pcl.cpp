/*
  Localization program using Normal Distributions Transform

  Yuki KITSUKAWA
*/

#define DEBUG 0

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/rawdata.h>

#include <tf/transform_broadcaster.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>

#define INITIAL_X -14771
#define INITIAL_Y -84757
#define INITIAL_Z 39.8
#define INITIAL_ROLL 0
#define INITIAL_PITCH 0
#define INITIAL_YAW 2.324

typedef struct{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
}Position;

// global variables
Position previous_pos, guess_pos, current_pos;
double offset_x, offset_y, offset_z, offset_yaw; // current_pos - previous_pos

// Can't load if typed "pcl::PointCloud<pcl::PointXYZRGB> map, add;"
pcl::PointCloud<pcl::PointXYZ> map, add;
pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr;

// If the map is loaded, the flag will be 1.
int flag = 0;

pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
int iter = 30; // Maximum iterations
float ndt_res = 1.0; // Resolution
double step_size = 0.1; // Step size
double trans_eps = 0.01; // Transformation epsilon

ros::Time callback_start, callback_end, t1_start, t1_end, t2_start, t2_end, t3_start, t3_end, t4_start, t4_end, t5_start, t5_end, t6_start, t6_end;
ros::Duration d_callback, d1, d2, d3, d4, d5, d6;

ros::Publisher pose_pub;
geometry_msgs::PoseStamped pose_msg;

void map_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  if(flag == 0){
    std::cout << "Loading map data..." << std::endl;
    map.header.frame_id = "/pointcloud_map_frame";

    // Convert the data type(from sensor_msgs to pcl).
    pcl::fromROSMsg(*input, map);

    // Setting parameters
    ndt.setMaximumIterations(iter);
    ndt.setResolution(ndt_res);
    ndt.setStepSize(step_size);
    ndt.setTransformationEpsilon(trans_eps);

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr (new pcl::PointCloud<pcl::PointXYZ>(map));
    // Setting point cloud to be aligned to.
    ndt.setInputTarget(map_ptr);

    // Setting position and posture for the fist time.
    previous_pos.x = INITIAL_X;
    previous_pos.y = INITIAL_Y;
    previous_pos.z = INITIAL_Z;
    previous_pos.roll = INITIAL_ROLL;
    previous_pos.pitch = INITIAL_PITCH;
    previous_pos.yaw = INITIAL_YAW;
    
    current_pos.x = INITIAL_X;
    current_pos.y = INITIAL_Y;
    current_pos.z = INITIAL_Z;
    current_pos.roll = INITIAL_ROLL;
    current_pos.pitch = INITIAL_PITCH;
    current_pos.yaw = INITIAL_YAW;

    std::cout << "NDT ready..." << std::endl;

    flag = 1;
  }
}

void velodyne_callback(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr& input)
{
  callback_start = ros::Time::now();

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;

  // 1 scan
  pcl::PointCloud<pcl::PointXYZ> scan;
  pcl::PointXYZ p;

  scan.header = input->header;
  scan.header.frame_id = "velodyne_scan_frame";

  t1_start = ros::Time::now();
  for(pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::const_iterator item = input->begin(); item != input->end(); item++)
    {
      p.x = (double)item->x;
      p.y = (double)item->y;
      p.z = (double)item->z;

      scan.points.push_back(p);
    }
  t1_end = ros::Time::now();
  d1 = t1_end - t1_start;

  Eigen::Matrix4f t(Eigen::Matrix4f::Identity());

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan_ptr (new pcl::PointCloud<pcl::PointXYZ>(scan));
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_ptr (new pcl::PointCloud<pcl::PointXYZ>);

  // Downsampling the velodyne scan using VoxelGrid filter
  t2_start = ros::Time::now();
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(1.0, 1.0, 1.0);
  voxel_grid_filter.setInputCloud(scan_ptr);
  voxel_grid_filter.filter(*filtered_scan_ptr);
  t2_end = ros::Time::now();
  d2 = t2_end - t2_start;

  // Setting point cloud to be aligned.
  ndt.setInputSource(filtered_scan_ptr);

  // Guess the initial gross estimation of the transformation
  t3_start = ros::Time::now();
  tf::Matrix3x3 init_rotation;

  guess_pos.x = previous_pos.x + offset_x;
  guess_pos.y = previous_pos.y + offset_y;
  guess_pos.z = previous_pos.z + offset_z;
  guess_pos.roll = previous_pos.roll;
  guess_pos.pitch = previous_pos.pitch;
  guess_pos.yaw = previous_pos.yaw + offset_yaw;

  Eigen::AngleAxisf init_rotation_x(guess_pos.roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(guess_pos.pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(guess_pos.yaw, Eigen::Vector3f::UnitZ());

  Eigen::Translation3f init_translation(guess_pos.x, guess_pos.y, guess_pos.z);

  Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

  t3_end = ros::Time::now();
  d3 = t3_end - t3_start;

  t4_start = ros::Time::now();
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  ndt.align(*output_cloud, init_guess);

  t = ndt.getFinalTransformation();
  t4_end = ros::Time::now();
  d4 = t4_end - t4_start;

  t5_start = ros::Time::now();
  /*
  tf::Vector3 origin;
  origin.setValue(static_cast<double>(t(0,3)), static_cast<double>(t(1,3)), static_cast<double>(t(2,3)));
  */

  tf::Matrix3x3 tf3d;

  tf3d.setValue(static_cast<double>(t(0,0)), static_cast<double>(t(0,1)), static_cast<double>(t(0,2)),
		static_cast<double>(t(1,0)), static_cast<double>(t(1,1)), static_cast<double>(t(1,2)),
		static_cast<double>(t(2,0)), static_cast<double>(t(2,1)), static_cast<double>(t(2,2)));

  // Update current_pos.
  current_pos.x = t(0,3);
  current_pos.y = t(1,3);
  current_pos.z = t(2,3);
  tf3d.getRPY(current_pos.roll, current_pos.pitch, current_pos.yaw, 1);

  // transform
  transform.setOrigin(tf::Vector3(current_pos.x, current_pos.y, current_pos.z));
  q.setRPY(current_pos.roll, current_pos.pitch, current_pos.yaw);
  transform.setRotation(q);

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "velodyne"));

  // publish the position
  pose_msg.header.frame_id = "/ndt_frame";
  pose_msg.pose.position.x = current_pos.x;
  pose_msg.pose.position.y = current_pos.y;
  pose_msg.pose.position.z = current_pos.z;
  pose_msg.pose.orientation.x = q.x();
  pose_msg.pose.orientation.y = q.y();
  pose_msg.pose.orientation.z = q.z();
  pose_msg.pose.orientation.w = q.w();

  pose_pub.publish(pose_msg);

  t5_end = ros::Time::now();
  d5 = t5_end - t5_start;

  // Writing position to position_log.txt
  t6_start = ros::Time::now();
  std::ofstream ofs("/home/kitsukawa/catkin_ws/position_log.txt", std::ios::app);
  if(ofs == NULL){
    std::cerr << "Could not open 'position_log.txt'." << std::endl;
    exit(1);
  }

  ofs << current_pos.x << " " << current_pos.y << " " << current_pos.z << " " << current_pos.roll << " " << current_pos.pitch << " " << current_pos.yaw << std::endl;
  t6_end = ros::Time::now();
  d6 = t6_end - t6_start;

  // Calculate the offset (curren_pos - previous_pos)
  offset_x = current_pos.x - previous_pos.x;
  offset_y = current_pos.y - previous_pos.y;
  offset_z = current_pos.z - previous_pos.z;
  offset_yaw = current_pos.yaw - previous_pos.yaw;

  // Update position and posture. current_pos -> previous_pos
  previous_pos.x = current_pos.x;
  previous_pos.y = current_pos.y;
  previous_pos.z = current_pos.z;
  previous_pos.roll = current_pos.roll;
  previous_pos.pitch = current_pos.pitch;
  previous_pos.yaw = current_pos.yaw;

  callback_end = ros::Time::now();
  d_callback = callback_end - callback_start;

  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "Sequence number: " << input->header.seq << std::endl;
  std::cout << "Number of scan points: " << scan_ptr->size() << " points." << std::endl;
  std::cout << "Number of filtered scan points: " << filtered_scan_ptr->size() << " points." << std::endl;
  std::cout << "NDT has converged: " << ndt.hasConverged() << std::endl;
  std::cout << "Fitness score: " << ndt.getFitnessScore() << std::endl;
  std::cout << "Number of iteration: " << ndt.getFinalNumIteration() << std::endl;
  std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
  std::cout << "(" << current_pos.x << ", " << current_pos.y << ", " << current_pos.z << ", " << current_pos.roll << ", " << current_pos.pitch << ", " << current_pos.yaw << ")" << std::endl;
  std::cout << "Transformation Matrix:" << std::endl;
  std::cout << t << std::endl;
  /*
  std::cout << "Duration of velodyne_callback: " << d_callback.toSec() << " secs." << std::endl;
  std::cout << "Adding scan points: " << d1.toSec() << " secs." << std::endl;
  std::cout << "VoxelGrid Filter: " << d2.toSec() << " secs." << std::endl;
  std::cout << "Guessing the initial gross estimation: " << d3.toSec() << " secs." << std::endl;
  std::cout << "NDT: " << d4.toSec() << " secs." << std::endl;
  std::cout << "tf: " << d5.toSec() << " secs." << std::endl;
  std::cout << "Writing position to file: " << d6.toSec() << " secs." << std::endl;
  */
  std::cout << "-----------------------------------------------------------------" << std::endl;

}

int main(int argc, char **argv)
{

  std::cout << "---------------------------------------" << std::endl; 
  std::cout << "NDT_PCL program coded by Yuki KITSUKAWA" << std::endl;
  std::cout << "---------------------------------------" << std::endl; 

  ros::init(argc, argv, "ndt_pcl");
  ros::NodeHandle n;

  pose_pub = n.advertise<geometry_msgs::PoseStamped>("/ndt_pose", 1000);

  // subscribing map data (only once)
  ros::Subscriber map_sub = n.subscribe("points_map", 10, map_callback);

  // subscribing the velodyne data
  ros::Subscriber velodyne_sub = n.subscribe("velodyne_points", 1000, velodyne_callback);

  ros::spin();

  return 0;
}
