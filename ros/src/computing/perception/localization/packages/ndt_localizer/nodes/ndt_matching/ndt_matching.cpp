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

/*
 Localization program using Normal Distributions Transform

 Yuki KITSUKAWA
 */

// #define OUTPUT 

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <chrono>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/rawdata.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>

#include <runtime_manager/ConfigNdt.h>

#include <ndt_localizer/ndt_stat.h>

#define THRESHOLD_EXE_TIME 100.0
#define THRESHOLD_ITERATION 10
#define THRESHOLD_SCORE 100.0
#define THRESHOLD_VELOCITY 100.0
#define THRESHOLD_ACCELERATION 10.0

struct pose {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

static pose initial_pose, predict_pose, previous_pose, ndt_pose, current_pose, control_pose, previous_gnss_pose, current_gnss_pose;

static double offset_x, offset_y, offset_z, offset_yaw; // current_pos - previous_pose

//Can't load if typed "pcl::PointCloud<pcl::PointXYZRGB> map, add;"
static pcl::PointCloud<pcl::PointXYZ> map, add;
static pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr;

// If the map is loaded, map_loaded will be 1.
static int map_loaded = 0;
static int _use_gnss = 1;
static int init_pos_set = 0;

static pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
// Default values
static int iter = 30; // Maximum iterations
static float ndt_res = 1.0; // Resolution
static double step_size = 0.1; // Step size
static double trans_eps = 0.01; // Transformation epsilon

// Leaf size of VoxelGrid filter.
static double voxel_leaf_size = 2.0;

static ros::Publisher predict_pose_pub;
static geometry_msgs::PoseStamped predict_pose_msg;

static ros::Publisher ndt_pose_pub;
static geometry_msgs::PoseStamped ndt_pose_msg;

static ros::Publisher current_pose_pub;
static geometry_msgs::PoseStamped current_pose_msg;

static ros::Publisher control_pose_pub;
static geometry_msgs::PoseStamped control_pose_msg;

static ros::Publisher estimate_twist_pub;
static geometry_msgs::TwistStamped estimate_twist_msg;

static double angle = 0.0;
static double control_shift_x = 0.0;
static double control_shift_y = 0.0;
static double control_shift_z = 0.0;

static int max = 63;
static int min = 0;
static int layer = 1;

static ros::Publisher fitness_score_pub;
static std_msgs::Float32 fitness_score;

static ros::Publisher ndt_stat_pub;
static std_msgs::Bool ndt_stat_msg;

static ros::Time current_scan_time;
static ros::Time previous_scan_time;
static ros::Duration scan_duration;

static double current_velocity = 0.0, previous_velocity = 0.0; // [m/s]
static double current_acceleration = 0.0, previous_acceleration = 0.0; // [m/s^2]

static double angular_velocity = 0.0;

static ros::Publisher estimated_vel_mps_pub, estimated_vel_kmph_pub;
static std_msgs::Float32 estimated_vel_mps, estimated_vel_kmph, previous_estimated_vel_kmph;

static std::chrono::time_point<std::chrono::system_clock> matching_start, matching_end;;
static ros::Publisher time_ndt_matching_pub;
static std_msgs::Float32 time_ndt_matching;

static std::string _scanner = "velodyne";
static int _queue_size = 1000;

static ros::Publisher velodyne_points_filtered_pub;

static ros::Publisher ndt_stat_pub_;
static ndt_localizer::ndt_stat ndt_stat_msg_;

static void param_callback(const runtime_manager::ConfigNdt::ConstPtr& input)
{
    if (_use_gnss != input->init_pos_gnss) {
        init_pos_set = 0;
    } else if (_use_gnss == 0 && (initial_pose.x != input->x || initial_pose.y != input->y || initial_pose.z != input->z || initial_pose.roll != input->roll || initial_pose.pitch != input->pitch || initial_pose.yaw != input->yaw)) {
        init_pos_set = 0;
    }

    _use_gnss = input->init_pos_gnss;

    ndt_res = input->resolution;
    step_size = input->step_size;
    trans_eps = input->trans_esp;

    voxel_leaf_size = input->leaf_size;

    // Setting parameters
    ndt.setMaximumIterations(iter);
    ndt.setResolution(ndt_res);
    ndt.setStepSize(step_size);
    ndt.setTransformationEpsilon(trans_eps);

    /*
    std::cout << "--- Parameters ---" << std::endl;
    std::cout << "ndt_res: " << ndt_res << std::endl;
    std::cout << "step_size: " << step_size << std::endl;
    std::cout << "trans_eps: " << trans_eps << std::endl;
    std::cout << "voxel_leaf_size: " << voxel_leaf_size << std::endl;
    */

    if (_use_gnss == 0 && init_pos_set == 0) {
        initial_pose.x = input->x;
        initial_pose.y = input->y;
        initial_pose.z = input->z;
        initial_pose.roll = input->roll;
        initial_pose.pitch = input->pitch;
        initial_pose.yaw = input->yaw;
        // Setting position and posture for the first time.
        previous_pose.x = initial_pose.x;
        previous_pose.y = initial_pose.y;
        previous_pose.z = initial_pose.z;
        previous_pose.roll = initial_pose.roll;
        previous_pose.pitch = initial_pose.pitch;
        previous_pose.yaw = initial_pose.yaw;
        current_pose.x = initial_pose.x;
        current_pose.y = initial_pose.y;
        current_pose.z = initial_pose.z;
        current_pose.roll = initial_pose.roll;
        current_pose.pitch = initial_pose.pitch;
        current_pose.yaw = initial_pose.yaw;

        init_pos_set = 1;
	/*
        std::cout << "--- Initial Position (static) ---" << std::endl;
        std::cout << "initial_x: " << initial_x << std::endl;
        std::cout << "initial_y: " << initial_y << std::endl;
        std::cout << "initial_z: " << initial_z << std::endl;
        std::cout << "initial_roll: " << initial_roll << std::endl;
        std::cout << "initial_pitch: " << initial_pitch << std::endl;
        std::cout << "initial_yaw: " << initial_yaw << std::endl;
        std::cout << "NDT ready..." << std::endl;
	*/
    }

    angle = input->angle_error;
    control_shift_x = input->shift_x;
    control_shift_y = input->shift_y;
    control_shift_z = input->shift_z;

    max = input->max;
    min = input->min;
    layer = input->layer;

    /*
    std::cout << "angle_error: " << angle << "." << std::endl;
    std::cout << "control_shift_x: " << control_shift_x << "." << std::endl;
    std::cout << "control_shift_y: " << control_shift_y << "." << std::endl;
    std::cout << "control_shift_z: " << control_shift_z << "." << std::endl;
    */
}

static void map_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  if (map_loaded == 0) {
    // Convert the data type(from sensor_msgs to pcl).
    pcl::fromROSMsg(*input, map);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZ>(map));
    // Setting point cloud to be aligned to.
    ndt.setInputTarget(map_ptr);
    
    // Setting NDT parameters to default values
    ndt.setMaximumIterations(iter);
    ndt.setResolution(ndt_res);
    ndt.setStepSize(step_size);
    ndt.setTransformationEpsilon(trans_eps);
    
    map_loaded = 1;
  }
}

static void gnss_callback(const geometry_msgs::PoseStamped::ConstPtr& input)
{
  tf::Quaternion gnss_q(input->pose.orientation.x, input->pose.orientation.y, input->pose.orientation.z, input->pose.orientation.w);
  tf::Matrix3x3 gnss_m(gnss_q);
  current_gnss_pose.x = input->pose.position.x;
  current_gnss_pose.y = input->pose.position.y;
  current_gnss_pose.z = input->pose.position.z;
  gnss_m.getRPY(current_gnss_pose.roll, current_gnss_pose.pitch, current_gnss_pose.yaw);

  if ((_use_gnss == 1 && init_pos_set == 0) || fitness_score.data >= 500.0) {
    previous_pose.x = previous_gnss_pose.x;
    previous_pose.y = previous_gnss_pose.y;
    previous_pose.z = previous_gnss_pose.z;
    previous_pose.roll = previous_gnss_pose.roll;
    previous_pose.pitch = previous_gnss_pose.pitch;
    previous_pose.yaw = previous_gnss_pose.yaw;
    
    current_pose.x = current_gnss_pose.x;
    current_pose.y = current_gnss_pose.y;
    current_pose.z = current_gnss_pose.z;
    current_pose.roll = current_gnss_pose.roll;
    current_pose.pitch = current_gnss_pose.pitch;
    current_pose.yaw = current_gnss_pose.yaw;
  
    offset_x = current_pose.x - previous_pose.x;
    offset_y = current_pose.y - previous_pose.y;
    offset_z = current_pose.z - previous_pose.z;
    offset_yaw = current_pose.yaw - previous_pose.yaw;
  
    init_pos_set = 1;
  }

  previous_gnss_pose.x = current_gnss_pose.x;
  previous_gnss_pose.y = current_gnss_pose.y;
  previous_gnss_pose.z = current_gnss_pose.z;
  previous_gnss_pose.roll = current_gnss_pose.roll;
  previous_gnss_pose.pitch = current_gnss_pose.pitch;
  previous_gnss_pose.yaw = current_gnss_pose.yaw;
}

static void initialpose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& input)
{
  tf::TransformListener listener;
  tf::StampedTransform transform;
  try{
    ros::Time now = ros::Time(0);
    listener.waitForTransform("/map", "/world", now, ros::Duration(10.0));
    listener.lookupTransform("/map", "world", now, transform);
  }
  catch(tf::TransformException &ex){
    ROS_ERROR("%s", ex.what());
  }

  tf::Quaternion q(input->pose.pose.orientation.x, input->pose.pose.orientation.y, input->pose.pose.orientation.z, input->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  current_pose.x = input->pose.pose.position.x + transform.getOrigin().x();
  current_pose.y = input->pose.pose.position.y + transform.getOrigin().y();
  current_pose.z = input->pose.pose.position.z + transform.getOrigin().z();
  m.getRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);

  previous_pose.x = current_pose.x;
  previous_pose.y = current_pose.y;
  previous_pose.z = current_pose.z;
  previous_pose.roll = current_pose.roll;
  previous_pose.pitch = current_pose.pitch;
  previous_pose.yaw = current_pose.yaw;

  offset_x = 0.0;
  offset_y = 0.0;
  offset_z = 0.0;
  offset_yaw = 0.0;
}

static void hokuyo_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  if(_scanner == "hokuyo"){
    if (map_loaded == 1 && init_pos_set == 1) {
      //        callback_start = ros::Time::now();

        static tf::TransformBroadcaster br;
        tf::Transform transform;
        tf::Quaternion q;

        tf::Quaternion q_control;

        // 1 scan
        pcl::PointCloud<pcl::PointXYZ> scan;
        pcl::PointXYZ p;

        /*
         std::cout << "scan.header.stamp: " << scan.header.stamp << std::endl;
         std::cout << "scan_time: " << scan_time << std::endl;
         std::cout << "scan_time.sec: " << scan_time.sec << std::endl;
         std::cout << "scan_time.nsec: " << scan_time.nsec << std::endl;
         */

	//        t1_start = ros::Time::now();
	/*
        for (pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::const_iterator item = input->begin(); item != input->end(); item++) {
            p.x = (double) item->x;
            p.y = (double) item->y;
            p.z = (double) item->z;

            scan.points.push_back(p);
        }
	*/
	pcl::fromROSMsg(*input, scan);

	/*
        scan.header = input->header;
        scan.header.frame_id = "velodyne_scan_frame";
	*/

        ros::Time scan_time;
        scan_time.sec = scan.header.stamp / 1000000.0;
        scan_time.nsec = (scan.header.stamp - scan_time.sec * 1000000.0) * 1000.0;


	//        t1_end = ros::Time::now();
	//        d1 = t1_end - t1_start;

        Eigen::Matrix4f t(Eigen::Matrix4f::Identity());

        pcl::PointCloud<pcl::PointXYZ>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZ>(scan));
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>);

        // Downsampling the velodyne scan using VoxelGrid filter
	//        t2_start = ros::Time::now();
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
        voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
        voxel_grid_filter.setInputCloud(scan_ptr);
        voxel_grid_filter.filter(*filtered_scan_ptr);
	//        t2_end = ros::Time::now();
	//        d2 = t2_end - t2_start;

        // Setting point cloud to be aligned.
        ndt.setInputSource(filtered_scan_ptr);

        // Guess the initial gross estimation of the transformation
	//        t3_start = ros::Time::now();
        tf::Matrix3x3 init_rotation;

        predict_pose.x = previous_pose.x + offset_x;
        predict_pose.y = previous_pose.y + offset_y;
        predict_pose.z = previous_pose.z + offset_z;
        predict_pose.roll = previous_pose.roll;
        predict_pose.pitch = previous_pose.pitch;
        predict_pose.yaw = previous_pose.yaw + offset_yaw;

        Eigen::AngleAxisf init_rotation_x(predict_pose.roll, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf init_rotation_y(predict_pose.pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf init_rotation_z(predict_pose.yaw, Eigen::Vector3f::UnitZ());

        Eigen::Translation3f init_translation(predict_pose.x, predict_pose.y, predict_pose.z);

        Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

	//        t3_end = ros::Time::now();
	//        d3 = t3_end - t3_start;

	//        t4_start = ros::Time::now();
        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        ndt.align(*output_cloud, init_guess);

        t = ndt.getFinalTransformation();
	//        t4_end = ros::Time::now();
	//        d4 = t4_end - t4_start;

	//        t5_start = ros::Time::now();
        /*
         tf::Vector3 origin;
         origin.setValue(static_cast<double>(t(0,3)), static_cast<double>(t(1,3)), static_cast<double>(t(2,3)));
         */


        tf::Matrix3x3 tf3d;

        tf3d.setValue(static_cast<double>(t(0, 0)), static_cast<double>(t(0, 1)), static_cast<double>(t(0, 2)), static_cast<double>(t(1, 0)), static_cast<double>(t(1, 1)), static_cast<double>(t(1, 2)), static_cast<double>(t(2, 0)), static_cast<double>(t(2, 1)), static_cast<double>(t(2, 2)));

        // Update current_pose.
        current_pose.x = t(0, 3);
        current_pose.y = t(1, 3);
        current_pose.z = t(2, 3);
        tf3d.getRPY(current_pose.roll, current_pose.pitch, current_pose.yaw, 1);

	// control_pose
	control_pose.roll = current_pose.roll;
	control_pose.pitch = current_pose.pitch;
	control_pose.yaw = current_pose.yaw - angle / 180.0 * M_PI;
	double theta = control_pose.yaw;
	control_pose.x = cos(theta) * (-control_shift_x) + sin(theta) * (-control_shift_y) + current_pose.x;
	control_pose.y = -sin(theta) * (-control_shift_x) + cos(theta) * (-control_shift_y) + current_pose.y;
	control_pose.z = current_pose.z - control_shift_z;

        // transform "/velodyne" to "/map"
#if 1
        transform.setOrigin(tf::Vector3(current_pose.x, current_pose.y, current_pose.z));
        q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
        transform.setRotation(q);
#else
	//
	// FIXME:
	// We corrected the angle of "/velodyne" so that pure_pursuit
	// can read this frame for the control.
	// However, this is not what we want because the scan of Velodyne
	// looks unmatched for the 3-D map on Rviz.
	// What we really want is to make another TF transforming "/velodyne"
	// to a new "/ndt_points" frame and modify pure_pursuit to
	// read this frame instead of "/velodyne".
	// Otherwise, can pure_pursuit just use "/ndt_frame"?
	//
        transform.setOrigin(tf::Vector3(control_pose.x, control_pose.y, control_pose.z));
        q.setRPY(control_pose.roll, control_pose.pitch, control_pose.yaw);
        transform.setRotation(q);
#endif

	q_control.setRPY(control_pose.roll, control_pose.pitch, control_pose.yaw);

        /*
         std::cout << "ros::Time::now(): " << ros::Time::now() << std::endl;
         std::cout << "ros::Time::now().sec: " << ros::Time::now().sec << std::endl;
         std::cout << "ros::Time::now().nsec: " << ros::Time::now().nsec << std::endl;
         */

        br.sendTransform(tf::StampedTransform(transform, scan_time, "map", "3d_urg"));

        static tf::TransformBroadcaster pose_broadcaster;
        tf::Transform pose_transform;
        tf::Quaternion pose_q;

/*        pose_transform.setOrigin(tf::Vector3(0, 0, 0));
        pose_q.setRPY(0, 0, 0);
        pose_transform.setRotation(pose_q);
        pose_broadcaster.sendTransform(tf::StampedTransform(pose_transform, scan_time, "map", "ndt_frame"));
*/
        // publish the position
        current_pose_msg.header.frame_id = "/map";
        current_pose_msg.header.stamp = scan_time;
        current_pose_msg.pose.position.x = current_pose.x;
        current_pose_msg.pose.position.y = current_pose.y;
        current_pose_msg.pose.position.z = current_pose.z;
        current_pose_msg.pose.orientation.x = q.x();
        current_pose_msg.pose.orientation.y = q.y();
        current_pose_msg.pose.orientation.z = q.z();
        current_pose_msg.pose.orientation.w = q.w();

	
	//   control_pose_msg.header.frame_id = "/ndt_frame";
        control_pose_msg.header.frame_id = "/map";
        control_pose_msg.header.stamp = scan_time;
        control_pose_msg.pose.position.x = control_pose.x;
        control_pose_msg.pose.position.y = control_pose.y;
        control_pose_msg.pose.position.z = control_pose.z;
        control_pose_msg.pose.orientation.x = q_control.x();
        control_pose_msg.pose.orientation.y = q_control.y();
        control_pose_msg.pose.orientation.z = q_control.z();
        control_pose_msg.pose.orientation.w = q_control.w();

        /*
         std::cout << "ros::Time::now(): " << ros::Time::now() << std::endl;
         std::cout << "ros::Time::now().sec: " << ros::Time::now().sec << std::endl;
         std::cout << "ros::Time::now().nsec: " << ros::Time::now().nsec << std::endl;
         */

        current_pose_pub.publish(current_pose_msg);
        control_pose_pub.publish(control_pose_msg);

	int iter_num = ndt.getFinalNumIteration();
	if(iter_num > 5){
	  ndt_stat_msg.data = false;
	}else{
	  ndt_stat_msg.data = true;
	}
	ndt_stat_pub.publish(ndt_stat_msg);

	//        t5_end = ros::Time::now();
	//        d5 = t5_end - t5_start;

#ifdef OUTPUT
        // Writing position to position_log.txt
        std::ofstream ofs("position_log.txt", std::ios::app);
        if (ofs == NULL) {
            std::cerr << "Could not open 'position_log.txt'." << std::endl;
            exit(1);
        }
        ofs << current_pose.x << " " << current_pose.y << " " << current_pose.z << " " << current_pose.roll << " " << current_pose.pitch << " " << current_pose.yaw << std::endl;
#endif

        // Calculate the offset (curren_pos - previous_pose)
        offset_x = current_pose.x - previous_pose.x;
        offset_y = current_pose.y - previous_pose.y;
        offset_z = current_pose.z - previous_pose.z;
        offset_yaw = current_pose.yaw - previous_pose.yaw;

        // Update position and posture. current_pose -> previous_pose
        previous_pose.x = current_pose.x;
        previous_pose.y = current_pose.y;
        previous_pose.z = current_pose.z;
        previous_pose.roll = current_pose.roll;
        previous_pose.pitch = current_pose.pitch;
        previous_pose.yaw = current_pose.yaw;

	//        callback_end = ros::Time::now();
	//        d_callback = callback_end - callback_start;

        std::cout << "-----------------------------------------------------------------" << std::endl;
        std::cout << "Sequence number: " << input->header.seq << std::endl;
        std::cout << "Number of scan points: " << scan_ptr->size() << " points." << std::endl;
        std::cout << "Number of filtered scan points: " << filtered_scan_ptr->size() << " points." << std::endl;
        std::cout << "NDT has converged: " << ndt.hasConverged() << std::endl;
        std::cout << "Fitness score: " << ndt.getFitnessScore() << std::endl;
        std::cout << "Number of iteration: " << ndt.getFinalNumIteration() << std::endl;
        std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
        std::cout << "(" << current_pose.x << ", " << current_pose.y << ", " << current_pose.z << ", " << current_pose.roll << ", " << current_pose.pitch << ", " << current_pose.yaw << ")" << std::endl;
        std::cout << "Transformation Matrix:" << std::endl;
        std::cout << t << std::endl;
	/*
#ifdef VIEW_TIME
        std::cout << "Duration of velodyne_callback: " << d_callback.toSec() << " secs." << std::endl;
        std::cout << "Adding scan points: " << d1.toSec() << " secs." << std::endl;
        std::cout << "VoxelGrid Filter: " << d2.toSec() << " secs." << std::endl;
        std::cout << "Guessing the initial gross estimation: " << d3.toSec() << " secs." << std::endl;
        std::cout << "NDT: " << d4.toSec() << " secs." << std::endl;
        std::cout << "tf: " << d5.toSec() << " secs." << std::endl;
#endif
	*/

        std::cout << "-----------------------------------------------------------------" << std::endl;
    }
  }
}

static void velodyne_callback(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr& input)
{
  if(_scanner == "velodyne"){
    if (map_loaded == 1 && init_pos_set == 1) {

      matching_start = std::chrono::system_clock::now();

      static tf::TransformBroadcaster br;
      tf::Transform transform;
      tf::Quaternion predict_q, ndt_q, current_q, control_q;
      
      pcl::PointCloud<pcl::PointXYZ> scan;
      pcl::PointXYZ p;
      
      scan.header = input->header;
      scan.header.frame_id = "velodyne";

      current_scan_time.sec = scan.header.stamp / 1000000.0;
      current_scan_time.nsec = (scan.header.stamp - current_scan_time.sec * 1000000.0) * 1000.0;

      for (pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::const_iterator item = input->begin(); item != input->end(); item++) {
            p.x = (double) item->x;
            p.y = (double) item->y;
            p.z = (double) item->z;

	    if(item->ring >= min && item->ring <= max && item->ring % layer == 0 ){
	      scan.points.push_back(p);
	    }
      }

      sensor_msgs::PointCloud2::Ptr velodyne_points_filtered(new sensor_msgs::PointCloud2);      
      pcl::toROSMsg(scan, *velodyne_points_filtered);
      velodyne_points_filtered_pub.publish(*velodyne_points_filtered);

      Eigen::Matrix4f t(Eigen::Matrix4f::Identity());
      
      pcl::PointCloud<pcl::PointXYZ>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZ>(scan));
      pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>);
      
      // Downsampling the velodyne scan using VoxelGrid filter
      pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
      voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
      voxel_grid_filter.setInputCloud(scan_ptr);
      voxel_grid_filter.filter(*filtered_scan_ptr);
      
      // Setting point cloud to be aligned.
      ndt.setInputSource(filtered_scan_ptr);
      
      // Guess the initial gross estimation of the transformation
      predict_pose.x = previous_pose.x + offset_x;
      predict_pose.y = previous_pose.y + offset_y;
      predict_pose.z = previous_pose.z + offset_z;
      predict_pose.roll = previous_pose.roll;
      predict_pose.pitch = previous_pose.pitch;
      predict_pose.yaw = previous_pose.yaw + offset_yaw;

      Eigen::Translation3f init_translation(predict_pose.x, predict_pose.y, predict_pose.z);      
      Eigen::AngleAxisf init_rotation_x(predict_pose.roll, Eigen::Vector3f::UnitX());
      Eigen::AngleAxisf init_rotation_y(predict_pose.pitch, Eigen::Vector3f::UnitY());
      Eigen::AngleAxisf init_rotation_z(predict_pose.yaw, Eigen::Vector3f::UnitZ());
      Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();
      
      pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      ndt.align(*output_cloud, init_guess);
      
      t = ndt.getFinalTransformation();

      fitness_score.data = ndt.getFitnessScore();
      fitness_score_pub.publish(fitness_score);

      tf::Matrix3x3 tf3d;
      tf3d.setValue(static_cast<double>(t(0, 0)), static_cast<double>(t(0, 1)), static_cast<double>(t(0, 2)),
		    static_cast<double>(t(1, 0)), static_cast<double>(t(1, 1)), static_cast<double>(t(1, 2)),
		    static_cast<double>(t(2, 0)), static_cast<double>(t(2, 1)), static_cast<double>(t(2, 2)));
      
      // Update ndt_pose
      ndt_pose.x = t(0, 3);
      ndt_pose.y = t(1, 3);
      ndt_pose.z = t(2, 3);
      tf3d.getRPY(ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw, 1);

      // Compute the velocity
      scan_duration = current_scan_time - previous_scan_time;
      double secs = scan_duration.toSec();
      double distance = sqrt((ndt_pose.x - previous_pose.x) * (ndt_pose.x - previous_pose.x) +
			     (ndt_pose.y - previous_pose.y) * (ndt_pose.y - previous_pose.y) +
			     (ndt_pose.z - previous_pose.z) * (ndt_pose.z - previous_pose.z));

      current_velocity = distance / secs;
      current_acceleration = (current_velocity - previous_velocity) / secs;

      estimated_vel_mps.data = current_velocity;
      estimated_vel_kmph.data = current_velocity * 3.6;

      estimated_vel_mps_pub.publish(estimated_vel_mps);
      estimated_vel_kmph_pub.publish(estimated_vel_kmph);

      //      if((abs(estimated_vel_kmph.data - previous_estimated_vel_kmph.data) < 50.0) && (fitness_score.data < 500.0)){
      if(1){
	current_pose.x = ndt_pose.x;
	current_pose.y = ndt_pose.y;
	current_pose.z = ndt_pose.z;
	current_pose.roll = ndt_pose.roll;
	current_pose.pitch = ndt_pose.pitch;
	current_pose.yaw = ndt_pose.yaw;
      }else{
	current_pose.x = predict_pose.x;
	current_pose.y = predict_pose.y;
	current_pose.z = predict_pose.z;
	current_pose.roll = predict_pose.roll;
	current_pose.pitch = predict_pose.pitch;
	current_pose.yaw = predict_pose.yaw;
      }

      control_pose.roll = current_pose.roll;
      control_pose.pitch = current_pose.pitch;
      control_pose.yaw = current_pose.yaw - angle / 180.0 * M_PI;
      double theta = control_pose.yaw;
      control_pose.x = cos(theta) * (-control_shift_x) + sin(theta) * (-control_shift_y) + current_pose.x;
      control_pose.y = -sin(theta) * (-control_shift_x) + cos(theta) * (-control_shift_y) + current_pose.y;
      control_pose.z = current_pose.z - control_shift_z;

      // Set values for publishing pose
      predict_q.setRPY(predict_pose.roll, predict_pose.pitch, predict_pose.yaw);
      predict_pose_msg.header.frame_id = "/map";
      predict_pose_msg.header.stamp = current_scan_time;
      predict_pose_msg.pose.position.x = predict_pose.x;
      predict_pose_msg.pose.position.y = predict_pose.y;
      predict_pose_msg.pose.position.z = predict_pose.z;
      predict_pose_msg.pose.orientation.x = predict_q.x();
      predict_pose_msg.pose.orientation.y = predict_q.y();
      predict_pose_msg.pose.orientation.z = predict_q.z();
      predict_pose_msg.pose.orientation.w = predict_q.w();
      
      ndt_q.setRPY(ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw);
      ndt_pose_msg.header.frame_id = "/map";
      ndt_pose_msg.header.stamp = current_scan_time;
      ndt_pose_msg.pose.position.x = ndt_pose.x;
      ndt_pose_msg.pose.position.y = ndt_pose.y;
      ndt_pose_msg.pose.position.z = ndt_pose.z;
      ndt_pose_msg.pose.orientation.x = ndt_q.x();
      ndt_pose_msg.pose.orientation.y = ndt_q.y();
      ndt_pose_msg.pose.orientation.z = ndt_q.z();
      ndt_pose_msg.pose.orientation.w = ndt_q.w();

      current_q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
      current_pose_msg.header.frame_id = "/map";
      current_pose_msg.header.stamp = current_scan_time;
      current_pose_msg.pose.position.x = current_pose.x;
      current_pose_msg.pose.position.y = current_pose.y;
      current_pose_msg.pose.position.z = current_pose.z;
      current_pose_msg.pose.orientation.x = current_q.x();
      current_pose_msg.pose.orientation.y = current_q.y();
      current_pose_msg.pose.orientation.z = current_q.z();
      current_pose_msg.pose.orientation.w = current_q.w();
      
      control_q.setRPY(control_pose.roll, control_pose.pitch, control_pose.yaw);
      control_pose_msg.header.frame_id = "/map";
      control_pose_msg.header.stamp = current_scan_time;
      control_pose_msg.pose.position.x = control_pose.x;
      control_pose_msg.pose.position.y = control_pose.y;
      control_pose_msg.pose.position.z = control_pose.z;
      control_pose_msg.pose.orientation.x = control_q.x();
      control_pose_msg.pose.orientation.y = control_q.y();
      control_pose_msg.pose.orientation.z = control_q.z();
      control_pose_msg.pose.orientation.w = control_q.w();
      
      predict_pose_pub.publish(predict_pose_msg);
      ndt_pose_pub.publish(ndt_pose_msg);
      current_pose_pub.publish(current_pose_msg);
      control_pose_pub.publish(control_pose_msg);

      // Send TF "/velodyne" to "/map"
      transform.setOrigin(tf::Vector3(current_pose.x, current_pose.y, current_pose.z));
      transform.setRotation(current_q);
      br.sendTransform(tf::StampedTransform(transform, current_scan_time, "/map", "/velodyne"));
      
      matching_end = std::chrono::system_clock::now();

      time_ndt_matching.data = std::chrono::duration_cast<std::chrono::microseconds>(matching_end-matching_start).count()/1000.0;

      time_ndt_matching_pub.publish(time_ndt_matching);

      int iter_num = ndt.getFinalNumIteration();
      
      if(iter_num > 5){
	ndt_stat_msg.data = false;
      }else{
	ndt_stat_msg.data = true;
      }
      ndt_stat_pub.publish(ndt_stat_msg);
      
#ifdef OUTPUT
      // matching_time_log.txt
      std::ofstream ofs_time_log("matching_time_log.txt", std::ios::app);
      if (ofs_time_log == NULL) {
	std::cerr << "Could not open 'matching_time_log.txt'." << std::endl;
	exit(1);
      }
      ofs_time_log << input->header.seq << "," << time_ndt_matching.data << std::endl;

      // position_log.txt
      std::ofstream ofs_position_log("position_log.txt", std::ios::app);
      if (ofs_position_log == NULL) {
	std::cerr << "Could not open 'position_log.txt'." << std::endl;
	exit(1);
      }
      ofs_position_log << current_pose.x << ","
		       << current_pose.y << ","
		       << current_pose.z << ","
		       << current_pose.roll << ","
		       << current_pose.pitch << ","
		       << current_pose.yaw << std::endl;
#endif

      // Set values for /estimate_twist
      angular_velocity = (current_pose.yaw - previous_pose.yaw) / secs;

      estimate_twist_msg.twist.linear.x = current_velocity;
      estimate_twist_msg.twist.linear.y = 0.0;
      estimate_twist_msg.twist.linear.z = 0.0;
      estimate_twist_msg.twist.angular.x = 0.0;
      estimate_twist_msg.twist.angular.y = 0.0;
      estimate_twist_msg.twist.angular.z = angular_velocity;

      estimate_twist_pub.publish(estimate_twist_msg);

      // Set values for /ndt_stat_
      ndt_stat_msg_.header.stamp = current_scan_time;
      ndt_stat_msg_.exe_time = time_ndt_matching.data;
      ndt_stat_msg_.iteration = ndt.getFinalNumIteration();
      ndt_stat_msg_.score = ndt.getFitnessScore();
      ndt_stat_msg_.velocity = current_velocity;
      ndt_stat_msg_.acceleration = current_acceleration;
      ndt_stat_msg_.use_predict_pose = 0;

      ndt_stat_pub_.publish(ndt_stat_msg_);

      std::cout << "-----------------------------------------------------------------" << std::endl;
      std::cout << "Sequence number: " << input->header.seq << std::endl;
      std::cout << "Number of scan points: " << scan_ptr->size() << " points." << std::endl;
      std::cout << "Number of filtered scan points: " << filtered_scan_ptr->size() << " points." << std::endl;
      std::cout << "NDT has converged: " << ndt.hasConverged() << std::endl;
      std::cout << "Fitness score: " << ndt.getFitnessScore() << std::endl;
      std::cout << "Number of iteration: " << ndt.getFinalNumIteration() << std::endl;
      std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
      std::cout << "(" << current_pose.x << ", " << current_pose.y << ", " << current_pose.z
		<< ", " << current_pose.roll << ", " << current_pose.pitch << ", " << current_pose.yaw << ")" << std::endl;
      std::cout << "Transformation Matrix:" << std::endl;
      std::cout << t << std::endl;
      std::cout << "-----------------------------------------------------------------" << std::endl;
      
      // Update previous_***
      offset_x = current_pose.x - previous_pose.x;
      offset_y = current_pose.y - previous_pose.y;
      offset_z = current_pose.z - previous_pose.z;
      offset_yaw = current_pose.yaw - previous_pose.yaw;
      
      previous_pose.x = current_pose.x;
      previous_pose.y = current_pose.y;
      previous_pose.z = current_pose.z;
      previous_pose.roll = current_pose.roll;
      previous_pose.pitch = current_pose.pitch;
      previous_pose.yaw = current_pose.yaw;

      previous_scan_time.sec = current_scan_time.sec;
      previous_scan_time.nsec = current_scan_time.nsec;

      previous_velocity = current_velocity;
      previous_acceleration = current_acceleration;
      previous_estimated_vel_kmph.data = estimated_vel_kmph.data;
    }
  }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ndt_matching");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // setting parameters
    private_nh.getParam("scanner", _scanner);
    private_nh.getParam("use_gnss", _use_gnss);
    private_nh.getParam("queue_size", _queue_size);

    // Updated in initialpose_callback or gnss_callback
    initial_pose.x = 0.0;
    initial_pose.y = 0.0;
    initial_pose.z = 0.0;
    initial_pose.roll = 0.0;
    initial_pose.pitch = 0.0;
    initial_pose.yaw = 0.0;

    // Publishers
    predict_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/predict_pose", 1000);
    ndt_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/ndt_pose", 1000);
    current_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 1000);
    control_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/control_pose", 1000);
    estimate_twist_pub = nh.advertise<geometry_msgs::TwistStamped>("estimate_twist", 1000);
    ndt_stat_pub = nh.advertise<std_msgs::Bool>("/ndt_stat", 1000);
    estimated_vel_mps_pub = nh.advertise<std_msgs::Float32>("/estimated_vel_mps", 1000);
    estimated_vel_kmph_pub = nh.advertise<std_msgs::Float32>("/estimated_vel_kmph", 1000);
    fitness_score_pub = nh.advertise<std_msgs::Float32>("/fitness_score", 1000);
    time_ndt_matching_pub = nh.advertise<std_msgs::Float32>("/time_ndt_matching", 1000);
    velodyne_points_filtered_pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_filtered", 1000);
    ndt_stat_pub_ = nh.advertise<ndt_localizer::ndt_stat>("/ndt_stat_", 1000);

    // Subscribers
    ros::Subscriber param_sub = nh.subscribe("config/ndt", 10, param_callback);
    ros::Subscriber gnss_sub = nh.subscribe("gnss_pose", 10, gnss_callback);
    ros::Subscriber map_sub = nh.subscribe("points_map", 10, map_callback);
    ros::Subscriber initialpose_sub = nh.subscribe("initialpose", 1000, initialpose_callback);
    ros::Subscriber velodyne_sub = nh.subscribe("points_raw", _queue_size, velodyne_callback);
    ros::Subscriber hokuyo_sub = nh.subscribe("hokuyo_3d/hokuyo_cloud2", _queue_size, hokuyo_callback);

    ros::spin();

    return 0;
}
