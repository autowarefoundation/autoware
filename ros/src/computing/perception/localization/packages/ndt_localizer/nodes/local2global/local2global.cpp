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

// #define VIEW_TIME

// If you want to output "position_log.txt", "#define OUTPUT".
//#define OUTPUT 

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/rawdata.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>

#include <runtime_manager/ConfigNdt.h>

struct Position {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

// global variables
static Position previous_pos, guess_pos, current_pos, current_pos_control;

static double offset_x = 0.0;
static double offset_y = 0.0;
static double offset_z = 0.0;
static double offset_yaw = 0.0; // current_pos - previous_pos

// Initial position (updated in param_callback)
static double initial_x = 0.0;
static double initial_y = 0.0;
static double initial_z = 0.0;
static double initial_roll = 0.0;
static double initial_pitch = 0.0;
static double initial_yaw = 0.0;

//Can't load if typed "pcl::PointCloud<pcl::PointXYZRGB> map, add;"
static pcl::PointCloud<pcl::PointXYZI> map, add;
static pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr;

static pcl::PointCloud<pcl::PointXYZI> additional_map;
pcl::PointCloud<pcl::PointXYZI>::Ptr additional_map_ptr(new pcl::PointCloud<pcl::PointXYZI>);

// If the map is loaded, map_loaded will be 1.
static int map_loaded = 0;
static int use_gnss = 1;
static int init_pos_set = 1;

static pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
// Default values
static int iter = 100; // Maximum iterations
static float ndt_res = 1.0; // Resolution
static double step_size = 0.1; // Step size
static double trans_eps = 0.01; // Transformation epsilon

// Leaf size of VoxelGrid filter.
static double voxel_leaf_size = 2.0;

static ros::Time callback_start, callback_end, t1_start, t1_end, t2_start, t2_end, t3_start, t3_end, t4_start, t4_end, t5_start, t5_end;
static ros::Duration d_callback, d1, d2, d3, d4, d5;

static ros::Publisher ndt_pose_pub;
static geometry_msgs::PoseStamped ndt_pose_msg;

static ros::Publisher control_pose_pub;
static geometry_msgs::PoseStamped control_pose_msg;

static double angle = 0.0;
static double control_shift_x = 0.0;
static double control_shift_y = 0.0;
static double control_shift_z = 0.0;

static ros::Publisher ndt_map_pub;

static void param_callback(const runtime_manager::ConfigNdt::ConstPtr& input)
{
    if (use_gnss != input->init_pos_gnss) {
        init_pos_set = 0;
    } else if (use_gnss == 0 && (initial_x != input->x || initial_y != input->y || initial_z != input->z || initial_roll != input->roll || initial_pitch != input->pitch || initial_yaw != input->yaw)) {
        init_pos_set = 0;
    }

    use_gnss = input->init_pos_gnss;

    ndt_res = input->resolution;
    step_size = input->step_size;
    trans_eps = input->trans_esp;

    voxel_leaf_size = input->leaf_size;

    // Setting parameters
    ndt.setMaximumIterations(iter);
    ndt.setResolution(ndt_res);
    ndt.setStepSize(step_size);
    ndt.setTransformationEpsilon(trans_eps);

    std::cout << "--- Parameters ---" << std::endl;
    std::cout << "ndt_res: " << ndt_res << std::endl;
    std::cout << "step_size: " << step_size << std::endl;
    std::cout << "trans_eps: " << trans_eps << std::endl;
    std::cout << "voxel_leaf_size: " << voxel_leaf_size << std::endl;

    if (use_gnss == 0 && init_pos_set == 0) {
        initial_x = input->x;
        initial_y = input->y;
        initial_z = input->z;
        initial_roll = input->roll;
        initial_pitch = input->pitch;
        initial_yaw = input->yaw;
        // Setting position and posture for the first time.
        previous_pos.x = initial_x;
        previous_pos.y = initial_y;
        previous_pos.z = initial_z;
        previous_pos.roll = initial_roll;
        previous_pos.pitch = initial_pitch;
        previous_pos.yaw = initial_yaw;
        current_pos.x = initial_x;
        current_pos.y = initial_y;
        current_pos.z = initial_z;
        current_pos.roll = initial_roll;
        current_pos.pitch = initial_pitch;
        current_pos.yaw = initial_yaw;

        init_pos_set = 1;
        std::cout << "--- Initial Position (static) ---" << std::endl;
        std::cout << "initial_x: " << initial_x << std::endl;
        std::cout << "initial_y: " << initial_y << std::endl;
        std::cout << "initial_z: " << initial_z << std::endl;
        std::cout << "initial_roll: " << initial_roll << std::endl;
        std::cout << "initial_pitch: " << initial_pitch << std::endl;
        std::cout << "initial_yaw: " << initial_yaw << std::endl;
        std::cout << "NDT ready..." << std::endl;
    }

    angle = input->angle_error;
    control_shift_x = input->shift_x;
    control_shift_y = input->shift_y;
    control_shift_z = input->shift_z;

    std::cout << "angle_error: " << angle << "." << std::endl;
    std::cout << "control_shift_x: " << control_shift_x << "." << std::endl;
    std::cout << "control_shift_y: " << control_shift_y << "." << std::endl;
    std::cout << "control_shift_z: " << control_shift_z << "." << std::endl;
}

static void gnss_callback(const geometry_msgs::PoseStamped::ConstPtr& input)
{
    if (use_gnss == 1 && init_pos_set == 0) {
        tf::Quaternion q(input->pose.orientation.x, input->pose.orientation.y, input->pose.orientation.z, input->pose.orientation.w);
        tf::Matrix3x3 m(q);
        initial_x = input->pose.position.x;
        initial_y = input->pose.position.y;
        initial_z = input->pose.position.z;
        m.getRPY(initial_roll, initial_pitch, initial_yaw);

        // Setting position and posture for the fist time.
        previous_pos.x = initial_x;
        previous_pos.y = initial_y;
        previous_pos.z = initial_z;
        previous_pos.roll = initial_roll;
        previous_pos.pitch = initial_pitch;
        previous_pos.yaw = initial_yaw;

        current_pos.x = initial_x;
        current_pos.y = initial_y;
        current_pos.z = initial_z;
        current_pos.roll = initial_roll;
        current_pos.pitch = initial_pitch;
        current_pos.yaw = initial_yaw;

        init_pos_set = 1;
        std::cout << "--- Initial Position (gnss) ---" << std::endl;
        std::cout << "initial_x: " << initial_x << std::endl;
        std::cout << "initial_y: " << initial_y << std::endl;
        std::cout << "initial_z: " << initial_z << std::endl;
        std::cout << "initial_roll: " << initial_roll << std::endl;
        std::cout << "initial_pitch: " << initial_pitch << std::endl;
        std::cout << "initial_yaw: " << initial_yaw << std::endl;
        std::cout << "NDT ready..." << std::endl;
    }
}

//static void velodyne_callback(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr& input)
static void map_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{

  if (map_loaded == 0) {
    std::cout << "Loading map data... ";
    map.header.frame_id = "/pointcloud_map_frame";
    
    // Convert the data type(from sensor_msgs to pcl).
    pcl::fromROSMsg(*input, map);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));
    // Setting point cloud to be aligned to.
    ndt.setInputTarget(map_ptr);
	
    // Setting NDT parameters to default values
    ndt.setMaximumIterations(iter);
    ndt.setResolution(ndt_res);
    ndt.setStepSize(step_size);
    ndt.setTransformationEpsilon(trans_eps);
    
    map_loaded = 1;
    std::cout << "Map Loaded." << std::endl;
  }



    if (map_loaded == 1 && init_pos_set == 1) {
        callback_start = ros::Time::now();

        static tf::TransformBroadcaster br;
        tf::Transform transform;
        tf::Quaternion q;

        tf::Quaternion q_control;

        // 1 scan
	/*
        pcl::PointCloud<pcl::PointXYZI> scan;
        pcl::PointXYZI p;
        scan.header = input->header;
        scan.header.frame_id = "velodyne_scan_frame";
	*/

        ros::Time scan_time;
        scan_time.sec = additional_map.header.stamp / 1000000.0;
        scan_time.nsec = (additional_map.header.stamp - scan_time.sec * 1000000.0) * 1000.0;

        /*
         std::cout << "scan.header.stamp: " << scan.header.stamp << std::endl;
         std::cout << "scan_time: " << scan_time << std::endl;
         std::cout << "scan_time.sec: " << scan_time.sec << std::endl;
         std::cout << "scan_time.nsec: " << scan_time.nsec << std::endl;
         */

        t1_start = ros::Time::now();
	/*
        for (pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::const_iterator item = input->begin(); item != input->end(); item++) {
            p.x = (double) item->x;
            p.y = (double) item->y;
            p.z = (double) item->z;

            scan.points.push_back(p);
        }
	*/
	//	pcl::fromROSMsg(*input, scan);
        t1_end = ros::Time::now();
        d1 = t1_end - t1_start;

        Eigen::Matrix4f t(Eigen::Matrix4f::Identity());

	//        pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_additional_map_ptr(new pcl::PointCloud<pcl::PointXYZI>);

        // Downsampling the velodyne scan using VoxelGrid filter
        t2_start = ros::Time::now();
        pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
        voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
        voxel_grid_filter.setInputCloud(additional_map_ptr);
        voxel_grid_filter.filter(*filtered_additional_map_ptr);
        t2_end = ros::Time::now();
        d2 = t2_end - t2_start;

        // Setting point cloud to be aligned.
        ndt.setInputSource(filtered_additional_map_ptr);

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
        pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        ndt.align(*output_cloud, init_guess);

        t = ndt.getFinalTransformation();
	pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_additional_map_ptr (new pcl::PointCloud<pcl::PointXYZI>());
	transformed_additional_map_ptr->header.frame_id = "/map";
	pcl::transformPointCloud(*additional_map_ptr, *transformed_additional_map_ptr, t);
	sensor_msgs::PointCloud2::Ptr msg_ptr(new sensor_msgs::PointCloud2);

	pcl::toROSMsg(*transformed_additional_map_ptr, *msg_ptr);
	msg_ptr->header.frame_id = "/map";
	ndt_map_pub.publish(*msg_ptr);

	// Writing Point Cloud data to PCD file
	pcl::io::savePCDFileASCII("global_map.pcd", *transformed_additional_map_ptr);
	std::cout << "Saved " << transformed_additional_map_ptr->points.size() << " data points to global_map.pcd." << std::endl;

	pcl::PointCloud<pcl::PointXYZRGB> output;
	output.width = transformed_additional_map_ptr->width;
	output.height = transformed_additional_map_ptr->height;
	output.points.resize(output.width * output.height);

	for(size_t i = 0; i < output.points.size(); i++){
	  output.points[i].x = transformed_additional_map_ptr->points[i].x;
	  output.points[i].y = transformed_additional_map_ptr->points[i].y;
	  output.points[i].z = transformed_additional_map_ptr->points[i].z;
	  output.points[i].rgb = 255 << 8;
	}

	pcl::io::savePCDFileASCII("global_map_rgb.pcd", output);
	std::cout << "Saved " << output.points.size() << " data points to global_map_rgb.pcd." << std::endl;

        t4_end = ros::Time::now();
        d4 = t4_end - t4_start;

        t5_start = ros::Time::now();
        /*
         tf::Vector3 origin;
         origin.setValue(static_cast<double>(t(0,3)), static_cast<double>(t(1,3)), static_cast<double>(t(2,3)));
         */

        tf::Matrix3x3 tf3d;

        tf3d.setValue(static_cast<double>(t(0, 0)), static_cast<double>(t(0, 1)), static_cast<double>(t(0, 2)), static_cast<double>(t(1, 0)), static_cast<double>(t(1, 1)), static_cast<double>(t(1, 2)), static_cast<double>(t(2, 0)), static_cast<double>(t(2, 1)), static_cast<double>(t(2, 2)));

        // Update current_pos.
        current_pos.x = t(0, 3);
        current_pos.y = t(1, 3);
        current_pos.z = t(2, 3);
        tf3d.getRPY(current_pos.roll, current_pos.pitch, current_pos.yaw, 1);

	// control_pose
	current_pos_control.roll = current_pos.roll;
	current_pos_control.pitch = current_pos.pitch;
	current_pos_control.yaw = current_pos.yaw - angle / 180.0 * M_PI;
	double theta = current_pos_control.yaw;
	current_pos_control.x = cos(theta) * (-control_shift_x) + sin(theta) * (-control_shift_y) + current_pos.x;
	current_pos_control.y = -sin(theta) * (-control_shift_x) + cos(theta) * (-control_shift_y) + current_pos.y;
	current_pos_control.z = current_pos.z - control_shift_z;

        // transform "/velodyne" to "/map"
#if 0
        transform.setOrigin(tf::Vector3(current_pos.x, current_pos.y, current_pos.z));
        q.setRPY(current_pos.roll, current_pos.pitch, current_pos.yaw);
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
        transform.setOrigin(tf::Vector3(current_pos_control.x, current_pos_control.y, current_pos_control.z));
        q.setRPY(current_pos_control.roll, current_pos_control.pitch, current_pos_control.yaw);
        transform.setRotation(q);
#endif

	q_control.setRPY(current_pos_control.roll, current_pos_control.pitch, current_pos_control.yaw);

        /*
         std::cout << "ros::Time::now(): " << ros::Time::now() << std::endl;
         std::cout << "ros::Time::now().sec: " << ros::Time::now().sec << std::endl;
         std::cout << "ros::Time::now().nsec: " << ros::Time::now().nsec << std::endl;
         */

	//        br.sendTransform(tf::StampedTransform(transform, scan_time, "map", "velodyne"));

        static tf::TransformBroadcaster pose_broadcaster;
        tf::Transform pose_transform;
        tf::Quaternion pose_q;

/*        pose_transform.setOrigin(tf::Vector3(0, 0, 0));
        pose_q.setRPY(0, 0, 0);
        pose_transform.setRotation(pose_q);
        pose_broadcaster.sendTransform(tf::StampedTransform(pose_transform, scan_time, "map", "ndt_frame"));
*/
        // publish the position
       // ndt_pose_msg.header.frame_id = "/ndt_frame";
        ndt_pose_msg.header.frame_id = "/map";
        ndt_pose_msg.header.stamp = scan_time;
        ndt_pose_msg.pose.position.x = current_pos.x;
        ndt_pose_msg.pose.position.y = current_pos.y;
        ndt_pose_msg.pose.position.z = current_pos.z;
        ndt_pose_msg.pose.orientation.x = q.x();
        ndt_pose_msg.pose.orientation.y = q.y();
        ndt_pose_msg.pose.orientation.z = q.z();
        ndt_pose_msg.pose.orientation.w = q.w();

        static tf::TransformBroadcaster pose_broadcaster_control;
        tf::Transform pose_transform_control;
        tf::Quaternion pose_q_control;

     /*   pose_transform_control.setOrigin(tf::Vector3(0, 0, 0));
        pose_q_control.setRPY(0, 0, 0);
        pose_transform_control.setRotation(pose_q_control);
        pose_broadcaster_control.sendTransform(tf::StampedTransform(pose_transform_control, scan_time, "map", "ndt_frame"));
*/
        // publish the position
     //   control_pose_msg.header.frame_id = "/ndt_frame";
        control_pose_msg.header.frame_id = "/map";
        control_pose_msg.header.stamp = scan_time;
        control_pose_msg.pose.position.x = current_pos_control.x;
        control_pose_msg.pose.position.y = current_pos_control.y;
        control_pose_msg.pose.position.z = current_pos_control.z;
        control_pose_msg.pose.orientation.x = q_control.x();
        control_pose_msg.pose.orientation.y = q_control.y();
        control_pose_msg.pose.orientation.z = q_control.z();
        control_pose_msg.pose.orientation.w = q_control.w();

        /*
         std::cout << "ros::Time::now(): " << ros::Time::now() << std::endl;
         std::cout << "ros::Time::now().sec: " << ros::Time::now().sec << std::endl;
         std::cout << "ros::Time::now().nsec: " << ros::Time::now().nsec << std::endl;
         */

        ndt_pose_pub.publish(ndt_pose_msg);
        control_pose_pub.publish(control_pose_msg);

        t5_end = ros::Time::now();
        d5 = t5_end - t5_start;

#ifdef OUTPUT
        // Writing position to position_log.txt
        std::ofstream ofs("position_log.txt", std::ios::app);
        if (ofs == NULL) {
            std::cerr << "Could not open 'position_log.txt'." << std::endl;
            exit(1);
        }
        ofs << current_pos.x << " " << current_pos.y << " " << current_pos.z << " " << current_pos.roll << " " << current_pos.pitch << " " << current_pos.yaw << std::endl;
#endif

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
        std::cout << "Number of scan points: " << additional_map_ptr->size() << " points." << std::endl;
        std::cout << "Number of filtered scan points: " << filtered_additional_map_ptr->size() << " points." << std::endl;
        std::cout << "NDT has converged: " << ndt.hasConverged() << std::endl;
        std::cout << "Fitness score: " << ndt.getFitnessScore() << std::endl;
        std::cout << "Number of iteration: " << ndt.getFinalNumIteration() << std::endl;
        std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
        std::cout << "(" << current_pos.x << ", " << current_pos.y << ", " << current_pos.z << ", " << current_pos.roll << ", " << current_pos.pitch << ", " << current_pos.yaw << ")" << std::endl;
        std::cout << "Transformation Matrix:" << std::endl;
        std::cout << t << std::endl;
#ifdef VIEW_TIME
        std::cout << "Duration of velodyne_callback: " << d_callback.toSec() << " secs." << std::endl;
        std::cout << "Adding scan points: " << d1.toSec() << " secs." << std::endl;
        std::cout << "VoxelGrid Filter: " << d2.toSec() << " secs." << std::endl;
        std::cout << "Guessing the initial gross estimation: " << d3.toSec() << " secs." << std::endl;
        std::cout << "NDT: " << d4.toSec() << " secs." << std::endl;
        std::cout << "tf: " << d5.toSec() << " secs." << std::endl;
#endif
        std::cout << "-----------------------------------------------------------------" << std::endl;
    }
}

int main(int argc, char **argv)
{
    if(argc != 8){
      std::cout << "Usage: rosrun ndt_localizer local2global \"filename\" \"x\" \"y\" \"z\" \"roll\" \"pitch\" \"yaw\"" << std::endl;
      exit(1);
    }

    ros::init(argc, argv, "local2global");
    ros::NodeHandle n;

    std::string filename = argv[1];
    std::cout << filename << std::endl;

    if(pcl::io::loadPCDFile<pcl::PointXYZI> (filename, *additional_map_ptr) == -1){
      std::cout << "Couldn't read " << filename << "." << std::endl;
      return(-1);
    }
    std::cout << "Loaded " << additional_map_ptr->size() << " data points from " << filename << "." << std::endl;

    initial_x = std::stod(argv[2]);
    initial_y = std::stod(argv[3]);
    initial_z = std::stod(argv[4]);
    initial_roll = std::stod(argv[5]);
    initial_pitch = std::stod(argv[6]);
    initial_yaw = std::stod(argv[7]);

    std::cout << "initial_pose: "
	      << initial_x << " " << initial_y << " " << initial_z << " "
	      << initial_roll << " " << initial_pitch << " " << initial_yaw << std::endl;

    previous_pos.x = initial_x;
    previous_pos.y = initial_y;
    previous_pos.z = initial_z;
    previous_pos.roll = initial_roll;
    previous_pos.pitch = initial_pitch;
    previous_pos.yaw = initial_yaw;
    
    current_pos.x = initial_x;
    current_pos.y = initial_y;
    current_pos.z = initial_z;
    current_pos.roll = initial_roll;
    current_pos.pitch = initial_pitch;
    current_pos.yaw = initial_yaw;

    ndt_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/current_pose", 1000);

    control_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/control_pose", 1000);

    ndt_map_pub = n.advertise<sensor_msgs::PointCloud2>("/ndt_map", 1000, true);

    // subscribing parameter
    ros::Subscriber param_sub = n.subscribe("config/ndt", 10, param_callback);

    // subscribing gnss position
    ros::Subscriber gnss_sub = n.subscribe("gnss_pose", 10, gnss_callback);

    // subscribing map data (only once)
    ros::Subscriber map_sub = n.subscribe("points_map", 10, map_callback);

    // subscribing the velodyne data
    //    ros::Subscriber velodyne_sub = n.subscribe("points_raw", 1000, velodyne_callback);
    //    ros::Subscriber velodyne_sub = n.subscribe("cloud_pcd", 1000, velodyne_callback);

    ros::spin();

    return 0;
}
