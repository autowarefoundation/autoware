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

#define OUTPUT 

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

#define PREDICT_POSE_THRESHOLD 0.5

#define Wa 0.4
#define Wb 0.3
#define Wc 0.3

struct pose {
	double x;
	double y;
	double z;
	double roll;
	double pitch;
	double yaw;
};

static pose initial_pose, predict_pose, previous_pose, ndt_pose, current_pose, control_pose, localizer_pose, previous_gnss_pose, current_gnss_pose;

static double offset_x, offset_y, offset_z, offset_yaw; // current_pos - previous_pose

//Can't load if typed "pcl::PointCloud<pcl::PointXYZRGB> map, add;"
static pcl::PointCloud<pcl::PointXYZ> map, add;

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

static ros::Publisher localizer_pose_pub;
static geometry_msgs::PoseStamped localizer_pose_msg;

static ros::Publisher estimate_twist_pub;
static geometry_msgs::TwistStamped estimate_twist_msg;

static double angle = 0.0;
static double control_shift_x = 0.0;
static double control_shift_y = 0.0;
static double control_shift_z = 0.0;

static int max = 63;
static int min = 0;
static int layer = 1;

static ros::Time current_scan_time;
static ros::Time previous_scan_time;
static ros::Duration scan_duration;

static double exe_time = 0.0;
static int iteration = 0;
static double score = 0.0;
static double trans_probability = 0.0;

static double current_velocity = 0.0, previous_velocity = 0.0; // [m/s]
static double current_velocity_smooth = 0.0, second_previous_velocity = 0.0;
static double current_acceleration = 0.0, previous_acceleration = 0.0; // [m/s^2]

static double angular_velocity = 0.0;

static int use_predict_pose = 0;

static ros::Publisher estimated_vel_mps_pub, estimated_vel_kmph_pub;
static std_msgs::Float32 estimated_vel_mps, estimated_vel_kmph, previous_estimated_vel_kmph;

static std::chrono::time_point<std::chrono::system_clock> matching_start, matching_end;;
static ros::Publisher time_ndt_matching_pub;
static std_msgs::Float32 time_ndt_matching;

static int _queue_size = 1000;

static ros::Publisher ndt_stat_pub;
static ndt_localizer::ndt_stat ndt_stat_msg;

static double predict_pose_error = 0.0;

static double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
static Eigen::Matrix4f tf_btol, tf_ltob;

static std::string _localizer = "velodyne";

static ros::Publisher ndt_reliability_pub;
static std_msgs::Float32 ndt_reliability;

static void param_callback(const runtime_manager::ConfigNdt::ConstPtr& input)
{
	if (_use_gnss != input->init_pos_gnss) {
		init_pos_set = 0;
	} else if (_use_gnss == 0 && (initial_pose.x != input->x || initial_pose.y != input->y || initial_pose.z != input->z || initial_pose.roll != input->roll || initial_pose.pitch != input->pitch || initial_pose.yaw != input->yaw)) {
		init_pos_set = 0;
	}

	_use_gnss = input->init_pos_gnss;

	voxel_leaf_size = input->leaf_size;

	// Setting parameters
	if(input->resolution != ndt_res){
		ndt_res = input->resolution;
		ndt.setResolution(ndt_res);
	}
	if(input->step_size != step_size){
		step_size = input->step_size;
		ndt.setStepSize(step_size);
	}
	if(input->trans_esp != trans_eps){
		trans_eps = input->trans_esp;
		ndt.setTransformationEpsilon(trans_eps);
	}

	if (_use_gnss == 0 && init_pos_set == 0) {

		initial_pose.x = input->x;
		initial_pose.y = input->y;
		initial_pose.z = input->z;
		initial_pose.roll = input->roll;
		initial_pose.pitch = input->pitch;
		initial_pose.yaw = input->yaw;

		// Setting position and posture for the first time.
		localizer_pose.x = initial_pose.x;
		localizer_pose.y = initial_pose.y;
		localizer_pose.z = initial_pose.z;
		localizer_pose.roll = initial_pose.roll;
		localizer_pose.pitch = initial_pose.pitch;
		localizer_pose.yaw = initial_pose.yaw;

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
	}

	angle = input->angle_error;
	control_shift_x = input->shift_x;
	control_shift_y = input->shift_y;
	control_shift_z = input->shift_z;

	max = input->max;
	min = input->min;
	layer = input->layer;

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

	if ((_use_gnss == 1 && init_pos_set == 0) || score >= 500.0) {
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

static void scan_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
	if (map_loaded == 1 && init_pos_set == 1) {
		matching_start = std::chrono::system_clock::now();

		static tf::TransformBroadcaster br;
		tf::Transform transform;
		tf::Quaternion predict_q, ndt_q, current_q, control_q, localizer_q;

		pcl::PointXYZ p;
		pcl::PointCloud<pcl::PointXYZ> scan;

		current_scan_time = input->header.stamp;

		pcl::fromROSMsg(*input, scan);

		if(_localizer == "velodyne"){
			pcl::PointCloud<velodyne_pointcloud::PointXYZIR> tmp;
			pcl::fromROSMsg(*input, tmp);
			scan.points.clear();

			for (pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::const_iterator item = tmp.begin(); item != tmp.end(); item++) {
				p.x = (double) item->x;
				p.y = (double) item->y;
				p.z = (double) item->z;
				if(item->ring >= min && item->ring <= max && item->ring % layer == 0 ){
					scan.points.push_back(p);
				}
			}
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZ>(scan));
		pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>());

		Eigen::Matrix4f t(Eigen::Matrix4f::Identity()); // base_link
		Eigen::Matrix4f t2(Eigen::Matrix4f::Identity()); // localizer

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
		Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x) * tf_btol;

		pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		ndt.align(*output_cloud, init_guess);

		t = ndt.getFinalTransformation(); // localizer
		t2 = t * tf_ltob; // base_link

		iteration = ndt.getFinalNumIteration();
		score = ndt.getFitnessScore();
		trans_probability = ndt.getTransformationProbability();

		tf::Matrix3x3 mat_l; // localizer
		mat_l.setValue(static_cast<double>(t(0, 0)), static_cast<double>(t(0, 1)), static_cast<double>(t(0, 2)),
				static_cast<double>(t(1, 0)), static_cast<double>(t(1, 1)), static_cast<double>(t(1, 2)),
				static_cast<double>(t(2, 0)), static_cast<double>(t(2, 1)), static_cast<double>(t(2, 2)));

		// Update ndt_pose
		localizer_pose.x = t(0, 3);
		localizer_pose.y = t(1, 3);
		localizer_pose.z = t(2, 3);
		mat_l.getRPY(localizer_pose.roll, localizer_pose.pitch, localizer_pose.yaw, 1);

		tf::Matrix3x3 mat_b; // base_link
		mat_b.setValue(static_cast<double>(t2(0, 0)), static_cast<double>(t2(0, 1)), static_cast<double>(t2(0, 2)),
				static_cast<double>(t2(1, 0)), static_cast<double>(t2(1, 1)), static_cast<double>(t2(1, 2)),
				static_cast<double>(t2(2, 0)), static_cast<double>(t2(2, 1)), static_cast<double>(t2(2, 2)));

		// Update ndt_pose
		ndt_pose.x = t2(0, 3);
		ndt_pose.y = t2(1, 3);
		ndt_pose.z = t2(2, 3);
		mat_b.getRPY(ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw, 1);

		// Compute the velocity
		scan_duration = current_scan_time - previous_scan_time;
		double secs = scan_duration.toSec();
		double distance = sqrt((ndt_pose.x - previous_pose.x) * (ndt_pose.x - previous_pose.x) +
				(ndt_pose.y - previous_pose.y) * (ndt_pose.y - previous_pose.y) +
				(ndt_pose.z - previous_pose.z) * (ndt_pose.z - previous_pose.z));

		predict_pose_error = sqrt((ndt_pose.x - predict_pose.x) * (ndt_pose.x - predict_pose.x) +
				(ndt_pose.y - predict_pose.y) * (ndt_pose.y - predict_pose.y) +
				(ndt_pose.z - predict_pose.z) * (ndt_pose.z - predict_pose.z));

		current_velocity = distance / secs;
		current_velocity_smooth = (current_velocity + previous_velocity + second_previous_velocity) / 3.0;
		if(current_velocity_smooth < 0.2){
			current_velocity_smooth = 0.0;
		}
		current_acceleration = (current_velocity - previous_velocity) / secs;

		estimated_vel_mps.data = current_velocity;
		estimated_vel_kmph.data = current_velocity * 3.6;

		estimated_vel_mps_pub.publish(estimated_vel_mps);
		estimated_vel_kmph_pub.publish(estimated_vel_kmph);

		if(predict_pose_error <= PREDICT_POSE_THRESHOLD){
			use_predict_pose = 0;
		}else{
			use_predict_pose = 1;
		}
		use_predict_pose = 0;

		if(use_predict_pose == 0){
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

		localizer_q.setRPY(localizer_pose.roll, localizer_pose.pitch, localizer_pose.yaw);
		localizer_pose_msg.header.frame_id = "/map";
		localizer_pose_msg.header.stamp = current_scan_time;
		localizer_pose_msg.pose.position.x = localizer_pose.x;
		localizer_pose_msg.pose.position.y = localizer_pose.y;
		localizer_pose_msg.pose.position.z = localizer_pose.z;
		localizer_pose_msg.pose.orientation.x = localizer_q.x();
		localizer_pose_msg.pose.orientation.y = localizer_q.y();
		localizer_pose_msg.pose.orientation.z = localizer_q.z();
		localizer_pose_msg.pose.orientation.w = localizer_q.w();

		predict_pose_pub.publish(predict_pose_msg);
		ndt_pose_pub.publish(ndt_pose_msg);
		current_pose_pub.publish(current_pose_msg);
		control_pose_pub.publish(control_pose_msg);
		localizer_pose_pub.publish(localizer_pose_msg);

		// Send TF "/base_link" to "/map"
		transform.setOrigin(tf::Vector3(current_pose.x, current_pose.y, current_pose.z));
		transform.setRotation(current_q);
		br.sendTransform(tf::StampedTransform(transform, current_scan_time, "/map", "/base_link"));

		matching_end = std::chrono::system_clock::now();
		exe_time = std::chrono::duration_cast<std::chrono::microseconds>(matching_end-matching_start).count()/1000.0;
		time_ndt_matching.data = exe_time;
		time_ndt_matching_pub.publish(time_ndt_matching);

		// Set values for /estimate_twist
		angular_velocity = (current_pose.yaw - previous_pose.yaw) / secs;

		estimate_twist_msg.header.stamp = current_scan_time;
		estimate_twist_msg.twist.linear.x = current_velocity;
		estimate_twist_msg.twist.linear.y = 0.0;
		estimate_twist_msg.twist.linear.z = 0.0;
		estimate_twist_msg.twist.angular.x = 0.0;
		estimate_twist_msg.twist.angular.y = 0.0;
		estimate_twist_msg.twist.angular.z = angular_velocity;

		estimate_twist_pub.publish(estimate_twist_msg);

		// Set values for /ndt_stat
		ndt_stat_msg.header.stamp = current_scan_time;
		ndt_stat_msg.exe_time = time_ndt_matching.data;
		ndt_stat_msg.iteration = iteration;
		ndt_stat_msg.score = score;
		ndt_stat_msg.velocity = current_velocity;
		ndt_stat_msg.acceleration = current_acceleration;
		ndt_stat_msg.use_predict_pose = 0;

		ndt_stat_pub.publish(ndt_stat_msg);

		/* Compute NDT_Reliability */
		ndt_reliability.data = Wa * (exe_time/100.0) * 100.0 + Wb * (iteration/10.0) * 100.0 + Wc * ((2.0-trans_probability)/2.0) * 100.0;
		ndt_reliability_pub.publish(ndt_reliability);

#ifdef OUTPUT
		// Output log.csv
		std::ofstream ofs_log("log.csv", std::ios::app);
		if (ofs_log == NULL) {
			std::cerr << "Could not open 'log.csv'." << std::endl;
			exit(1);
		}
		ofs_log << input->header.seq << ","
				<< step_size << ","
				<< trans_eps << ","
				<< voxel_leaf_size << ","
				<< current_pose.x << ","
				<< current_pose.y << ","
				<< current_pose.z << ","
				<< current_pose.roll << ","
				<< current_pose.pitch << ","
				<< current_pose.yaw << ","
				<< predict_pose.x << ","
				<< predict_pose.y << ","
				<< predict_pose.z << ","
				<< predict_pose.roll << ","
				<< predict_pose.pitch << ","
				<< predict_pose.yaw << ","
				<< current_pose.x - predict_pose.x << ","
				<< current_pose.y - predict_pose.y << ","
				<< current_pose.z - predict_pose.z << ","
				<< current_pose.roll - predict_pose.roll << ","
				<< current_pose.pitch - predict_pose.pitch << ","
				<< current_pose.yaw - predict_pose.yaw << ","
				<< predict_pose_error << ","
				<< iteration << ","
				<< score << ","
				<< trans_probability << ","
				<< ndt_reliability.data << ","
				<< current_velocity << ","
				<< current_velocity_smooth << ","
				<< current_acceleration << ","
				<< angular_velocity << ","
				<< time_ndt_matching.data << ","
				<< std::endl;
#endif

		std::cout << "-----------------------------------------------------------------" << std::endl;
		std::cout << "Sequence: " << input->header.seq << std::endl;
		std::cout << "Timestamp: " << input->header.stamp << std::endl;
		std::cout << "Frame ID: " << input->header.frame_id << std::endl;
		std::cout << "Number of Scan Points: " << scan_ptr->size() << " points." << std::endl;
		std::cout << "Number of Filtered Scan Points: " << filtered_scan_ptr->size() << " points." << std::endl;
		std::cout << "Leaf Size: " << voxel_leaf_size << std::endl;
		std::cout << "NDT has converged: " << ndt.hasConverged() << std::endl;
		std::cout << "Fitness Score: " << ndt.getFitnessScore() << std::endl;
		std::cout << "Transformation Probability: " << ndt.getTransformationProbability() << std::endl;
		std::cout << "Execution Time: " << exe_time << " ms." << std::endl;
		std::cout << "Number of Iterations: " << ndt.getFinalNumIteration() << std::endl;
		std::cout << "NDT Reliability: " << ndt_reliability.data << std::endl;
		std::cout << "(x,y,z,roll,pitch,yaw): " << std::endl;
		std::cout << "(" << current_pose.x << ", " << current_pose.y << ", " << current_pose.z
				<< ", " << current_pose.roll << ", " << current_pose.pitch << ", " << current_pose.yaw << ")" << std::endl;
		std::cout << "Transformation Matrix: " << std::endl;
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

		second_previous_velocity = previous_velocity;
		previous_velocity = current_velocity;
		previous_acceleration = current_acceleration;
		previous_estimated_vel_kmph.data = estimated_vel_kmph.data;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ndt_matching");

	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	// setting parameters
	private_nh.getParam("use_gnss", _use_gnss);
	private_nh.getParam("queue_size", _queue_size);

	if(nh.getParam("localizer", _localizer) == false){
		std::cout << "localizer is not set." << std::endl;
		return 1;
	}

	if(nh.getParam("tf_x", _tf_x) == false){
		std::cout << "tf_x is not set." << std::endl;
		return 1;
	}
	if(nh.getParam("tf_y", _tf_y) == false){
		std::cout << "tf_y is not set." << std::endl;
		return 1;
	}
	if(nh.getParam("tf_z", _tf_z) == false){
		std::cout << "tf_z is not set." << std::endl;
		return 1;
	}
	if(nh.getParam("tf_roll", _tf_roll) == false){
		std::cout << "tf_roll is not set." << std::endl;
		return 1;
	}
	if(nh.getParam("tf_pitch", _tf_pitch) == false){
		std::cout << "tf_pitch is not set." << std::endl;
		return 1;
	}
	if(nh.getParam("tf_yaw", _tf_yaw) == false){
		std::cout << "tf_yaw is not set." << std::endl;
		return 1;
	}

	std::cout << "_localizer: " << _localizer << std::endl;
	std::cout << "_tf_x: " << _tf_x << std::endl;
	std::cout << "_tf_y: " << _tf_y << std::endl;
	std::cout << "_tf_z: " << _tf_z << std::endl;
	std::cout << "_tf_roll: " << _tf_roll << std::endl;
	std::cout << "_tf_pitch: " << _tf_pitch << std::endl;
	std::cout << "_tf_yaw: " << _tf_yaw << std::endl;

	Eigen::Translation3f tl_btol(_tf_x, _tf_y, _tf_z); // tl: translation
	Eigen::AngleAxisf rot_x_btol(_tf_roll, Eigen::Vector3f::UnitX()); //rot: rotation
	Eigen::AngleAxisf rot_y_btol(_tf_pitch, Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf rot_z_btol(_tf_yaw, Eigen::Vector3f::UnitZ());
	tf_btol = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

	Eigen::Translation3f tl_ltob((-1.0)*_tf_x, (-1.0)*_tf_y, (-1.0)*_tf_z); // tl: translation
	Eigen::AngleAxisf rot_x_ltob((-1.0)*_tf_roll, Eigen::Vector3f::UnitX()); //rot: rotation
	Eigen::AngleAxisf rot_y_ltob((-1.0)*_tf_pitch, Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf rot_z_ltob((-1.0)*_tf_yaw, Eigen::Vector3f::UnitZ());
	tf_ltob = (tl_ltob * rot_z_ltob * rot_y_ltob * rot_x_ltob).matrix();

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
	localizer_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/localizer_pose", 1000);
	estimate_twist_pub = nh.advertise<geometry_msgs::TwistStamped>("/estimate_twist", 1000);
	estimated_vel_mps_pub = nh.advertise<std_msgs::Float32>("/estimated_vel_mps", 1000);
	estimated_vel_kmph_pub = nh.advertise<std_msgs::Float32>("/estimated_vel_kmph", 1000);
	time_ndt_matching_pub = nh.advertise<std_msgs::Float32>("/time_ndt_matching", 1000);
	ndt_stat_pub = nh.advertise<ndt_localizer::ndt_stat>("/ndt_stat", 1000);
	ndt_reliability_pub = nh.advertise<std_msgs::Float32>("/ndt_reliability", 1000);

	// Subscribers
	ros::Subscriber param_sub = nh.subscribe("config/ndt", 10, param_callback);
	ros::Subscriber gnss_sub = nh.subscribe("gnss_pose", 10, gnss_callback);
	ros::Subscriber map_sub = nh.subscribe("points_map", 10, map_callback);
	ros::Subscriber initialpose_sub = nh.subscribe("initialpose", 1000, initialpose_callback);
	ros::Subscriber scan_sub = nh.subscribe("points_raw", _queue_size, scan_callback);

	ros::spin();

	return 0;
}
