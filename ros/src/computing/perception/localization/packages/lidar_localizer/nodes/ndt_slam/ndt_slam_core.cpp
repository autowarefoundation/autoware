/*
 *  Copyright (c) 2017, Tier IV, Inc.
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

#include "ndt_slam_core.h"

#include <pcl_conversions/pcl_conversions.h>

#include <lidar_localizer/util/data_structs.h>
#include <lidar_localizer/util/convert_ros_msgs.h>
#include <visualization_msgs/MarkerArray.h>

template <class PointType>
static void limitPointCloudRange(const boost::shared_ptr< pcl::PointCloud<PointType> > input_ptr, boost::shared_ptr< pcl::PointCloud<PointType> > output_ptr, double min_range_meter, double max_range_meter)
{
    double r = 0;
    for (const auto& p : input_ptr->points) {
      r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
      if (r > min_range_meter && r < max_range_meter) {
        output_ptr->push_back(p);
      }
    }
}

NdtSlam::NdtSlam(ros::NodeHandle nh, ros::NodeHandle private_nh)
    :nh_(nh)
    ,private_nh_(private_nh)
    ,method_type_(MethodType::PCL_GENERIC)
    ,with_mapping_(false)
    ,separate_mapping_(false)
    ,init_pos_gnss_(false)
    ,use_odometry_(false)
    ,sensor_frame_("/velodyne")
    ,base_link_frame_("/base_link")
    ,map_frame_("/map")
    ,min_scan_range_(5.0)
    ,max_scan_range_(200.0)
    ,min_add_scan_shift_(1.0)

{

    int method_type_tmp = 1;
    private_nh.getParam("method_type", method_type_tmp);
    method_type_ = static_cast<MethodType>(method_type_tmp);

    //TODO switch-break
    if(method_type_ == MethodType::PCL_GENERIC) {
        ROS_INFO("use NDT SLAM PCL GENERIC version");
        localizer_ptr_.reset(new NdtSlamPCL<PointSource, PointTarget>);
    }
    else if(method_type_ == MethodType::PCL_OPENMP) {
        ROS_INFO("use NDT SLAM PCL OPENMP version");
        localizer_ptr_.reset(new NdtSlamPCLOMP<PointSource, PointTarget>);
    }
    else if(method_type_ == MethodType::PCL_ANH) {
        ROS_INFO("use NDT SLAM PCL ANH version");
        localizer_ptr_.reset(new NdtSlamPCLANH<PointSource, PointTarget>);
    }
    else if(method_type_ == MethodType::PCL_ANH_GPU) {
        ROS_INFO("use NDT SLAM PCL ANH GPU version");
        localizer_ptr_.reset(new NdtSlamPCLANHGPU<PointSource, PointTarget>);
    }
    else {
        // ROS_INFO("unkonwn method_type. use NDT SLAM PCL GENERIC version");
        // localizer_ptr_.reset(new NdtSlamPCL<PointSource, PointTarget>);

        ROS_INFO("unkonwn method_type. use NDT SLAM PCL ANH version");
        localizer_ptr_.reset(new NdtSlamPCLANH<PointSource, PointTarget>);
    }

    private_nh_.getParam("init_pos_gnss", init_pos_gnss_);
    ROS_INFO("init_pos_gnss: %d", init_pos_gnss_);

    int points_queue_size = 1;
    private_nh_.getParam("points_queue_size", points_queue_size);
    points_queue_size = points_queue_size <= 0 ? 1 : points_queue_size;
    ROS_INFO("points_queue_size: %d", points_queue_size);

    private_nh_.getParam("sensor_frame", sensor_frame_);
    private_nh_.getParam("base_link_frame", base_link_frame_);
    private_nh_.getParam("map_frame", map_frame_);
    ROS_INFO("sensor_frame_id: %s, base_link_frame_id: %s, map_frame_id: %s", sensor_frame_.c_str(), base_link_frame_.c_str(), map_frame_.c_str());

    double trans_epsilon = 0.01;
    double step_size = 0.1;
    double resolution = 1.0;
    int max_iterations = 30;
    private_nh_.getParam("trans_epsilon", trans_epsilon);
    private_nh_.getParam("step_size", step_size);
    private_nh_.getParam("resolution", resolution);
    private_nh_.getParam("max_iterations", max_iterations);
    localizer_ptr_->setTransformationEpsilon(trans_epsilon);
    localizer_ptr_->setStepSize(step_size);
    localizer_ptr_->setResolution(resolution);
    localizer_ptr_->setMaximumIterations(max_iterations);
    ROS_INFO("trans_epsilon: %lf, step_size: %lf, resolution: %lf, max_iterations: %d", trans_epsilon, step_size, resolution, max_iterations);

    double save_map_leaf_size = 0.2;
    double separate_map_size = 100.0;
    private_nh_.getParam("with_mapping", with_mapping_);
    private_nh_.getParam("save_map_leaf_size", save_map_leaf_size);
    private_nh_.getParam("min_scan_range", min_scan_range_);
    private_nh_.getParam("max_scan_range", max_scan_range_);
    private_nh_.getParam("min_add_scan_shift", min_add_scan_shift_);
    private_nh_.getParam("separate_mapping", separate_mapping_);
    private_nh_.getParam("separate_map_size", separate_map_size);
    localizer_ptr_->setSaveMapLeafSize(save_map_leaf_size);
    localizer_ptr_->setSaveSeparateMapSize(separate_map_size);
    ROS_INFO("with_mapping: %d, save_map_leaf_size: %lf, min_scan_range: %lf, max_scan_range: %lf, min_add_scan_shift: %lf", with_mapping_, save_map_leaf_size, min_scan_range_, max_scan_range_, min_add_scan_shift_);
    ROS_INFO("separate_mapping: %d, separate_map_size: %lf", separate_mapping_, separate_map_size);

    private_nh_.getParam("use_odometry", use_odometry_);
    ROS_INFO("use_odometry: %d", use_odometry_);

    //TODO
    //reliability parameter

    Pose tf_pose;
    try {
        tf::StampedTransform tf_base_link_to_sensor;
        ros::Duration(0.1).sleep();  //wait for tf. especially use sim_time
        tf_listener_.waitForTransform(base_link_frame_, sensor_frame_, ros::Time(0), ros::Duration(1.0));
        tf_listener_.lookupTransform(base_link_frame_, sensor_frame_, ros::Time(0), tf_base_link_to_sensor);
        tf_pose.x = tf_base_link_to_sensor.getOrigin().x();
        tf_pose.y = tf_base_link_to_sensor.getOrigin().y();
        tf_pose.z = tf_base_link_to_sensor.getOrigin().z();
        tf::Matrix3x3(tf_base_link_to_sensor.getRotation()).getRPY(tf_pose.roll, tf_pose.pitch, tf_pose.yaw);
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ROS_ERROR("Please publish TF %s to %s", base_link_frame_.c_str(), sensor_frame_.c_str());
        //exit(1);
    }

    ROS_INFO("tf(x, y, z, roll, pitch, yaw): %lf, %lf, %lf, %lf, %lf, %lf", tf_pose.x, tf_pose.y, tf_pose.z, tf_pose.roll, tf_pose.pitch, tf_pose.yaw);
    tf_btol_ = convertToEigenMatrix4f(tf_pose);

    points_map_region_pub_  = nh_.advertise<sensor_msgs::PointCloud2>   ("points_map_region",  10);
    localizer_pose_pub_     = nh_.advertise<geometry_msgs::PoseStamped> ("localizer_pose",     10);
    localizer_twist_pub_    = nh_.advertise<geometry_msgs::TwistStamped>("localizer_velocity", 10);
    localizer_score_pub_    = nh_.advertise<std_msgs::Float32>  ("localizer_score", 10);
    localizer_score_ave_pub_ = nh_.advertise<std_msgs::Float32>  ("localizer_score_ave", 10);
    localizer_score_var_pub_ = nh_.advertise<std_msgs::Float32>  ("localizer_score_var", 10);
    ndvoxel_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray> ("ndvoxel", 10);

    config_sub_ = nh_.subscribe("/config/ndtslam", 10, &NdtSlam::configCallback, this);
    //points_map_updated_sub_ = nh_.subscribe("/points_map_updated", 10, &NdtSlam::pointsMapUpdatedCallback, this);
    points_map_updated_sub_ = nh_.subscribe("/points_map", 10, &NdtSlam::pointsMapUpdatedCallback, this);
    //manual_pose_sub_        = nh_.subscribe("/manual_pose",        10, &NdtSlam::manualPoseCallback, this);
    manual_pose_sub_        = nh_.subscribe("/initialpose",        10, &NdtSlam::manualPoseCallback, this);
    static_pose_sub_        = nh_.subscribe("/static_pose",        10, &NdtSlam::staticPoseCallback, this);
    gnss_pose_sub_          = nh_.subscribe("/gnss_pose",          10, &NdtSlam::gnssPoseCallback, this);
    odom_sub_               = nh_.subscribe("/odom",               points_queue_size*10, &NdtSlam::odometryCallback, this);
    //TODO need imu topic?

    points_raw_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "/points_raw", points_queue_size));
    points_filtered_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "/filtered_points", points_queue_size));
    points_synchronizer_.reset(new message_filters::Synchronizer<SyncPolicyPoints>(SyncPolicyPoints(10), *points_raw_sub_, *points_filtered_sub_));
    points_synchronizer_->registerCallback(boost::bind(&NdtSlam::pointsRawAndFilterdCallback, this, _1, _2));
}

NdtSlam::~NdtSlam()
{
    if(with_mapping_) {
        if(separate_mapping_) {
            localizer_ptr_->saveSeparateMap();
        }
        else {
            localizer_ptr_->saveSingleMap();
        }
    }
}

void NdtSlam::configCallback(const autoware_msgs::ConfigNdtSlam::ConstPtr& config_msg_ptr)
{
    const double current_time_sec = config_msg_ptr->header.stamp.toSec();

    localizer_ptr_->setStepSize(config_msg_ptr->step_size);
    localizer_ptr_->setTransformationEpsilon(config_msg_ptr->trans_epsilon);
    localizer_ptr_->setMaximumIterations(config_msg_ptr->max_iterations);

    //TODO need if?
    if(config_msg_ptr->resolution != localizer_ptr_->getResolution()) {
        localizer_ptr_->setResolution(config_msg_ptr->resolution);
    }

    init_pos_gnss_ = config_msg_ptr->init_pos_gnss;
    static Pose static_initial_pose = Pose(0, 0, 0, 0, 0, 0);
    const Pose initial_pose = Pose(config_msg_ptr->x, config_msg_ptr->y, config_msg_ptr->z, config_msg_ptr->roll, config_msg_ptr->pitch, config_msg_ptr->yaw);
    //TODO
    if(init_pos_gnss_ == false && static_initial_pose != initial_pose) {
        static_initial_pose = initial_pose;
        const auto initial_localizer_pose = transformToPose(initial_pose, tf_btol_);
        localizer_ptr_->updateManualPose(initial_localizer_pose, current_time_sec);
    }

    if(config_msg_ptr->with_mapping == true && with_mapping_ == false) {
        if(separate_mapping_) {
            localizer_ptr_->saveSeparateMap();
            localizer_ptr_->loadAroundMap();
        }
    }
    else if(config_msg_ptr->with_mapping == false && with_mapping_ == true) {
        if(separate_mapping_) {
            localizer_ptr_->saveSeparateMap();
        }
        else {
            localizer_ptr_->saveSingleMap();
        }
    }
    with_mapping_ = config_msg_ptr->with_mapping;

}

void NdtSlam::pointsMapUpdatedCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud2_msg_ptr)
{
    pcl::PointCloud<PointTarget> pointcloud;
    pcl::fromROSMsg(*pointcloud2_msg_ptr, pointcloud);
    localizer_ptr_->updatePointsMap(pointcloud.makeShared());
}

void NdtSlam::manualPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_cov_msg_ptr)
{
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = pose_cov_msg_ptr->header;
    pose_msg.pose = pose_cov_msg_ptr->pose.pose;

    geometry_msgs::PoseStamped transformed_pose_msg;
    try {
        tf_listener_.waitForTransform(map_frame_, pose_msg.header.frame_id, ros::Time(0), ros::Duration(1.0));
        tf_listener_.transformPose(map_frame_, ros::Time(0), pose_msg, pose_msg.header.frame_id, transformed_pose_msg);
    }
    catch (tf::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
    }

    const auto base_link_pose = convertFromROSMsg(transformed_pose_msg);
    const auto localizer_pose = transformToPose(base_link_pose, tf_btol_);
    const double current_time_sec = pose_cov_msg_ptr->header.stamp.toSec();
    localizer_ptr_->updateManualPose(localizer_pose, current_time_sec);
}

void NdtSlam::staticPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg_ptr)
{
    const auto base_link_pose = convertFromROSMsg(*pose_msg_ptr);
    const double current_time_sec = pose_msg_ptr->header.stamp.toSec();
    const auto localizer_pose = transformToPose(base_link_pose, tf_btol_);
    localizer_ptr_->updateStaticPose(localizer_pose, current_time_sec);
}

void NdtSlam::gnssPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg_ptr)
{
    if(init_pos_gnss_ == false) {
      return;
    }

    //TODO: transform tf_gnss to tf_map
    const auto gnss_pose = convertFromROSMsg(*pose_msg_ptr);
    const double current_time_sec = pose_msg_ptr->header.stamp.toSec();
    const auto localizer_pose = transformToPose(gnss_pose, tf_btol_);
    localizer_ptr_->updateGnssPose(localizer_pose, current_time_sec);
}

void NdtSlam::odometryCallback(const nav_msgs::Odometry::ConstPtr& odom_msg_ptr)
{
    if(use_odometry_ == false) {
        return;
    }

    const auto velocity = convertFromROSMsg(odom_msg_ptr->twist.twist);
    const double current_time_sec = odom_msg_ptr->header.stamp.toSec();
    localizer_ptr_->updateDeadReconing(velocity, current_time_sec);
    publishPosition(odom_msg_ptr->header.stamp);
}

void NdtSlam::pointsRawAndFilterdCallback(const sensor_msgs::PointCloud2::ConstPtr& points_raw_msg_ptr, const sensor_msgs::PointCloud2::ConstPtr& points_filtered_msg_ptr)
{
    static bool is_first_call = true;
    if(is_first_call && with_mapping_ && localizer_ptr_->getMapPtr() == nullptr) {
        is_first_call = false;
        pcl::PointCloud<PointTarget> points_raw;
        pcl::fromROSMsg(*points_raw_msg_ptr, points_raw);
        boost::shared_ptr< pcl::PointCloud<PointTarget> > points_raw_limilt_range(new pcl::PointCloud<PointTarget>);
        limitPointCloudRange(points_raw.makeShared(), points_raw_limilt_range, min_scan_range_, max_scan_range_);
        localizer_ptr_->setPointsMap(points_raw_limilt_range);
    }

    const double current_time_sec = points_filtered_msg_ptr->header.stamp.toSec();

    pcl::PointCloud<PointSource> points_filtered;
    pcl::fromROSMsg(*points_filtered_msg_ptr, points_filtered);
    const bool localizer_succeed = localizer_ptr_->updateLocalizer(points_filtered.makeShared(), current_time_sec);
    if(localizer_succeed == false) {
        return;
    }

    publishPosition(points_filtered_msg_ptr->header.stamp);
    publishLocalizerStatus(points_filtered_msg_ptr->header.stamp);

    if(with_mapping_) {

        const auto localizer_pose = localizer_ptr_->getLocalizerPose();
        static auto added_pose = localizer_pose;
        const double add_scan_shift_meter = std::sqrt(std::pow(localizer_pose.x - added_pose.x, 2.0) + std::pow(localizer_pose.y - added_pose.y, 2.0) + std::pow(localizer_pose.z - added_pose.z, 2.0));
        if(add_scan_shift_meter >= min_add_scan_shift_) {
            added_pose = localizer_pose;
            pcl::PointCloud<PointTarget> points_raw;
            pcl::fromROSMsg(*points_raw_msg_ptr, points_raw);
            boost::shared_ptr< pcl::PointCloud<PointTarget> > points_raw_limilt_range(new pcl::PointCloud<PointTarget>);
            limitPointCloudRange(points_raw.makeShared(), points_raw_limilt_range, min_scan_range_, max_scan_range_);
            localizer_ptr_->addMap(points_raw_limilt_range);
        }

        if(separate_mapping_) {
            const int map_x = std::floor(localizer_pose.x / localizer_ptr_->getSaveSeparateMapSize());
            const int map_y = std::floor(localizer_pose.y / localizer_ptr_->getSaveSeparateMapSize());
            static int prev_map_x = map_x;
            static int prev_map_y = map_x;

            if(map_x != prev_map_x || map_y != prev_map_y)
            {
                std::cout << "saveSeparateMap & loadAroundMap" << std::endl;
                const auto save_start = std::chrono::system_clock::now();
                localizer_ptr_->saveSeparateMap();
                const auto save_end = std::chrono::system_clock::now();
                const auto save_time = std::chrono::duration_cast<std::chrono::microseconds>(save_end - save_start).count() / 1000.0;
                std::cout << "save_time: " << save_time << "ms" << std::endl;

                const auto load_start = std::chrono::system_clock::now();
                localizer_ptr_->loadAroundMap();
                const auto load_end = std::chrono::system_clock::now();
                const auto load_time = std::chrono::duration_cast<std::chrono::microseconds>(load_end - load_start).count() / 1000.0;
                std::cout << "load_time: " << load_time << "ms" << std::endl;

                prev_map_x = map_x;
                prev_map_y = map_y;
            }
        }

        //std::cout << "map points size:" << localizer_ptr_->getMapPtr()->points.size() << std::endl;
        //TODO
        static int loop_count_donw_map = 0;
        if(++loop_count_donw_map >= 300) {
            loop_count_donw_map = 0;
            localizer_ptr_->downsampleMap();
        }

        //TODO timer
        static int loop_count_pub_map = 0;
        if(++loop_count_pub_map >= 30) {
            loop_count_pub_map = 0;
            if (points_map_region_pub_.getNumSubscribers() > 0) {
                publishPointsMap(points_raw_msg_ptr->header.stamp);
            }
        }
    }

    //TODO
    static int loop_count_pub_ndvoxel = 0;
    if(++loop_count_pub_ndvoxel >= 30) {
        loop_count_pub_ndvoxel = 0;
        if (ndvoxel_marker_pub_.getNumSubscribers() > 0) {
            publishNDVoxelMap(points_raw_msg_ptr->header.stamp);
        }
    }


    pcl::PointCloud<PointSource>::Ptr points_filtered_tmp(new pcl::PointCloud<PointSource>);
    for (const auto& point : points_filtered) {
      //TODO point.z >= 0.0
      if (point.z >= 0.0) {
        points_filtered_tmp->points.push_back(point);
      }
    }
    int nr = 0;

    const double score = localizer_ptr_->getFitnessScore(points_filtered_tmp, &nr, 1.0);
    std::cout << "FitnessScore: " << score << std::endl;
    const double ratio = (points_filtered_tmp->points.size() > 0) ? (static_cast<double>(nr) / static_cast<double>(points_filtered_tmp->points.size())) : 0;

    reliability_.setScore(ratio);

    std_msgs::Float32 localizer_score_msg;
    localizer_score_msg.data = ratio;
    localizer_score_pub_.publish(localizer_score_msg);

    // std_msgs::Float32 localizer_score_ave_msg;
    // localizer_score_ave_msg.data = reliability_.getAverage();
    // localizer_score_ave_pub_.publish(localizer_score_ave_msg);
    //
    // std_msgs::Float32 localizer_score_var_msg;
    // localizer_score_var_msg.data = reliability_.getVariance();
    // localizer_score_var_pub_.publish(localizer_score_var_msg);


    localizer_ptr_->writeLogFile();
}

void NdtSlam::publishPosition(ros::Time time_stamp)
{
    std_msgs::Header common_header;
    common_header.frame_id = map_frame_;
    common_header.stamp = time_stamp;

    const auto localizer_pose = localizer_ptr_->getLocalizerPose();
    const auto base_link_pose = transformToPose(localizer_pose, tf_btol_.inverse());
    const auto base_link_pose_msg = convertToROSMsg(common_header, base_link_pose);
    localizer_pose_pub_.publish(base_link_pose_msg);  //TODO: rename publisher?
    //TODO low-pass filter

    auto localizer_velocity = localizer_ptr_->getLocalizerVelocity();
    geometry_msgs::TwistStamped localizer_twist_msg;
    localizer_twist_msg.header = common_header;
    localizer_twist_msg.header.frame_id = base_link_frame_;
    localizer_twist_msg.twist.linear.x = std::sqrt(std::pow(localizer_velocity.linear.x, 2.0) + std::pow(localizer_velocity.linear.y, 2.0) + std::pow(localizer_velocity.linear.z, 2.0));
    localizer_twist_msg.twist.linear.y = 0;
    localizer_twist_msg.twist.linear.z = 0;
    localizer_twist_msg.twist.angular.x = 0;
    localizer_twist_msg.twist.angular.y = 0;
    localizer_twist_msg.twist.angular.z = localizer_velocity.angular.z;
    localizer_twist_pub_.publish(localizer_twist_msg);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(base_link_pose.x, base_link_pose.y, base_link_pose.z));
    tf::Quaternion tf_quaternion;
    tf_quaternion.setRPY(base_link_pose.roll, base_link_pose.pitch, base_link_pose.yaw);
    transform.setRotation(tf_quaternion);
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform, common_header.stamp, map_frame_, base_link_frame_));
}

void NdtSlam::publishPointsMap(ros::Time time_stamp)
{
    const auto map = localizer_ptr_->getMapPtr();
    if(map->points.size() <= 0) {
      return;
    }
    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(*map, map_msg);

    map_msg.header.frame_id = map_frame_;
    map_msg.header.stamp = time_stamp;
    points_map_region_pub_.publish(map_msg);
}

void NdtSlam::publishNDVoxelMap(ros::Time time_stamp)
{
    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker ndt_marker_common;
    ndt_marker_common.header.frame_id = "/map";
    ndt_marker_common.header.stamp =time_stamp;
    ndt_marker_common.type = visualization_msgs::Marker::SPHERE;
    ndt_marker_common.action = visualization_msgs::Marker::ADD;
    ndt_marker_common.ns = "NDVoxel";
    ndt_marker_common.color.a = 0.2;
    ndt_marker_common.color.r = 0.0;
    ndt_marker_common.color.g = 1.0;
    ndt_marker_common.color.b = 0.0;

    int id = 0;
    std::vector<Eigen::Vector3d> centroid_array = localizer_ptr_->getCentroid();
    std::vector<Eigen::Matrix3d> covariance_array = localizer_ptr_->getCovariance();
    for(int i = 0; i < covariance_array.size(); ++i)
    {
        if((covariance_array)[i](0) < 0.0001 && (covariance_array)[i](5) < 0.0001 && (covariance_array)[i](9) < 0.0001) {
            continue;
        }

        visualization_msgs::Marker ndvoxel_marker_tmp;
        ndvoxel_marker_tmp = ndt_marker_common;

        //Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es((covariance_array)[i]);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es((covariance_array)[i].inverse());
        auto el = es.eigenvalues();
        if(el(0) < 0)
          continue;
        el = el.normalized();
        auto ev = es.eigenvectors().col(2);

        const double x_2 = 1.5; //TODO
        ndvoxel_marker_tmp.scale.x = sqrt(x_2*el(2));
        ndvoxel_marker_tmp.scale.y = sqrt(x_2*el(1));
        ndvoxel_marker_tmp.scale.z = sqrt(x_2*el(0));
        ndvoxel_marker_tmp.id = id++;
        ndvoxel_marker_tmp.frame_locked = true;

        ndvoxel_marker_tmp.pose.position.x = (centroid_array)[i](0);
        ndvoxel_marker_tmp.pose.position.y = (centroid_array)[i](1);
        ndvoxel_marker_tmp.pose.position.z = (centroid_array)[i](2);

        double xy_theta = std::atan2(ev(1), ev(0));
        double xz_theta = std::atan2(ev(2), ev(0));

        //TODO
        ndvoxel_marker_tmp.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(xz_theta, 0, xy_theta);;
        marker_array.markers.push_back(ndvoxel_marker_tmp);
    }

    // std::vector<Eigen::Vector3d> centroid_array = localizer_ptr_->getCentroid();
    // std::vector<Eigen::Vector3d> eval_array = localizer_ptr_->getEval();
    // std::vector<Eigen::Matrix3d> evec_array = localizer_ptr_->getEvec();
    // for(int i = 0; i < centroid_array.size(); ++i)
    // {
    //     visualization_msgs::Marker ndvoxel_marker_tmp;
    //     ndvoxel_marker_tmp = ndt_marker_common;
    //
    //     auto el = eval_array[i].normalized();
    //     auto ev1 = evec_array[i].col(2);
    //     auto ev2 = evec_array[i].col(1);
    //     auto ev3 = evec_array[i].col(0);
    //     if(std::isinf(el(0)) || std::isinf(el(1)) || std::isinf(el(2))
    //     || std::isnan(el(0)) || std::isnan(el(1)) || std::isnan(el(2))
    //     || std::isinf(ev1(0)) || std::isinf(ev1(1)) || std::isinf(ev1(2))
    //     || std::isnan(ev1(0)) || std::isnan(ev1(1)) || std::isnan(ev1(2)))
    //       continue;
    //
    //     const double x_2 = 1.5; //TODO
    //     ndvoxel_marker_tmp.scale.x = sqrt(x_2*el(2));
    //     ndvoxel_marker_tmp.scale.y = sqrt(x_2*el(1));
    //     ndvoxel_marker_tmp.scale.z = sqrt(x_2*el(0));
    //     ndvoxel_marker_tmp.id = id++;
    //     ndvoxel_marker_tmp.frame_locked = true;
    //
    //     ndvoxel_marker_tmp.pose.position.x = (centroid_array)[i](0);
    //     ndvoxel_marker_tmp.pose.position.y = (centroid_array)[i](1);
    //     ndvoxel_marker_tmp.pose.position.z = (centroid_array)[i](2);
    //
    //     auto ev12 = ev1.cross(ev2);
    //     double xz_theta = 0;
    //     double yz_theta = std::atan2(ev12(2), ev12(1));
    //     double xy_theta = std::atan2(ev12(1), ev12(0));
    //
    //     //TODO
    //     ndvoxel_marker_tmp.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(xz_theta, yz_theta, xy_theta);;
    //     marker_array.markers.push_back(ndvoxel_marker_tmp);
    // }

    ndvoxel_marker_pub_.publish(marker_array);

}

void NdtSlam::publishLocalizerStatus(ros::Time time_stamp)
{
}
