/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "ndt_slam_core.h"

#include <iomanip>
#include <pcl_conversions/pcl_conversions.h>

#include <lidar_localizer/util/data_structs.h>
#include <lidar_localizer/util/convert_ros_msgs.h>
#include <lidar_localizer/util/util_functions.h>

#include <visualization_msgs/MarkerArray.h>

#include <boost/filesystem.hpp>

NdtSlam::NdtSlam(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(nh)
    , private_nh_(private_nh)
    , method_type_(MethodType::PCL_GENERIC)
    , with_mapping_(false)
    , separate_mapping_(false)
    , use_nn_point_z_when_initial_pose_(false)
    , sensor_frame_("/velodyne")
    , base_link_frame_("/base_link")
    , map_frame_("/map")
    , min_scan_range_(5.0)
    , max_scan_range_(200.0)
    , min_add_scan_shift_(1.0)

{
    int method_type_tmp = 0;
    private_nh.getParam("method_type", method_type_tmp);
    method_type_ = static_cast<MethodType>(method_type_tmp);

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
        ROS_INFO("unkonwn method_type. use NDT SLAM PCL GENERIC version");
        localizer_ptr_.reset(new NdtSlamPCL<PointSource, PointTarget>);
    }

    int points_queue_size = 1;
    private_nh_.getParam("points_queue_size", points_queue_size);
    points_queue_size = points_queue_size <= 0 ? 1 : points_queue_size;
    ROS_INFO("points_queue_size: %d", points_queue_size);

    private_nh_.getParam("sensor_frame", sensor_frame_);
    private_nh_.getParam("base_link_frame", base_link_frame_);
    private_nh_.getParam("map_frame", map_frame_);
    ROS_INFO("sensor_frame_id: %s, base_link_frame_id: %s, map_frame_id: %s", sensor_frame_.c_str(), base_link_frame_.c_str(), map_frame_.c_str());

    private_nh_.getParam("init_x", init_pose_stamped_.pose.x);
    private_nh_.getParam("init_y", init_pose_stamped_.pose.y);
    private_nh_.getParam("init_z", init_pose_stamped_.pose.z);
    private_nh_.getParam("init_roll", init_pose_stamped_.pose.roll);
    private_nh_.getParam("init_pitch", init_pose_stamped_.pose.pitch);
    private_nh_.getParam("init_yaw", init_pose_stamped_.pose.yaw);

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
    map_manager_.setSaveMapLeafSize(save_map_leaf_size);
    map_manager_.setSaveSeparateMapSize(separate_map_size);
    ROS_INFO("with_mapping: %d, save_map_leaf_size: %lf, min_scan_range: %lf, max_scan_range: %lf, min_add_scan_shift: %lf", with_mapping_, save_map_leaf_size, min_scan_range_, max_scan_range_, min_add_scan_shift_);
    ROS_INFO("separate_mapping: %d, separate_map_size: %lf", separate_mapping_, separate_map_size);

    private_nh_.getParam("use_nn_point_z_when_initial_pose", use_nn_point_z_when_initial_pose_);
    ROS_INFO("use_nn_point_z_when_initial_pose: %d", use_nn_point_z_when_initial_pose_);

    const std::time_t now = std::time(NULL);
    const std::tm* pnow = std::localtime(&now);
    char buffer[80];
    std::strftime(buffer, 80, "%Y%m%d_%H%M%S", pnow);

    log_file_directory_path_ = "/tmp/Autoware/log/ndt_slam/" + std::string(buffer);
    boost::filesystem::create_directories(boost::filesystem::path(log_file_directory_path_));

    std::string map_file_path = log_file_directory_path_ + "/map";
    boost::filesystem::create_directories(boost::filesystem::path(map_file_path));
    map_manager_.setFileDirectoryPath(map_file_path);

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
    }

    ROS_INFO("tf(x, y, z, roll, pitch, yaw): %lf, %lf, %lf, %lf, %lf, %lf", tf_pose.x, tf_pose.y, tf_pose.z, tf_pose.roll, tf_pose.pitch, tf_pose.yaw);
    tf_btol_ = convertToEigenMatrix4f(tf_pose);

    points_map_pub_     = nh_.advertise<sensor_msgs::PointCloud2>("ndt_map", 10);
    ndt_pose_pub_       = nh_.advertise<geometry_msgs::PoseStamped>("ndt_pose", 10);
    localizer_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("localizer_pose", 10);
    estimate_twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("estimate_twist", 10);

    config_sub_       = nh_.subscribe("/config/ndtslam", 1, &NdtSlam::configCallback, this);
    points_map_sub_   = nh_.subscribe("/points_map", 1, &NdtSlam::pointsMapUpdatedCallback, this);
    initial_pose_sub_ = nh_.subscribe("/initialpose", points_queue_size*100, &NdtSlam::initialPoseCallback, this);

    mapping_points_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "/points_raw", points_queue_size));
    localizing_points_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "/filtered_points", points_queue_size));
    points_synchronizer_.reset(new message_filters::Synchronizer<SyncPolicyPoints>(SyncPolicyPoints(10), *mapping_points_sub_, *localizing_points_sub_));
    points_synchronizer_->registerCallback(boost::bind(&NdtSlam::mappingAndLocalizingPointsCallback, this, _1, _2));
}

NdtSlam::~NdtSlam()
{
    if(with_mapping_) {
        if(separate_mapping_) {
            map_manager_.downsampleMapThread();
            map_manager_.saveSeparateMapThread();
        }
        else {
            map_manager_.downsampleMapThread();
            map_manager_.saveSingleMapThread();
        }
    }
}

void NdtSlam::configCallback(const autoware_config_msgs::ConfigNDTSlam::ConstPtr& config_msg_ptr)
{
    const Pose initial_pose = Pose(config_msg_ptr->init_x, config_msg_ptr->init_y, config_msg_ptr->init_z, config_msg_ptr->init_roll, config_msg_ptr->init_pitch, config_msg_ptr->init_yaw);
    if(init_pose_stamped_.pose != initial_pose) {
      init_pose_stamped_.pose = initial_pose;
      pose_interpolator_.clearPoseStamped();
    }

    localizer_ptr_->setStepSize(config_msg_ptr->step_size);
    localizer_ptr_->setTransformationEpsilon(config_msg_ptr->trans_epsilon);
    localizer_ptr_->setMaximumIterations(config_msg_ptr->max_iterations);
    localizer_ptr_->setResolution(config_msg_ptr->resolution);

    if(config_msg_ptr->with_mapping == true && with_mapping_ == false) {
        if(separate_mapping_) {
            map_manager_.downsampleMapThread();
            map_manager_.saveSeparateMapThread();
            map_manager_.loadAroundMapThread(localizer_ptr_->getLocalizerPose());
        }
    }
    else if(config_msg_ptr->with_mapping == false && with_mapping_ == true) {
        if(separate_mapping_) {
            map_manager_.downsampleMapThread();
            map_manager_.saveSeparateMapThread();

        }
        else {
            map_manager_.downsampleMapThread();
            map_manager_.saveSingleMapThread();
        }
    }
    with_mapping_ = config_msg_ptr->with_mapping;

}

void NdtSlam::pointsMapUpdatedCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud2_msg_ptr)
{
    pcl::PointCloud<PointTarget> pointcloud;
    pcl::fromROSMsg(*pointcloud2_msg_ptr, pointcloud);
    map_manager_.setMap(pointcloud.makeShared());
}

void NdtSlam::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_conv_msg_ptr)
{
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = pose_conv_msg_ptr->header;
    pose_msg.pose = pose_conv_msg_ptr->pose.pose;

    geometry_msgs::PoseStamped transformed_pose_msg;
    if(pose_msg.header.frame_id != map_frame_) {
        try {
            tf_listener_.waitForTransform(map_frame_, pose_msg.header.frame_id, ros::Time(0), ros::Duration(1.0));
            tf_listener_.transformPose(map_frame_, ros::Time(0), pose_msg, pose_msg.header.frame_id, transformed_pose_msg);
        }
        catch (tf::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
        }
    }
    else {
        transformed_pose_msg = pose_msg;
    }
    auto base_link_pose = convertFromROSMsg(transformed_pose_msg);

    if(use_nn_point_z_when_initial_pose_){
      const auto map_ptr = map_manager_.getMap();
      if(map_ptr != nullptr) {
        double min_distance = DBL_MAX;
        double nearest_z = base_link_pose.z;
        for(const auto& p : map_ptr->points) {
          double distance = hypot(base_link_pose.x - p.x, base_link_pose.y - p.y);
          if(distance < min_distance) {
            min_distance = distance;
            nearest_z = p.z;
          }
        }
        base_link_pose.z = nearest_z;
      }
    }

    const double current_time_sec = transformed_pose_msg.header.stamp.toSec();

    init_pose_stamped_ = PoseStamped(base_link_pose, current_time_sec);
    pose_interpolator_.clearPoseStamped();
}


void NdtSlam::staticPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg_ptr)
{
    const auto base_link_pose = convertFromROSMsg(*pose_msg_ptr);
    const double current_time_sec = pose_msg_ptr->header.stamp.toSec();
    init_pose_stamped_ = PoseStamped(base_link_pose, current_time_sec);
    pose_interpolator_.clearPoseStamped();
}

void NdtSlam::mappingAndLocalizingPointsCallback(const sensor_msgs::PointCloud2::ConstPtr& mapping_points_msg_ptr, const sensor_msgs::PointCloud2::ConstPtr& localizing_points_msg_ptr)
{
    const double current_time_sec = localizing_points_msg_ptr->header.stamp.toSec();

    static bool is_first_call = true;
    if(is_first_call && with_mapping_ && map_manager_.getMap() == nullptr) {
        is_first_call = false;
        boost::shared_ptr< pcl::PointCloud<PointTarget> > mapping_points_ptr(new pcl::PointCloud<PointTarget>);
        pcl::fromROSMsg(*mapping_points_msg_ptr, *mapping_points_ptr);
        boost::shared_ptr< pcl::PointCloud<PointTarget> > mapping_points_limilt_range(new pcl::PointCloud<PointTarget>);
        limitPointCloudRange(mapping_points_ptr, mapping_points_limilt_range, min_scan_range_, max_scan_range_);

        const auto localizer_pose = transformToPose(init_pose_stamped_.pose, tf_btol_);
        const auto eigen_pose = convertToEigenMatrix4f(localizer_pose);
        pcl::transformPointCloud(*mapping_points_limilt_range, *mapping_points_limilt_range, eigen_pose);

        map_manager_.setMap(mapping_points_limilt_range);
    }

    if(map_manager_.getMap() == nullptr) {
        ROS_WARN("received points. But map is not loaded");
        return;
    }

    localizer_ptr_->updatePointsMap(map_manager_.getMap());

    Pose predict_base_link_pose;
    if(pose_interpolator_.isNotSetPoseStamped()) {
        predict_base_link_pose = init_pose_stamped_.pose;
    }
    else {
        predict_base_link_pose = pose_interpolator_.getInterpolatePoseStamped(current_time_sec).pose;
    }
    const Pose predict_localizer_pose = transformToPose(predict_base_link_pose, tf_btol_);

    pcl::PointCloud<PointSource> localizing_points;
    pcl::fromROSMsg(*localizing_points_msg_ptr, localizing_points);
    const bool align_succeed = localizer_ptr_->alignMap(localizing_points.makeShared(), predict_localizer_pose);
    if(align_succeed == false) {
          return;
    }

    const auto localizer_pose = localizer_ptr_->getLocalizerPose();
    const auto base_link_pose = transformToPose(localizer_pose, tf_btol_.inverse());
    PoseStamped base_link_pose_stamped(base_link_pose, current_time_sec);
    if(pose_interpolator_.isNotSetPoseStamped()) {
        pose_interpolator_.pushbackPoseStamped(base_link_pose_stamped);
        pose_interpolator_.pushbackPoseStamped(base_link_pose_stamped);
    }
    else {
        pose_interpolator_.pushbackPoseStamped(base_link_pose_stamped);
    }

    if(with_mapping_) {
        boost::shared_ptr< pcl::PointCloud<PointTarget> > mapping_points_ptr(new pcl::PointCloud<PointTarget>);
        pcl::fromROSMsg(*mapping_points_msg_ptr, *mapping_points_ptr);
        mapping(mapping_points_ptr);
        publishPointsMap(mapping_points_msg_ptr->header.stamp);
    }

    publishPosition(localizing_points_msg_ptr->header.stamp);
    publishVelocity(localizing_points_msg_ptr->header.stamp);
    publishTF(localizing_points_msg_ptr->header.stamp);

    std::cout << "------------------------------------------------" << std::endl;
    std::cout << "base_link_pose " << base_link_pose << std::endl;
    std::cout << "velocity: " << pose_interpolator_.getVelocity() << std::endl;
    std::cout << "align_time: " << localizer_ptr_->getAlignTime() << "ms" << std::endl;

}

void NdtSlam::mapping(const boost::shared_ptr< pcl::PointCloud<PointTarget> >& mapping_points_ptr)
{
    const auto localizer_pose = localizer_ptr_->getLocalizerPose();
    static auto added_pose = localizer_pose;
    const double add_scan_shift_meter = std::sqrt(std::pow(localizer_pose.x - added_pose.x, 2.0) + std::pow(localizer_pose.y - added_pose.y, 2.0) + std::pow(localizer_pose.z - added_pose.z, 2.0));
    if(add_scan_shift_meter >= min_add_scan_shift_) {
        added_pose = localizer_pose;
        const boost::shared_ptr< pcl::PointCloud<PointTarget> > mapping_points_limilt_range(new pcl::PointCloud<PointTarget>);
        limitPointCloudRange(mapping_points_ptr, mapping_points_limilt_range, min_scan_range_, max_scan_range_);
        const auto localizer_pose = localizer_ptr_->getLocalizerPose();
        const auto eigen_pose = convertToEigenMatrix4f(localizer_pose);
        pcl::transformPointCloud(*mapping_points_limilt_range, *mapping_points_limilt_range, eigen_pose);
        map_manager_.addPointCloudMapThread(mapping_points_limilt_range);
    }

    if(separate_mapping_) {
        const int map_x = std::floor(localizer_pose.x / map_manager_.getSaveSeparateMapSize());
        const int map_y = std::floor(localizer_pose.y / map_manager_.getSaveSeparateMapSize());
        static int prev_map_x = map_x;
        static int prev_map_y = map_x;

        if(map_x != prev_map_x || map_y != prev_map_y)
        {
            map_manager_.downsampleMapThread();
            map_manager_.saveSeparateMapThread();
            map_manager_.loadAroundMapThread(localizer_pose);

            prev_map_x = map_x;
            prev_map_y = map_y;
        }
    }

    const int loop_count_donw_map_max = 300;
    static int loop_count_donw_map = 0;
    if(++loop_count_donw_map >= loop_count_donw_map_max) {
        loop_count_donw_map = 0;
        map_manager_.downsampleMapThread();
    }
}

void NdtSlam::publishPosition(const ros::Time& time_stamp)
{
    std_msgs::Header common_header;
    common_header.frame_id = map_frame_;
    common_header.stamp = time_stamp;

    const auto localizer_pose = localizer_ptr_->getLocalizerPose();
    const auto localizer_pose_msg = convertToROSMsg(common_header, localizer_pose);
    localizer_pose_pub_.publish(localizer_pose_msg);

    const auto base_link_pose = transformToPose(localizer_pose, tf_btol_.inverse());
    const auto base_link_pose_msg = convertToROSMsg(common_header, base_link_pose);
    ndt_pose_pub_.publish(base_link_pose_msg);
}

void NdtSlam::publishVelocity(const ros::Time& time_stamp)
{
    std_msgs::Header common_header;
    common_header.frame_id = map_frame_;
    common_header.stamp = time_stamp;

    const auto base_link_velocity = pose_interpolator_.getVelocity();
    const auto trans_current_pose = convertPoseIntoRelativeCoordinate(pose_interpolator_.getCurrentPoseStamped().pose, pose_interpolator_.getPrevPoseStamped().pose);
    double v = std::sqrt(std::pow(base_link_velocity.linear.x, 2.0) + std::pow(base_link_velocity.linear.y, 2.0) + std::pow(base_link_velocity.linear.z, 2.0));
    v = trans_current_pose.x >= 0 ? v : -v;
    double w = base_link_velocity.angular.z;


    geometry_msgs::TwistStamped estimate_twist_msg;
    estimate_twist_msg.header = common_header;
    estimate_twist_msg.header.frame_id = base_link_frame_;
    estimate_twist_msg.twist.linear.x = v;
    estimate_twist_msg.twist.linear.y = 0;
    estimate_twist_msg.twist.linear.z = 0;
    estimate_twist_msg.twist.angular.x = 0;
    estimate_twist_msg.twist.angular.y = 0;
    estimate_twist_msg.twist.angular.z = w;
    estimate_twist_pub_.publish(estimate_twist_msg);
}


void NdtSlam::publishTF(const ros::Time& time_stamp)
{
    const auto localizer_pose = localizer_ptr_->getLocalizerPose();
    const auto base_link_pose = transformToPose(localizer_pose, tf_btol_.inverse());

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(base_link_pose.x, base_link_pose.y, base_link_pose.z));
    tf::Quaternion tf_quaternion;
    tf_quaternion.setRPY(base_link_pose.roll, base_link_pose.pitch, base_link_pose.yaw);
    transform.setRotation(tf_quaternion);
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform, time_stamp, map_frame_, base_link_frame_));
}

void NdtSlam::publishPointsMap(const ros::Time& time_stamp)
{
    if (points_map_pub_.getNumSubscribers() <= 0) {
      return;
    }

    const int loop_count_max = 30;
    static int loop_count = 0;
    if(++loop_count < loop_count_max) {
      return;
    }
    loop_count = 0;

    const auto map_ptr = map_manager_.getMap();
    if(map_ptr == nullptr || map_ptr->points.size() == 0) {
      return;
    }

    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(*map_ptr, map_msg);

    map_msg.header.frame_id = map_frame_;
    map_msg.header.stamp = time_stamp;
    points_map_pub_.publish(map_msg);
}
