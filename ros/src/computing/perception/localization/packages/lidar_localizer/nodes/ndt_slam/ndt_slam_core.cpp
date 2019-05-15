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

#include <cmath>
#include <iomanip>
#include <algorithm>
#include <thread>

#include <pcl_conversions/pcl_conversions.h>

#include <autoware_lidar_localizer/util/convert_ros_msgs.h>
#include <autoware_lidar_localizer/util/data_structs.h>
#include <autoware_lidar_localizer/util/util_functions.h>

#include <boost/filesystem.hpp>

NdtSlam::NdtSlam(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(nh), private_nh_(private_nh), tf2_listener_(tf2_buffer_),
      with_mapping_(false), separate_mapping_(false),
      use_nn_point_z_when_initial_pose_(false), publish_tf_(true), sensor_frame_("velodyne"),
      target_frame_("base_link"), map_frame_("map"), world_frame_("map"),
      min_scan_range_(5.0), max_scan_range_(200.0), min_add_scan_shift_(1.0),
      matching_score_use_points_num_(300), matching_score_cutoff_lower_limit_z_(0.2),
      matching_score_cutoff_upper_limit_z_(2.0), matching_score_cutoff_lower_limit_range_(5.0),
      matching_score_cutoff_upper_limit_range_(100.0), matching_score_(0.0)

{
  ROS_INFO("use NDT SLAM PCL GENERIC version");
  localizer_ptr_.reset(new NdtSlamPCL<PointSource, PointTarget>);

  int points_queue_size = 1;
  private_nh_.getParam("points_queue_size", points_queue_size);
  points_queue_size = points_queue_size <= 0 ? 1 : points_queue_size;
  ROS_INFO("points_queue_size: %d", points_queue_size);

  private_nh_.getParam("target_frame", target_frame_);
  private_nh_.getParam("world_frame", world_frame_);
  ROS_INFO("target_frame_id: %s, world_frame: %s",
            target_frame_.c_str(), world_frame_.c_str());

  private_nh_.getParam("init_x", init_pose_stamped_.pose.x);
  private_nh_.getParam("init_y", init_pose_stamped_.pose.y);
  private_nh_.getParam("init_z", init_pose_stamped_.pose.z);
  private_nh_.getParam("init_roll", init_pose_stamped_.pose.roll);
  private_nh_.getParam("init_pitch", init_pose_stamped_.pose.pitch);
  private_nh_.getParam("init_yaw", init_pose_stamped_.pose.yaw);

  double trans_epsilon = localizer_ptr_->getTransformationEpsilon();
  double step_size = localizer_ptr_->getStepSize();
  double resolution = localizer_ptr_->getResolution();
  int max_iterations = localizer_ptr_->getMaximumIterations();
  private_nh_.getParam("trans_epsilon", trans_epsilon);
  private_nh_.getParam("step_size", step_size);
  private_nh_.getParam("resolution", resolution);
  private_nh_.getParam("max_iterations", max_iterations);
  localizer_ptr_->setTransformationEpsilon(trans_epsilon);
  localizer_ptr_->setStepSize(step_size);
  localizer_ptr_->setResolution(resolution);
  localizer_ptr_->setMaximumIterations(max_iterations);
  ROS_INFO("trans_epsilon: %lf, step_size: %lf, resolution: %lf, max_iterations: %d",
            trans_epsilon, step_size, resolution, max_iterations);

  double save_map_leaf_size = map_manager_.getSaveMapLeafSize();
  double separate_map_size = map_manager_.getSaveSeparateMapSize();
  bool save_added_map = map_manager_.getSaveAddedMap();
  private_nh_.getParam("with_mapping", with_mapping_);
  private_nh_.getParam("save_added_map", save_added_map);
  private_nh_.getParam("save_map_leaf_size", save_map_leaf_size);
  private_nh_.getParam("min_scan_range", min_scan_range_);
  private_nh_.getParam("max_scan_range", max_scan_range_);
  private_nh_.getParam("min_add_scan_shift", min_add_scan_shift_);
  private_nh_.getParam("separate_mapping", separate_mapping_);
  private_nh_.getParam("separate_map_size", separate_map_size);
  map_manager_.setSaveMapLeafSize(save_map_leaf_size);
  map_manager_.setSaveSeparateMapSize(separate_map_size);
  map_manager_.setSaveAddedMap(save_added_map);
  ROS_INFO("with_mapping: %d, save_added_map: %d, save_map_leaf_size: %lf, min_scan_range: %lf, max_scan_range: %lf, min_add_scan_shift: %lf",
            with_mapping_,    save_added_map,     save_map_leaf_size,      min_scan_range_,     max_scan_range_,     min_add_scan_shift_);
  ROS_INFO("separate_mapping: %d, separate_map_size: %lf", separate_mapping_, separate_map_size);


  private_nh.getParam("matching_score_use_points_num", matching_score_use_points_num_);
  private_nh.getParam("matching_score_cutoff_lower_limit_z", matching_score_cutoff_lower_limit_z_);
  private_nh.getParam("matching_score_cutoff_upper_limit_z", matching_score_cutoff_upper_limit_z_);
  private_nh.getParam("matching_score_cutoff_lower_limit_range", matching_score_cutoff_lower_limit_range_);
  private_nh.getParam("matching_score_cutoff_upper_limit_range", matching_score_cutoff_upper_limit_range_);
  ROS_INFO("matching_score_use_points_num: %d, matching_score_cutoff_lower_limit_z: %lf, matching_score_cutoff_upper_limit_z: %lf, matching_score_cutoff_lower_limit_range: %lf, matching_score_cutoff_upper_limit_range: %lf",
            matching_score_use_points_num_,    matching_score_cutoff_lower_limit_z_,     matching_score_cutoff_upper_limit_z_,     matching_score_cutoff_lower_limit_range_,     matching_score_cutoff_upper_limit_range_);

  double matching_score_kT = matching_score_class_.getFermikT();
  private_nh.getParam("matching_score_kT", matching_score_kT);
  matching_score_class_.setFermikT(matching_score_kT);

  double matching_score_mu = matching_score_class_.getFermiMu();
  private_nh.getParam("matching_score_mu", matching_score_mu);
  matching_score_class_.setFermiMu(matching_score_mu);


  private_nh_.getParam("use_nn_point_z_when_initial_pose", use_nn_point_z_when_initial_pose_);
  ROS_INFO("use_nn_point_z_when_initial_pose: %d", use_nn_point_z_when_initial_pose_);

  bool use_fusion_localizer_feedback = false;
  private_nh_.getParam("use_fusion_localizer_feedback", use_fusion_localizer_feedback);
  ROS_INFO("use_fusion_localizer_feedback: %d", use_fusion_localizer_feedback);

  private_nh_.getParam("publish_tf", publish_tf_);
  ROS_INFO("publish_tf: %d", publish_tf_);

  std::string mapping_file_directory_path = "/tmp/Autoware/log/ndt_slam";
  private_nh_.getParam("mapping_file_directory_path", mapping_file_directory_path);
  ROS_INFO("mapping_file_directory_path: %s", mapping_file_directory_path.c_str());

  const std::time_t now = std::time(NULL);
  const std::tm *pnow = std::localtime(&now);
  char buffer[80];
  std::strftime(buffer, 80, "%Y%m%d_%H%M%S", pnow);

  mapping_file_directory_path += std::string("/") + std::string(buffer);
  boost::filesystem::create_directories(boost::filesystem::path(mapping_file_directory_path));
  map_manager_.setFileDirectoryPath(mapping_file_directory_path);

  points_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("ndt_map", 10);
  ndt_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("ndt_pose", 10);
  ndt_pose_with_covariance_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("ndt_pose_with_covariance", 10);
  predict_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("predict_pose", 10);
  sensor_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("localizer_pose", 10);
  estimate_twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("estimate_twist", 10);
  matching_score_pub_ = nh.advertise<std_msgs::Float32>("matching_score", 10);
  matching_score_histogram_pub_ = nh.advertise<jsk_recognition_msgs::HistogramWithRange>("nearest_points_histogram", 10);
  matching_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("matching_points", 10);
  unmatching_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("unmatching_points", 10);
  time_ndt_matching_pub_ = nh.advertise<std_msgs::Float32>("time_ndt_matching", 10);

  config_sub_ = nh_.subscribe("/config/ndt_slam", 1, &NdtSlam::configCallback, this);
  points_map_sub_ = nh_.subscribe("/points_map", 1, &NdtSlam::pointsMapUpdatedCallback, this);
  initial_pose_sub_ = nh_.subscribe("/initialpose", 1, &NdtSlam::initialPoseCallback, this);

  mapping_points_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "/points_raw", points_queue_size));
  localizing_points_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "/filtered_points", points_queue_size));
  current_pose_sub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, "/ekf_pose", points_queue_size*1000));

  if(use_fusion_localizer_feedback) {
      points_and_pose_synchronizer_.reset(new message_filters::Synchronizer<SyncPolicyPointsAndPose>(SyncPolicyPointsAndPose(points_queue_size*1000), *mapping_points_sub_, *localizing_points_sub_, *current_pose_sub_));
      points_and_pose_synchronizer_->registerCallback(boost::bind(&NdtSlam::mappingAndLocalizingPointsAndCurrentPoseCallback, this, _1, _2, _3));
  }
  else {
      points_synchronizer_.reset(new message_filters::Synchronizer<SyncPolicyPoints>(SyncPolicyPoints(10), *mapping_points_sub_, *localizing_points_sub_));
      points_synchronizer_->registerCallback(boost::bind(&NdtSlam::mappingAndLocalizingPointsCallback, this, _1, _2));
  }

}

NdtSlam::~NdtSlam() {
  if (with_mapping_) {
    if (separate_mapping_) {
      map_manager_.saveSeparateMapThread();
    }
    else {
      map_manager_.saveSingleMapThread();
    }
  }
}

void NdtSlam::configCallback(const autoware_config_msgs::ConfigNDTSlam::ConstPtr &config_msg_ptr) {

  static bool is_first_call = false;
  if(is_first_call) {
    is_first_call = true;
    init_pose_stamped_.pose = Pose(config_msg_ptr->init_x, config_msg_ptr->init_y,
               config_msg_ptr->init_z, config_msg_ptr->init_roll,
               config_msg_ptr->init_pitch, config_msg_ptr->init_yaw);
    pose_interpolator_.clearPoseStamped();
  }

  localizer_ptr_->setStepSize(config_msg_ptr->step_size);
  localizer_ptr_->setTransformationEpsilon(config_msg_ptr->trans_epsilon);
  localizer_ptr_->setMaximumIterations(config_msg_ptr->max_iterations);
  localizer_ptr_->setResolution(config_msg_ptr->resolution);

  if (config_msg_ptr->with_mapping == true && with_mapping_ == false) {
    if (separate_mapping_) {
      map_manager_.saveSeparateMap();
      map_manager_.loadAroundMap(localizer_ptr_->getLocalizerPose());
    }
  }
  else if (config_msg_ptr->with_mapping == false && with_mapping_ == true) {
    if (separate_mapping_) {
      map_manager_.saveSeparateMapThread();
    }
    else {
      map_manager_.saveSingleMapThread();
    }
  }
  with_mapping_ = config_msg_ptr->with_mapping;
}

void NdtSlam::pointsMapUpdatedCallback(const sensor_msgs::PointCloud2::ConstPtr &pointcloud2_msg_ptr) {

  map_frame_ = pointcloud2_msg_ptr->header.frame_id;

  pcl::PointCloud<PointTarget> pointcloud;
  pcl::fromROSMsg(*pointcloud2_msg_ptr, pointcloud);
  map_manager_.setMap(pointcloud.makeShared());

  if(with_mapping_ && separate_mapping_) {
      map_manager_.saveSeparateMap();
      map_manager_.loadAroundMap(localizer_ptr_->getLocalizerPose());
  }
}

void NdtSlam::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_conv_msg_ptr) {

  geometry_msgs::PoseStamped init_pose_msg;
  init_pose_msg.header = pose_conv_msg_ptr->header;
  init_pose_msg.pose = pose_conv_msg_ptr->pose.pose;

  geometry_msgs::TransformStamped::Ptr trans_map_to_init_pose_ptr(new geometry_msgs::TransformStamped);
  getTransform(map_frame_, init_pose_msg.header.frame_id, trans_map_to_init_pose_ptr);
  geometry_msgs::PoseStamped mapTF_init_pose_msg;
  tf2::doTransform(init_pose_msg, mapTF_init_pose_msg, *trans_map_to_init_pose_ptr);

  auto mapTF_init_pose = convertFromROSMsg(mapTF_init_pose_msg);

  if (use_nn_point_z_when_initial_pose_) {
    if (map_manager_.getMapPtr() != nullptr) {
      double min_distance = DBL_MAX;
      double nearest_z = mapTF_init_pose.z;
      for (const auto &p : map_manager_.getMapPtr()->points) {
        double distance = hypot(mapTF_init_pose.x - p.x, mapTF_init_pose.y - p.y);
        if (distance < min_distance) {
          min_distance = distance;
          nearest_z = p.z;
        }
      }
      mapTF_init_pose.z = nearest_z;
    }
  }

  const double current_time_sec = pose_conv_msg_ptr->header.stamp.toSec();

  init_pose_stamped_ = PoseStamped(mapTF_init_pose, current_time_sec);
  pose_interpolator_.clearPoseStamped();
}

void NdtSlam::mappingAndLocalizingPointsCallback(
    const sensor_msgs::PointCloud2::ConstPtr &mapping_points_msg_ptr,
    const sensor_msgs::PointCloud2::ConstPtr &localizing_points_msg_ptr)
{
  current_scan_time_ = localizing_points_msg_ptr->header.stamp;
  sensor_frame_ = localizing_points_msg_ptr->header.frame_id;

  boost::shared_ptr<pcl::PointCloud<PointTarget>> mapping_points_ptr(new pcl::PointCloud<PointTarget>);
  pcl::fromROSMsg(*mapping_points_msg_ptr, *mapping_points_ptr);

  boost::shared_ptr<pcl::PointCloud<PointSource>> localizing_points_ptr(new pcl::PointCloud<PointSource>);
  pcl::fromROSMsg(*localizing_points_msg_ptr, *localizing_points_ptr);

  const auto mapTF_predict_target_pose = getPredictPose();

  mainLoop(mapping_points_ptr, localizing_points_ptr, mapTF_predict_target_pose);
}

void NdtSlam::mappingAndLocalizingPointsAndCurrentPoseCallback(
    const sensor_msgs::PointCloud2::ConstPtr &mapping_points_msg_ptr,
    const sensor_msgs::PointCloud2::ConstPtr &localizing_points_msg_ptr,
    const geometry_msgs::PoseStamped::ConstPtr &current_pose_msg_ptr)
{
  current_scan_time_ = localizing_points_msg_ptr->header.stamp;
  sensor_frame_ = localizing_points_msg_ptr->header.frame_id;

  boost::shared_ptr<pcl::PointCloud<PointTarget>> mapping_points_ptr(new pcl::PointCloud<PointTarget>);
  pcl::fromROSMsg(*mapping_points_msg_ptr, *mapping_points_ptr);

  boost::shared_ptr<pcl::PointCloud<PointSource>> localizing_points_ptr(new pcl::PointCloud<PointSource>);
  pcl::fromROSMsg(*localizing_points_msg_ptr, *localizing_points_ptr);

  geometry_msgs::TransformStamped::Ptr trans_map_to_current_pose_ptr(new geometry_msgs::TransformStamped);
  getTransform(map_frame_, current_pose_msg_ptr->header.frame_id, trans_map_to_current_pose_ptr);
  geometry_msgs::PoseStamped mapTF_current_pose_msg;
  tf2::doTransform(*current_pose_msg_ptr, mapTF_current_pose_msg, *trans_map_to_current_pose_ptr);
  const auto mapTF_predict_target_pose = convertFromROSMsg(mapTF_current_pose_msg);

  mainLoop(mapping_points_ptr, localizing_points_ptr, mapTF_predict_target_pose);
}

void NdtSlam::mainLoop(
    const boost::shared_ptr<pcl::PointCloud<PointTarget>> &mapping_points_ptr,
    const boost::shared_ptr<pcl::PointCloud<PointSource>> &localizing_points_ptr,
    const Pose &mapTF_predict_target_pose)
{
    const auto exe_start_time = std::chrono::system_clock::now();

    updateTransforms();

    const auto mapTF_predict_sensor_pose = transformToPose(mapTF_predict_target_pose, target_to_sensor_matrix_);
    const auto worldTF_pridict_target_pose = transformToPose(world_to_map_matrix_, mapTF_predict_target_pose);

    static bool is_first_call = true;
    if (is_first_call && with_mapping_ && map_manager_.getMapPtr() == nullptr) {
      is_first_call = false;

      boost::shared_ptr<pcl::PointCloud<PointTarget>> mapping_points_limilt_range(new pcl::PointCloud<PointTarget>);
      limitPointCloudRange(mapping_points_ptr, mapping_points_limilt_range, min_scan_range_, max_scan_range_);
      const auto eigen_pose = convertToEigenMatrix4f(mapTF_predict_sensor_pose);
      pcl::transformPointCloud(*mapping_points_limilt_range, *mapping_points_limilt_range, eigen_pose);
      map_manager_.addPointCloudMap(mapping_points_limilt_range);
    }


    if (map_manager_.getMapPtr() == nullptr) {
      ROS_WARN("received points. But map is not loaded");
      return;
    }
    localizer_ptr_->updatePointsMap(map_manager_.getMapPtr());

    const bool align_succeed = localizer_ptr_->alignMap(localizing_points_ptr, mapTF_predict_sensor_pose);
    if (align_succeed == false) {
      return;
    }

    const auto mapTF_sensor_pose = localizer_ptr_->getLocalizerPose();
    if (with_mapping_) {
      mapping(mapping_points_ptr, mapTF_sensor_pose);
    }

    processMatchingScore(localizing_points_ptr);

    const auto worldTF_sensor_pose = transformToPose(world_to_map_matrix_, mapTF_sensor_pose);

    const auto mapTF_target_pose = transformToPose(mapTF_sensor_pose, target_to_sensor_matrix_.inverse());
    const auto worldTF_target_pose = transformToPose(worldTF_sensor_pose, target_to_sensor_matrix_.inverse());

    publish(sensor_pose_pub_, world_frame_, worldTF_sensor_pose);
    publish(ndt_pose_pub_, world_frame_, worldTF_target_pose);
    publish(predict_pose_pub_, world_frame_, worldTF_pridict_target_pose);
    const auto cov = createCovariance();
    publish(ndt_pose_with_covariance_pub_, world_frame_, worldTF_target_pose, cov);

    estimateVelocity(mapTF_target_pose);

    if(publish_tf_) {
        publishTF(map_frame_, target_frame_, mapTF_target_pose);
    }

    const auto exe_end_time = std::chrono::system_clock::now();
    const double exe_time = std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count() / 1000.0;
    publish(time_ndt_matching_pub_, exe_time);

    std::cout << "------------------------------------------------" << std::endl;
    std::cout << "target_pose " << mapTF_target_pose << std::endl;
    std::cout << "velocity: " << pose_interpolator_.getVelocity() << std::endl;
    std::cout << "align_time: " << localizer_ptr_->getAlignTime() << "ms" << std::endl;
    std::cout << "exe_time: " << exe_time << "ms" << std::endl;
    std::cout << "covariance: " << std::endl << localizer_ptr_->getHessian().inverse()*-1.0 << std::endl;
}

void NdtSlam::updateTransforms() {

    auto convertToPose = [](const geometry_msgs::TransformStamped &trans)
    {
        Pose pose;
        pose.x = trans.transform.translation.x;
        pose.y = trans.transform.translation.y;
        pose.z = trans.transform.translation.z;
        tf2::Quaternion q(trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w);
        tf2::Matrix3x3(q).getRPY(pose.roll, pose.pitch, pose.yaw);
        return pose;
    };

    geometry_msgs::TransformStamped::Ptr trans_target_to_sensor_ptr(new geometry_msgs::TransformStamped);
    getTransform(target_frame_, sensor_frame_, trans_target_to_sensor_ptr);
    Pose target_to_sensor_pose = convertToPose(*trans_target_to_sensor_ptr);
    target_to_sensor_matrix_ = convertToEigenMatrix4f(target_to_sensor_pose);

    geometry_msgs::TransformStamped::Ptr trans_world_to_map_ptr(new geometry_msgs::TransformStamped);
    getTransform(world_frame_, map_frame_, trans_world_to_map_ptr);
    Pose world_to_map_pose = convertToPose(*trans_world_to_map_ptr);
    world_to_map_matrix_ = convertToEigenMatrix4f(world_to_map_pose);

}

Pose NdtSlam::getPredictPose() {
    Pose predict_pose;
    if (pose_interpolator_.isNotSetPoseStamped()) {
      predict_pose = init_pose_stamped_.pose;
    }
    else {
      predict_pose = pose_interpolator_.getInterpolatePoseStamped(current_scan_time_.toSec()).pose;
    }
    return predict_pose;
}

void NdtSlam::mapping(const boost::shared_ptr<pcl::PointCloud<PointTarget>> &mapping_points_ptr, const Pose &sensor_pose) {

  static auto added_pose = sensor_pose;
  const double add_scan_shift_meter = std::sqrt(std::pow(sensor_pose.x - added_pose.x, 2.0) +
                                                std::pow(sensor_pose.y - added_pose.y, 2.0) +
                                                std::pow(sensor_pose.z - added_pose.z, 2.0));
  if (add_scan_shift_meter >= min_add_scan_shift_) {
    added_pose = sensor_pose;
    const boost::shared_ptr<pcl::PointCloud<PointTarget>> mapping_points_limilt_range(new pcl::PointCloud<PointTarget>);
    limitPointCloudRange(mapping_points_ptr, mapping_points_limilt_range, min_scan_range_, max_scan_range_);
    const auto eigen_pose = convertToEigenMatrix4f(sensor_pose);
    pcl::transformPointCloud(*mapping_points_limilt_range, *mapping_points_limilt_range, eigen_pose);
    map_manager_.addPointCloudMapThread(mapping_points_limilt_range);
  }

  if (separate_mapping_) {
    const int map_x = std::floor(sensor_pose.x / map_manager_.getSaveSeparateMapSize());
    const int map_y = std::floor(sensor_pose.y / map_manager_.getSaveSeparateMapSize());
    static int prev_map_x = map_x;
    static int prev_map_y = map_x;

    if (map_x != prev_map_x || map_y != prev_map_y) {

      map_manager_.saveSeparateMapThread();
      map_manager_.loadAroundMapThread(sensor_pose);

      prev_map_x = map_x;
      prev_map_y = map_y;
    }
  }

  const int loop_count_donw_map_max = 300;
  static int loop_count_donw_map = 0;
  if (++loop_count_donw_map >= loop_count_donw_map_max) {
    loop_count_donw_map = 0;
    map_manager_.downsampleMapThread();
  }

  if (points_map_pub_.getNumSubscribers() > 0) {
      const int loop_count_max = 30;
      static int loop_count = 0;
      if (++loop_count > loop_count_max) {
          loop_count = 0;
          std::thread map_publish_thread([this]() {
              publish(points_map_pub_, map_frame_, map_manager_.getMapPtr());
          });
          map_publish_thread.detach();
      }
  }

}

void NdtSlam::processMatchingScore(const boost::shared_ptr<pcl::PointCloud<PointTarget>> &points_ptr) {

    if(map_manager_.getMapPtr() == nullptr || map_manager_.getMapPtr()->points.empty()
    || points_ptr == nullptr || points_ptr->points.empty()) {
        return;
    }


    if ( matching_score_pub_.getNumSubscribers() > 0 || matching_score_histogram_pub_.getNumSubscribers() > 0
      || matching_points_pub_.getNumSubscribers() > 0 || unmatching_points_pub_.getNumSubscribers() > 0) {

        updateMatchingScore(points_ptr);
        publish(matching_score_pub_, matching_score_);

        if(matching_score_histogram_pub_.getNumSubscribers() > 0){
            MatchingScoreHistogram matching_score_histogram;
            const auto point_with_distance_array = matching_score_class_.getPointWithDistanceArray();
            const auto histogram_bin_array = matching_score_histogram.createHistogramWithRangeBinArray(point_with_distance_array);
            publish(matching_score_histogram_pub_, map_frame_, histogram_bin_array);
        }

        if(matching_points_pub_.getNumSubscribers() > 0 || unmatching_points_pub_.getNumSubscribers() > 0) {
            const boost::shared_ptr<pcl::PointCloud<PointTarget>> match_points_ptr(new pcl::PointCloud<PointTarget>);
            const boost::shared_ptr<pcl::PointCloud<PointTarget>> unmatch_points_ptr(new pcl::PointCloud<PointTarget>);
            getMatchAndUnmatchPoints(match_points_ptr, unmatch_points_ptr);
            publish(matching_points_pub_, map_frame_, match_points_ptr);
            publish(unmatching_points_pub_, map_frame_, unmatch_points_ptr);
        }
    }
}

void NdtSlam::updateMatchingScore(const boost::shared_ptr<pcl::PointCloud<PointTarget>> &points_ptr) {

    pcl::PointCloud<PointTarget>::Ptr points_baselinkTF_ptr(new pcl::PointCloud<PointTarget>);
    pcl::transformPointCloud(*points_ptr, *points_baselinkTF_ptr, target_to_sensor_matrix_);
    pcl::PointCloud<PointTarget>::Ptr points_baselinkTF_cuttoff_ptr(new pcl::PointCloud<PointTarget>);

    size_t step_size = matching_score_use_points_num_ != 0 ? points_ptr->points.size() / matching_score_use_points_num_ : 1;
    step_size = std::max(step_size, static_cast<size_t>(1));

    size_t points_num = 0;
    for(const auto point : points_baselinkTF_ptr->points) {
        const double range = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
        if ((point.z > matching_score_cutoff_upper_limit_z_ || point.z < matching_score_cutoff_lower_limit_z_)
          &&(range > matching_score_cutoff_lower_limit_range_ && range < matching_score_cutoff_upper_limit_range_)) {

            //random downsample
            if(step_size == 0 || ++points_num % step_size == 0) {
                points_baselinkTF_cuttoff_ptr->push_back(point);
            }
        }
    }
    pcl::PointCloud<PointTarget>::Ptr points_mapTF_cuttoff_ptr(new pcl::PointCloud<PointTarget>);
    const auto mapTF_sensor_pose = localizer_ptr_->getLocalizerPose();
    const auto mapTF_target_pose = transformToPose(mapTF_sensor_pose, target_to_sensor_matrix_.inverse());
    const auto map_to_target_matrix = convertToEigenMatrix4f(mapTF_target_pose);
    pcl::transformPointCloud(*points_baselinkTF_cuttoff_ptr, *points_mapTF_cuttoff_ptr, map_to_target_matrix);

    matching_score_class_.setSearchMethodTarget(localizer_ptr_->getSearchMethodTarget());
    matching_score_ = matching_score_class_.calcMatchingScore(points_mapTF_cuttoff_ptr);
}


void NdtSlam::getMatchAndUnmatchPoints(const boost::shared_ptr<pcl::PointCloud<PointTarget>> &match_points_ptr,
                                       const boost::shared_ptr<pcl::PointCloud<PointTarget>> &unmatch_points_ptr) {

    const auto point_with_distance_array = matching_score_class_.getPointWithDistanceArray();
    for(const auto& point_with_distance : point_with_distance_array) {
        //more than score 50%
        if(point_with_distance.distance < matching_score_class_.getFermiMu()) {
            match_points_ptr->points.push_back(point_with_distance.point);
        }
        else {
            unmatch_points_ptr->points.push_back(point_with_distance.point);
        }
    }
}

std::array<double, 36> NdtSlam::createCovariance() {

    std::array<double, 36> cov_array = {};

    for(size_t i = 0; i < 6; ++i) {
        double coffe = -1.0;
        for(size_t j = 0; j < 6; ++j) {
            double v = localizer_ptr_->getHessian().inverse()(i,j);
            v = (!std::isnan(v)&&!std::isinf(v)) ? v : 100; //TODO
            cov_array[6*i + j] = v*coffe;
        }
    }

    return cov_array;
}

void NdtSlam::estimateVelocity(const Pose &pose) {
    PoseStamped pose_stamped(pose, current_scan_time_.toSec());
    if (pose_interpolator_.isNotSetPoseStamped()) {
      pose_interpolator_.pushbackPoseStamped(pose_stamped);
      pose_interpolator_.pushbackPoseStamped(pose_stamped);
    }
    else {
      pose_interpolator_.pushbackPoseStamped(pose_stamped);
    }
    const auto velocity = pose_interpolator_.getVelocity();
    const auto trans_current_pose = convertPoseIntoRelativeCoordinate(pose_interpolator_.getCurrentPoseStamped().pose,
                                                                      pose_interpolator_.getPrevPoseStamped().pose);
    const bool is_move_forward = trans_current_pose.x >= 0;
    const auto targetTF_velocity = transformBaseLinkTFVelocity(velocity, is_move_forward);
    publish(estimate_twist_pub_, target_frame_, targetTF_velocity);
}

Velocity NdtSlam::transformBaseLinkTFVelocity(const Velocity& velocity, const bool is_move_forward) {
    Velocity targetTF_velocity;
    double v = std::sqrt(std::pow(velocity.linear.x, 2.0) +
                         std::pow(velocity.linear.y, 2.0) +
                         std::pow(velocity.linear.z, 2.0));
    targetTF_velocity.linear.x = is_move_forward ? v : -v;
    targetTF_velocity.angular.z = velocity.angular.z;
    return targetTF_velocity;
}


void NdtSlam::publish(const ros::Publisher &publisher, const double value) {
    std_msgs::Float32 msg;
    msg.data = value;
    publisher.publish(msg);
}

void NdtSlam::publish(const ros::Publisher &publisher, const std::string frame_id, const Pose &pose) {
    std_msgs::Header header;
    header.frame_id = frame_id;
    header.stamp = current_scan_time_;
    const auto msg = convertToROSMsg(header, pose);
    publisher.publish(msg);
}

void NdtSlam::publish(const ros::Publisher &publisher, const std::string frame_id, const Pose &pose, const std::array<double, 36> cov) {
    std_msgs::Header header;
    header.frame_id = frame_id;
    header.stamp = current_scan_time_;
    const auto msg = convertToROSMsg(header, pose, cov);
    publisher.publish(msg);
}

void NdtSlam::publish(const ros::Publisher &publisher, const std::string frame_id, const Velocity &velocity) {
    std_msgs::Header header;
    header.frame_id = frame_id;
    header.stamp = current_scan_time_;
    const auto msg = convertToROSMsg(header, velocity);
    publisher.publish(msg);
}

void NdtSlam::publish(const ros::Publisher &publisher, const std::string frame_id, const std::vector<HistogramWithRangeBin> &histogram_bin_array) {
    std_msgs::Header header;
    header.frame_id = frame_id;
    header.stamp = current_scan_time_;
    const auto msg = convertToROSMsg(header, histogram_bin_array);
    publisher.publish(msg);
}

template<class PointType>
void NdtSlam::publish(const ros::Publisher &publisher, const std::string frame_id, const boost::shared_ptr<pcl::PointCloud<PointType>> &points_ptr) {

    if (points_ptr == nullptr || points_ptr->points.size() == 0) {
      return;
    }

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*points_ptr, msg);
    msg.header.frame_id = frame_id;
    msg.header.stamp = current_scan_time_;
    publisher.publish(msg);
}

void NdtSlam::publishTF(const std::string &frame_id, const std::string &child_frame_id, const Pose &pose) {

  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.transform.translation.x = pose.x;
  transform_stamped.transform.translation.y = pose.y;
  transform_stamped.transform.translation.z = pose.z;
  tf2::Quaternion tf_quaternion;
  tf_quaternion.setRPY(pose.roll, pose.pitch, pose.yaw);
  transform_stamped.transform.rotation.x = tf_quaternion.x();
  transform_stamped.transform.rotation.y = tf_quaternion.y();
  transform_stamped.transform.rotation.z = tf_quaternion.z();
  transform_stamped.transform.rotation.w = tf_quaternion.w();
  tf2_broadcaster_.sendTransform(transform_stamped);
}

bool NdtSlam::getTransform(const std::string &target_frame, const std::string &source_frame, const geometry_msgs::TransformStamped::Ptr &transform_stamped_ptr)
{
    if(target_frame == source_frame) {
        transform_stamped_ptr->header.stamp = ros::Time::now();
        transform_stamped_ptr->header.frame_id = target_frame;
        transform_stamped_ptr->child_frame_id = source_frame;
        transform_stamped_ptr->transform.translation.x = 0.0;
        transform_stamped_ptr->transform.translation.y = 0.0;
        transform_stamped_ptr->transform.translation.z = 0.0;
        transform_stamped_ptr->transform.rotation.x = 0.0;
        transform_stamped_ptr->transform.rotation.y = 0.0;
        transform_stamped_ptr->transform.rotation.z = 0.0;
        transform_stamped_ptr->transform.rotation.w = 1.0;
        return true;
    }

    try {
      *transform_stamped_ptr = tf2_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      ROS_ERROR("Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

      transform_stamped_ptr->header.stamp = ros::Time::now();
      transform_stamped_ptr->header.frame_id = target_frame;
      transform_stamped_ptr->child_frame_id = source_frame;
      transform_stamped_ptr->transform.translation.x = 0.0;
      transform_stamped_ptr->transform.translation.y = 0.0;
      transform_stamped_ptr->transform.translation.z = 0.0;
      transform_stamped_ptr->transform.rotation.x = 0.0;
      transform_stamped_ptr->transform.rotation.y = 0.0;
      transform_stamped_ptr->transform.rotation.z = 0.0;
      transform_stamped_ptr->transform.rotation.w = 1.0;
      return false;
    }
    return true;
}
