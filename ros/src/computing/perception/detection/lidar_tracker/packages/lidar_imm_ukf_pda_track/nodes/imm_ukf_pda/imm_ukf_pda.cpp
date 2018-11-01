/*
 *  Copyright (c) 2018, Nagoya University
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

#include "imm_ukf_pda.h"

ImmUkfPda::ImmUkfPda()
  : target_id_(0)
  ,  // assign unique ukf_id_ to each tracking targets
  init_(false)
{
  ros::NodeHandle private_nh_("~");

  private_nh_.param<std::string>("input_topic", input_topic_, "/detection/lidar_detector/objects");
  ROS_INFO("[%s] input_topic: %s", __APP_NAME__, input_topic_.c_str());
  private_nh_.param<std::string>("tracking_frame", tracking_frame_, "world");
  ROS_INFO("[%s] tracking_frame: %s", __APP_NAME__, tracking_frame_.c_str());
  private_nh_.param<int>("life_time_thres", life_time_thres_, 8);
  ROS_INFO("[%s] life_time_thres: %d", __APP_NAME__, life_time_thres_);
  private_nh_.param<double>("gating_thres", gating_thres_, 9.22);
  ROS_INFO("[%s] gating_thres: %f", __APP_NAME__, gating_thres_);
  private_nh_.param<double>("gate_probability", gate_probability_, 0.99);
  ROS_INFO("[%s] gate_probability: %f", __APP_NAME__, gate_probability_);
  private_nh_.param<double>("detection_probability", detection_probability_, 0.9);
  ROS_INFO("[%s] detection_probability: %f", __APP_NAME__, detection_probability_);
  private_nh_.param<double>("distance_thres", distance_thres_, 99);
  ROS_INFO("[%s] distance_thres: %f", __APP_NAME__, distance_thres_);
  private_nh_.param<double>("static_velocity_thres", static_velocity_thres_, 0.5);
  ROS_INFO("[%s] static_velocity_thres: %f", __APP_NAME__, static_velocity_thres_);
  private_nh_.param<double>("prevent_explosion_thres", prevent_explosion_thres_, 1000);
  ROS_INFO("[%s] prevent_explosion_thres: %f", __APP_NAME__, prevent_explosion_thres_);
  private_nh_.param<bool>("use_sukf", use_sukf_, false);
  ROS_INFO("[%s] use_sukf: %d", __APP_NAME__, use_sukf_);
}

void ImmUkfPda::Run()
{
  std::string output_topic = "/detection/lidar_tracker/objects";

  pub_object_array_ = node_handle_.advertise<autoware_msgs::DetectedObjectArray>(output_topic, 1);
  ROS_INFO("[%s] output_topic: %s", __APP_NAME__, output_topic.c_str());

  sub_detected_array_ = node_handle_.subscribe(input_topic_, 1, &ImmUkfPda::DetectionsCallback, this);
  ROS_INFO("[%s] source_topic: %s", __APP_NAME__, input_topic_.c_str());
}

void ImmUkfPda::DetectionsCallback(const autoware_msgs::DetectedObjectArray& in_objects)
{
  autoware_msgs::DetectedObjectArray transformed_input, transformed_output;
  autoware_msgs::DetectedObjectArray tracked_objects;

  transformed_input = transformPoseToGlobal(in_objects, tracking_frame_);

  tracked_objects = tracker(transformed_input);

  transformed_output = transformPoseToLocal(tracked_objects, in_objects.header.frame_id);

  pub_object_array_.publish(transformed_output);
}

autoware_msgs::DetectedObjectArray ImmUkfPda::transformPoseToGlobal(
    const autoware_msgs::DetectedObjectArray& in_objects, const std::string& in_target_frame)
{
  try
  {
    tf_listener_.waitForTransform(in_objects.header.frame_id, in_target_frame, ros::Time(0), ros::Duration(1.0));
    // get sensor -> world frame
    tf_listener_.lookupTransform(in_target_frame, in_objects.header.frame_id, ros::Time(0), local2global_);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  autoware_msgs::DetectedObjectArray transformed_objects;
  transformed_objects.header = in_objects.header;
  transformed_objects.header.frame_id = in_target_frame;

  for (size_t i = 0; i < in_objects.objects.size(); i++)
  {
    geometry_msgs::PoseStamped pose_in, pose_out;

    pose_in.header = in_objects.header;
    pose_in.pose = in_objects.objects[i].pose;
    tf::Transform input_object_pose;
    input_object_pose.setOrigin(tf::Vector3(in_objects.objects[i].pose.position.x,
                                            in_objects.objects[i].pose.position.y,
                                            in_objects.objects[i].pose.position.z));
    input_object_pose.setRotation(
        tf::Quaternion(in_objects.objects[i].pose.orientation.x, in_objects.objects[i].pose.orientation.y,
                       in_objects.objects[i].pose.orientation.z, in_objects.objects[i].pose.orientation.w));
    tf::poseTFToMsg(local2global_ * input_object_pose, pose_out.pose);

    autoware_msgs::DetectedObject dd;
    dd = in_objects.objects[i];
    dd.header = in_objects.header;
    dd.header.frame_id = in_target_frame;
    dd.pose = pose_out.pose;

    transformed_objects.objects.push_back(dd);
  }
  return transformed_objects;
}

autoware_msgs::DetectedObjectArray ImmUkfPda::transformPoseToLocal(const autoware_msgs::DetectedObjectArray& in_objects,
                                                                   const std::string& in_target_frame)
{
  autoware_msgs::DetectedObjectArray transformed_objects;
  transformed_objects.header = in_objects.header;
  transformed_objects.header.frame_id = in_target_frame;

  for (size_t i = 0; i < in_objects.objects.size(); i++)
  {
    geometry_msgs::PoseStamped detected_pose_out;

    tf::Transform output_object_pose;
    output_object_pose.setOrigin(tf::Vector3(in_objects.objects[i].pose.position.x,
                                             in_objects.objects[i].pose.position.y,
                                             in_objects.objects[i].pose.position.z));
    output_object_pose.setRotation(
        tf::Quaternion(in_objects.objects[i].pose.orientation.x, in_objects.objects[i].pose.orientation.y,
                       in_objects.objects[i].pose.orientation.z, in_objects.objects[i].pose.orientation.w));
    tf::poseTFToMsg(local2global_.inverse() * output_object_pose, detected_pose_out.pose);

    autoware_msgs::DetectedObject dd;
    dd = in_objects.objects[i];
    dd.header = in_objects.header;
    dd.header.frame_id = in_target_frame;
    dd.pose = detected_pose_out.pose;

    transformed_objects.objects.push_back(dd);
  }
  return transformed_objects;
}  // transformPoseToLocal

void ImmUkfPda::measurementValidation(const autoware_msgs::DetectedObjectArray& input, UKF& target,
                                      const bool second_init, const Eigen::VectorXd& max_det_z,
                                      const Eigen::MatrixXd& max_det_s,
                                      std::vector<autoware_msgs::DetectedObject>& object_vec,
                                      std::vector<bool>& matching_vec)
{
  // alert: different from original imm-pda filter, here picking up most likely measurement
  // if making it allows to have more than one measurement, you will see non semipositive definite covariance
  bool second_init_done = false;
  double smallest_nis = std::numeric_limits<double>::max();
  autoware_msgs::DetectedObject smallest_meas_object;
  for (size_t i = 0; i < input.objects.size(); i++)
  {
    double x = input.objects[i].pose.position.x;
    double y = input.objects[i].pose.position.y;

    Eigen::VectorXd meas = Eigen::VectorXd(2);
    meas << x, y;

    Eigen::VectorXd diff = meas - max_det_z;
    double nis = diff.transpose() * max_det_s.inverse() * diff;

    if (nis < gating_thres_)
    {  // x^2 99% range
      if (matching_vec[i] == false)
      {
        target.lifetime_++;
      }

      if (nis < smallest_nis)
      {
        smallest_nis = nis;
        smallest_meas_object = input.objects[i];
        matching_vec[i] = true;
        second_init_done = true;
      }
    }
  }
  if (second_init_done)
  {
    object_vec.push_back(smallest_meas_object);
  }
}  // measurementValidation

void ImmUkfPda::getNearestEuclidCluster(const UKF& target, const std::vector<autoware_msgs::DetectedObject>& object_vec,
                                        autoware_msgs::DetectedObject& object, double& min_dist)
{
  int min_ind = 0;
  double px = target.x_merge_(0);
  double py = target.x_merge_(1);

  for (size_t i = 0; i < object_vec.size(); i++)
  {
    double meas_x = object_vec[i].pose.position.x;
    double meas_y = object_vec[i].pose.position.y;

    double dist = sqrt((px - meas_x) * (px - meas_x) + (py - meas_y) * (py - meas_y));
    if (dist < min_dist)
    {
      min_dist = dist;
      min_ind = i;
    }
  }

  object = object_vec[min_ind];
}  // getNearestEuclidCluster

void ImmUkfPda::associateBB(const std::vector<autoware_msgs::DetectedObject>& object_vec, UKF& target)
{
  // skip if no validated measurement
  if (object_vec.empty())
  {
    return;
  }
  if (target.tracking_num_ == TrackingState::Stable && target.lifetime_ >= life_time_thres_)
  {
    autoware_msgs::DetectedObject nearest_object;
    double min_dist = std::numeric_limits<double>::max();
    getNearestEuclidCluster(target, object_vec, nearest_object, min_dist);
    if (min_dist < distance_thres_)
    {
      target.is_reliable_ = true;                             // this should be changed for setters
      target.object_pose_ = nearest_object.pose;              // this should be changed for setters
      target.object_dimensions_ = nearest_object.dimensions;  // this should be changed for setters
    }
  }
  else
  {
    autoware_msgs::DetectedObject nearest_object;
    double min_dist = std::numeric_limits<double>::max();
    getNearestEuclidCluster(target, object_vec, nearest_object, min_dist);
    target.object_pose_ = nearest_object.pose;              // this should be changed for setters
    target.object_dimensions_ = nearest_object.dimensions;  // this should be changed for setters
  }
}  // associateBB

void ImmUkfPda::updateBehaviorState(const UKF& target, autoware_msgs::DetectedObject& object)
{
  if (target.mode_prob_cv_ > target.mode_prob_ctrv_ && target.mode_prob_cv_ > target.mode_prob_rm_)
  {
    object.behavior_state = MotionModel::CV;
  }
  else if (target.mode_prob_ctrv_ > target.mode_prob_cv_ && target.mode_prob_ctrv_ > target.mode_prob_rm_)
  {
    object.behavior_state = MotionModel::CTRV;
  }
  else
  {
    object.behavior_state = MotionModel::RM;
  }
}  // updateBehaviorState

void ImmUkfPda::initTracker(const autoware_msgs::DetectedObjectArray& in_objects, double timestamp)
{
  for (size_t i = 0; i < in_objects.objects.size(); i++)
  {
    double px = in_objects.objects[i].pose.position.x;
    double py = in_objects.objects[i].pose.position.y;
    Eigen::VectorXd init_meas = Eigen::VectorXd(2);
    init_meas << px, py;

    UKF ukf;
    ukf.initialize(init_meas, timestamp, target_id_, in_objects.objects[i].label);
    targets_.push_back(ukf);
    target_id_++;
  }
  timestamp_ = timestamp;
  init_ = true;
}  // initTracker

void ImmUkfPda::secondInit(UKF& target, const std::vector<autoware_msgs::DetectedObject>& object_vec, double dt)
{
  if (object_vec.size() == 0)
  {
    target.tracking_num_ = TrackingState::Die;
    return;
  }
  // record init measurement for env classification
  target.init_meas_ << target.x_merge_(0), target.x_merge_(1);

  // state update
  double target_x = object_vec[0].pose.position.x;
  double target_y = object_vec[0].pose.position.y;
  double target_diff_x = target_x - target.x_merge_(0);
  double target_diff_y = target_y - target.x_merge_(1);
  double target_yaw = atan2(target_diff_y, target_diff_x);
  double dist = sqrt(target_diff_x * target_diff_x + target_diff_y * target_diff_y);
  double target_v = dist / dt;

  while (target_yaw > M_PI)
    target_yaw -= 2. * M_PI;
  while (target_yaw < -M_PI)
    target_yaw += 2. * M_PI;

  target.x_merge_(0) = target.x_cv_(0) = target.x_ctrv_(0) = target.x_rm_(0) = target_x;
  target.x_merge_(1) = target.x_cv_(1) = target.x_ctrv_(1) = target.x_rm_(1) = target_y;
  target.x_merge_(2) = target.x_cv_(2) = target.x_ctrv_(2) = target.x_rm_(2) = target_v;
  target.x_merge_(3) = target.x_cv_(3) = target.x_ctrv_(3) = target.x_rm_(3) = target_yaw;

  // target.initCovarQs(dt, target_yaw);

  target.tracking_num_++;
}  // secondInit

void ImmUkfPda::updateTrackingNum(const std::vector<autoware_msgs::DetectedObject>& object_vec, UKF& target)
{
  if (object_vec.size() > 0)
  {
    if (target.tracking_num_ < TrackingState::Stable)
    {
      target.tracking_num_++;
    }
    else if (target.tracking_num_ == TrackingState::Stable)
    {
      target.tracking_num_ = TrackingState::Stable;
    }
    else if (target.tracking_num_ >= TrackingState::Stable && target.tracking_num_ < TrackingState::Lost)
    {
      target.tracking_num_ = TrackingState::Stable;
    }
    else if (target.tracking_num_ == TrackingState::Lost)
    {
      target.tracking_num_ = TrackingState::Die;
    }
  }
  else
  {
    if (target.tracking_num_ < TrackingState::Stable)
    {
      target.tracking_num_ = TrackingState::Die;
    }
    else if (target.tracking_num_ >= TrackingState::Stable && target.tracking_num_ < TrackingState::Lost)
    {
      target.tracking_num_++;
    }
    else if (target.tracking_num_ == TrackingState::Lost)
    {
      target.tracking_num_ = TrackingState::Die;
    }
  }
}  // updateTrackingNum

void ImmUkfPda::probabilisticDataAssociation(const autoware_msgs::DetectedObjectArray& in_objects, const double dt,
                                             std::vector<bool>& matching_vec,
                                             std::vector<autoware_msgs::DetectedObject>& object_vec, UKF& target,
                                             bool& is_skip_target)
{
  double det_s = 0;
  Eigen::VectorXd max_det_z;
  Eigen::MatrixXd max_det_s;
  is_skip_target = false;

  if (use_sukf_)
  {
    max_det_z = target.z_pred_ctrv_;
    max_det_s = target.s_ctrv_;
    det_s = max_det_s.determinant();
  }
  else
  {
    // find maxDetS associated with predZ
    target.findMaxZandS(max_det_z, max_det_s);
    det_s = max_det_s.determinant();
  }

  // prevent ukf not to explode
  if (std::isnan(det_s) || det_s > prevent_explosion_thres_)
  {
    target.tracking_num_ = TrackingState::Die;
    is_skip_target = true;
    return;
  }

  bool is_second_init;
  if (target.tracking_num_ == TrackingState::Init)
  {
    is_second_init = true;
  }
  else
  {
    is_second_init = false;
  }

  // measurement gating, get measVec, bboxVec, matchingVec through reference
  measurementValidation(in_objects, target, is_second_init, max_det_z, max_det_s, object_vec, matching_vec);

  // bounding box association if target is stable :plus, right angle correction if its needed
  // input: track number, bbox measurements, &target
  associateBB(object_vec, target);

  // second detection for a target: update v and yaw
  if (is_second_init)
  {
    secondInit(target, object_vec, dt);
    is_skip_target = true;
    return;
  }

  // update tracking number
  updateTrackingNum(object_vec, target);

  if (target.tracking_num_ == TrackingState::Die)
  {
    is_skip_target = true;
    return;
  }
}  // probabilisticDataAssociation

void ImmUkfPda::makeNewTargets(const double timestamp, const autoware_msgs::DetectedObjectArray& in_objects,
                               const std::vector<bool>& matching_vec)
{
  for (size_t i = 0; i < in_objects.objects.size(); i++)
  {
    if (matching_vec[i] == false)
    {
      double px = in_objects.objects[i].pose.position.x;
      double py = in_objects.objects[i].pose.position.y;
      Eigen::VectorXd init_meas = Eigen::VectorXd(2);
      init_meas << px, py;

      UKF ukf;
      ukf.initialize(init_meas, timestamp, target_id_, in_objects.objects[i].label);
      targets_.push_back(ukf);
      target_id_++;
    }
  }
}  // makeNewTargets

void ImmUkfPda::staticClassification()
{
  for (size_t i = 0; i < targets_.size(); i++)
  {
    targets_[i].vel_history_.push_back(targets_[i].x_merge_(2));
    if (targets_[i].tracking_num_ == TrackingState::Stable && targets_[i].lifetime_ > life_time_thres_)
    {
      double sum_vel = 0;
      double avg_vel = 0;
      for (int ind = 1; ind < life_time_thres_; ind++)
      {
        sum_vel += targets_[i].vel_history_.end()[-ind];
      }
      avg_vel = double(sum_vel / life_time_thres_);

      if ((avg_vel < static_velocity_thres_) && (targets_[i].mode_prob_rm_ > targets_[i].mode_prob_cv_ ||
                                                 targets_[i].mode_prob_rm_ > targets_[i].mode_prob_ctrv_))
      {
        targets_[i].is_static_ = true;
      }
    }
  }
}  // staticClassification

autoware_msgs::DetectedObjectArray ImmUkfPda::makeOutput(const autoware_msgs::DetectedObjectArray& in_objects)
{
  autoware_msgs::DetectedObjectArray output_objects;

  output_objects.header = in_objects.header;
  for (size_t i = 0; i < targets_.size(); i++)
  {
    double tx = targets_[i].x_merge_(0);
    double ty = targets_[i].x_merge_(1);

    double tv = targets_[i].x_merge_(2);
    double tyaw = targets_[i].x_merge_(3);
    while (tyaw > M_PI)
      tyaw -= 2. * M_PI;
    while (tyaw < -M_PI)
      tyaw += 2. * M_PI;

    tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, tyaw);

    autoware_msgs::DetectedObject dd;
    dd.header = in_objects.header;
    dd.id = targets_[i].ukf_id_;
    dd.pose = targets_[i].object_pose_;
    dd.dimensions = targets_[i].object_dimensions_;
    dd.pose_reliable = targets_[i].is_reliable_;
    dd.label = targets_[i].object_label_;
    dd.velocity.linear.x = tv;
    // store yaw rate for motion into dd.accerelation.linear.y
    dd.acceleration.linear.y = targets_[i].x_merge_(4);
    if (targets_[i].is_reliable_)
    {
      std::string s_velocity = std::to_string(tv * 3.6);
      std::string modified_sv = s_velocity.substr(0, s_velocity.find(".") + 3);

      std::string text =
          "<" + std::to_string(targets_[i].ukf_id_) + ">" + " " + std::to_string(targets_[i].x_merge_(2)) + " m/s "
          //+ "(" + std::to_string(targets_[i].x_merge_(0)) + ", "
          // + std::to_string(targets_[i].x_merge_(1)) + ")"
          ;
      dd.label += text;
      output_objects.objects.push_back(dd);
    }


    dd.pose.position.x = tx;
    dd.pose.position.y = ty;
    dd.pose.orientation.x = q[0];
    dd.pose.orientation.y = q[1];
    dd.pose.orientation.z = q[2];
    dd.pose.orientation.w = q[3];

    updateBehaviorState(targets_[i], dd);
  }

  return output_objects;
}  // makeOutput

void ImmUkfPda::removeUnnecessaryTarget()
{
  std::vector<UKF> temp_targets;
  for (size_t i = 0; i < targets_.size(); i++)
  {
    if (targets_[i].tracking_num_ != TrackingState::Die)
    {
      temp_targets.push_back(targets_[i]);
    }
  }
  std::vector<UKF>().swap(targets_);
  targets_ = temp_targets;
}  // removeUnnecessaryTarget

autoware_msgs::DetectedObjectArray ImmUkfPda::tracker(const autoware_msgs::DetectedObjectArray& in_objects)
{
  double timestamp = in_objects.header.stamp.toSec();

  autoware_msgs::DetectedObjectArray detected_objects_output;

  if (!init_)
  {
    initTracker(in_objects, timestamp);
    detected_objects_output = makeOutput(in_objects);
    return detected_objects_output;
  }

  double dt = (timestamp - timestamp_);
  timestamp_ = timestamp;
  // making new target with no data association
  std::vector<bool> matching_vec(in_objects.objects.size(), false);

  // start UKF process
  for (size_t i = 0; i < targets_.size(); i++)
  {
    // reset is_vis_bb_ to false
    targets_[i].is_reliable_ = false;
    targets_[i].is_static_ = false;

    if (targets_[i].tracking_num_ == TrackingState::Die)
    {
      continue;
    }
    // prevent ukf not to explode
    if (targets_[i].p_merge_.determinant() > prevent_explosion_thres_ ||
        targets_[i].p_merge_(4, 4) > prevent_explosion_thres_)
    {
      targets_[i].tracking_num_ = TrackingState::Die;
      continue;
    }

    if (use_sukf_)
    {
      // standard ukf prediction step
      targets_[i].predictionSUKF(dt);
      // data association
      bool is_skip_target;
      std::vector<autoware_msgs::DetectedObject> object_vec;
      probabilisticDataAssociation(in_objects, dt, matching_vec, object_vec, targets_[i], is_skip_target);
      if (is_skip_target)
      {
        continue;
      }
      // standard ukf update step
      targets_[i].updateSUKF(object_vec);
    }
    else  // immukfpda filter
    {
      // immukf prediction step
      targets_[i].predictionIMMUKF(dt);
      // data association
      bool is_skip_target;
      std::vector<autoware_msgs::DetectedObject> object_vec;
      probabilisticDataAssociation(in_objects, dt, matching_vec, object_vec, targets_[i], is_skip_target);
      if (is_skip_target)
      {
        continue;
      }
      // immukf update step
      targets_[i].updateIMMUKF(detection_probability_, gate_probability_, gating_thres_, object_vec);
    }
  }
  // end UKF process

  // making new ukf target for no data association clusters
  makeNewTargets(timestamp, in_objects, matching_vec);

  // static dynamic classification
  staticClassification();

  // making output for visualization
  detected_objects_output = makeOutput(in_objects);

  // remove unnecessary ukf object
  removeUnnecessaryTarget();

  return detected_objects_output;
}  // tracker
