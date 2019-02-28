/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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
  ********************
    Abraham Monrroy
    based on Alexander Carballo's idea for Tsukuba Challenge for NDT monitoring.

    Created on: March 20, 2018
 */

#include "ndt_matching_monitor.h"

void ROSNDTMatchingMonitor::gnss_callback(const geometry_msgs::PoseStamped::ConstPtr& input)
{
    gnss_pose_.header = input->header;
    gnss_pose_.pose.pose = input->pose;
    gnss_pose_available_ = true;
    gnss_text_ = " - GNSS available";
}

void ROSNDTMatchingMonitor::ndt_stat_callback(const autoware_msgs::NDTStat::ConstPtr& input)
{
    iteration_count_ = input->iteration;

    if (last_score_ == 0.f)
        last_score_ = input->score;

    current_score_ = input->score;
    score_delta_ = fabs(current_score_-last_score_);
}

geometry_msgs::PoseWithCovarianceStamped
                ROSNDTMatchingMonitor::predict_next_pose(geometry_msgs::PoseWithCovarianceStamped prev_pose,
                                                         geometry_msgs::PoseWithCovarianceStamped current_pose)
{
    geometry_msgs::PoseWithCovarianceStamped predicted_pose;

    double time_delta = current_pose.header.stamp.toSec() - prev_pose.header.stamp.toSec();

    if (time_delta == 0)
    {
        return current_pose;
    }

    predicted_pose.header = current_pose.header;
    predicted_pose.header.stamp.sec += time_delta;
    predicted_pose.pose.pose.orientation = current_pose.pose.pose.orientation;

    predicted_pose.pose.pose.position.x = current_pose.pose.pose.position.x +
            (current_pose.pose.pose.position.x - current_pose.pose.pose.position.x) / time_delta;
    predicted_pose.pose.pose.position.y = current_pose.pose.pose.position.y +
            (current_pose.pose.pose.position.y - current_pose.pose.pose.position.y) / time_delta;
    predicted_pose.pose.pose.position.z = current_pose.pose.pose.position.z +
            (current_pose.pose.pose.position.z - current_pose.pose.pose.position.z) / time_delta;

    return predicted_pose;

}

void ROSNDTMatchingMonitor::initialpose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& input)
{
    //if currently blocking reset and receiving a different blocking pose, then try to reset
    if (ndt_status::NDT_FATAL == ndt_status_
        && input->pose.pose.position.x != initialpose_.pose.pose.position.x
        && input->pose.pose.position.y != initialpose_.pose.pose.position.y
        && input->pose.pose.position.z != initialpose_.pose.pose.position.z
        && input->pose.pose.orientation.x != initialpose_.pose.pose.orientation.x
        && input->pose.pose.orientation.y != initialpose_.pose.pose.orientation.y
        && input->pose.pose.orientation.z != initialpose_.pose.pose.orientation.z
        && input->pose.pose.orientation.w != initialpose_.pose.pose.orientation.w)
    {
        ndt_status_ = ndt_status::NDT_NOT_INITIALIZED;
        initialpose_ = *input;
        initialized_ = false;
        last_score_ = 0.;
        ROS_INFO_STREAM(__APP_NAME__ << " RECEIVED NEW INITIAL POSE, RESETTING.");
    }
}

void ROSNDTMatchingMonitor::ndt_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& input)
{
    geometry_msgs::PoseWithCovarianceStamped initialpose_msg;
    initialpose_msg.header = input->header;
    jsk_rviz_plugins::OverlayText rviz_info_text;
    std_msgs::String ndt_status_msg;

    if (ndt_status::NDT_FATAL == ndt_status_)
    {
        rviz_info_text = ndt_fatal_text_;
        ROS_ERROR_STREAM(__APP_NAME__ << " FATAL CANNOT RECOVER - STOPPING.");
        initialpose_msg.pose = initialpose_.pose;
        initialpose_pub_.publish(initialpose_msg);
        overlay_info_text_pub_.publish(rviz_info_text);
        stable_samples_ = 0;
        return;
    }

    if (iteration_count_ < iteration_threshold_warning_
        && score_delta_ < score_delta_threshold_)
    {
        //update the last good pose
        initialpose_.pose.pose = input->pose;
        prev_initialpose_ = initialpose_;
        if (stable_samples_ >= (unsigned int) min_stable_samples_)
        {
            ndt_status_ = ndt_status::NDT_OK;
            initialized_ = true;
        }
        stable_samples_++;
    }
    else if ( initialized_ &&
            ((iteration_count_ >= iteration_threshold_stop_)
             || (iteration_count_ >= iteration_threshold_warning_ && score_delta_ >= score_delta_threshold_)))
    {
        ndt_status_ = ndt_status::NDT_ERROR;
        if (gnss_pose_available_)
        {
            initialpose_msg.header.frame_id = gnss_pose_.header.frame_id;
            initialpose_msg.pose = predict_next_pose(prev_gnss_pose_, gnss_pose_).pose;
        }
        else
        {
            initialpose_msg.pose = predict_next_pose(prev_initialpose_, initialpose_).pose;
            //if there are too many predictions in a short time, set fatal status
            if (prediction_samples_ > fatal_time_threshold_
                && (input->header.stamp.toSec() - last_prediction_time_.toSec()) <= fatal_time_threshold_ )
            {
                ndt_status_ = ndt_status::NDT_FATAL;
            }
            last_prediction_time_ = input->header.stamp;
            prediction_samples_++;
        }

        if (gnss_pose_available_
            || stable_samples_ >= (unsigned int) min_stable_samples_ )
        {
            ROS_INFO_STREAM(__APP_NAME__ << " Resetting position.");
            initialpose_pub_.publish(initialpose_msg);
            stable_samples_ = 0;
        }
    }
    else if (initialized_ &&
            ((iteration_count_ > iteration_threshold_warning_
              && iteration_count_< iteration_threshold_stop_)
              || ndt_status_ == ndt_status::NDT_ERROR))
    {
        ndt_status_ = ndt_status::NDT_WARNING;
        prev_initialpose_ = initialpose_;
    }

    prev_gnss_pose_ = gnss_pose_;

    if (!initialized_)
    {
        rviz_info_text = ndt_not_ready_text_;
    } else
    {
        switch (ndt_status_)
        {
            case ndt_status::NDT_ERROR:
                rviz_info_text = ndt_error_text_;
                break;
            case ndt_status::NDT_WARNING:
                rviz_info_text = ndt_warn_text_;
                break;
            case ndt_status::NDT_OK:
                rviz_info_text = ndt_normal_text_;
                break;
            case ndt_status::NDT_NOT_INITIALIZED:
            default:
                rviz_info_text = ndt_not_ready_text_;
                break;
        }
    }

    rviz_info_text.text+=gnss_text_;

    ndt_status_msg.data = ndt_status_names[ndt_status_];

    overlay_info_text_pub_.publish(rviz_info_text);
    ndt_status_pub_.publish(ndt_status_msg);
    last_score_ = current_score_;
}

void ROSNDTMatchingMonitor::Run()
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Geting parameters
    private_nh.param("/monitor/iteration_threshold_warn", iteration_threshold_warning_, NDT_THRESHOLD_ITERATION_WARN);
    private_nh.param("/monitor/iteration_threshold_stop", iteration_threshold_stop_, NDT_THRESHOLD_ITERATION_STOP);
    private_nh.param("/monitor/score_delta_threshold", score_delta_threshold_, NDT_THRESHOLD_SCORE_MAX_DELTA);
    private_nh.param("/monitor/min_stable_samples", min_stable_samples_, NDT_MIN_STABLE_SAMPLES);
    private_nh.param("/monitor/fatal_time_threshold", fatal_time_threshold_, NDT_TIME_TO_FATAL_PREDICTIONS);


    //swap values in case of error
    if (iteration_threshold_warning_ > iteration_threshold_stop_) {
        auto aux = iteration_threshold_stop_;
        iteration_threshold_stop_ = iteration_threshold_warning_;
        iteration_threshold_warning_ = aux;
    }

    // Subscribers
    ros::Subscriber ndt_stat_sub = nh.subscribe("/ndt_stat", 10, &ROSNDTMatchingMonitor::ndt_stat_callback, this);
    ros::Subscriber ndt_pose_sub = nh.subscribe("/ndt_pose", 10, &ROSNDTMatchingMonitor::ndt_pose_callback, this);
    ros::Subscriber initial_pose_sub = nh.subscribe("/initialpose", 10, &ROSNDTMatchingMonitor::initialpose_callback, this);
    ros::Subscriber gnss_sub = nh.subscribe("gnss_pose", 10, &ROSNDTMatchingMonitor::gnss_callback, this);

    // Publishers
    initialpose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    overlay_info_text_pub_ = nh.advertise<jsk_rviz_plugins::OverlayText>("/ndt_monitor/ndt_info_text", 1);
    ndt_status_pub_ = nh.advertise<std_msgs::String>("/ndt_monitor/ndt_status", 1);

    ros::spin();

}

ROSNDTMatchingMonitor::ROSNDTMatchingMonitor()
{
    gnss_pose_available_ = false;
    last_score_     = 0.0;
    stable_samples_ = 0;
    prediction_samples_ = 0;
    ndt_status_     = ndt_status::NDT_NOT_INITIALIZED;
    initialized_    = false;
    gnss_text_ = " - NO GNSS available";

    ndt_normal_text_.width  = 800;
    ndt_normal_text_.height = 100;
    ndt_normal_text_.left   = 10;
    ndt_normal_text_.top    = 10;
    ndt_normal_text_.text_size  = 16;
    ndt_normal_text_.line_width = 2;

    ndt_normal_text_.bg_color.r = 0.f;
    ndt_normal_text_.bg_color.g = 0.f;
    ndt_normal_text_.bg_color.b = 0.f;
    ndt_normal_text_.bg_color.a = 0.2f;

    ndt_normal_text_.text = "NDT MONITOR - OK";
    ndt_normal_text_.fg_color.r = 0.0f;
    ndt_normal_text_.fg_color.g = 1.f;
    ndt_normal_text_.fg_color.b = 0.0f;
    ndt_normal_text_.fg_color.a = 1.0f;

    ndt_warn_text_ = ndt_normal_text_;
    ndt_warn_text_.text = "NDT MONITOR - WARNING";
    ndt_warn_text_.fg_color.r = 1.0f;
    ndt_warn_text_.fg_color.g = 0.6f;

    ndt_not_ready_text_ = ndt_warn_text_;
    ndt_not_ready_text_.text = "NDT MONITOR - NOT INITIALIZED";

    ndt_error_text_ = ndt_normal_text_;
    ndt_error_text_.text = "NDT MONITOR - ERROR \n TRYING LAST CORRECT LOCALIZATION";
    ndt_error_text_.fg_color.r = 1.0f;
    ndt_error_text_.fg_color.g = 0.f;

    ndt_fatal_text_ = ndt_error_text_;
    ndt_fatal_text_.text = "NDT MONITOR - FATAL CANNOT RECOVER AUTOMATICALLY";
}