/*
 * robot_control
 * Copyright (c) 2015, Robotnik Automation, SLL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robotnik Automation, SLL. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \author Robotnik
 * \brief Controller for an ackermann car robot in ackermann-steering (single)
    Control steering (2 direction axes) and traction axes (4W) of the RBCAR single Ackerman drive kinematics
    transforms the commands received from the controller in motor position / velocity commands
 */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "diagnostic_updater/publisher.h"


#define PI 3.1415926535

using namespace std;

class SimController {

public:

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    double desired_freq_;

    ros::Time last_time, current_time;

    // Diagnostics
    diagnostic_updater::Updater diagnostic_;				// General status diagnostic updater
    diagnostic_updater::FrequencyStatus freq_diag_;		// Component frequency diagnostics
    diagnostic_updater::HeaderlessTopicDiagnostic *subs_command_freq; // Topic reception frequency diagnostics
    ros::Time last_command_time_;					// Last moment when the component received a command
    diagnostic_updater::FunctionDiagnosticTask command_freq_;

    // Robot model
    std::string robot_model_;

    // Velocity and position references to low level controllers
    ros::Publisher ref_vel_flw_;
    ros::Publisher ref_vel_frw_;
    ros::Publisher ref_vel_blw_;
    ros::Publisher ref_vel_brw_;
    ros::Publisher ref_pos_flw_;
    ros::Publisher ref_pos_frw_;

    // Joint states published by the joint_state_controller of the Controller Manager
    ros::Subscriber joint_state_sub_;

    // High level robot command
    ros::Subscriber cmd_sub_;

    // Ackermann Topics - control action - traction - velocity
    std::string frw_vel_topic_;
    std::string flw_vel_topic_;
    std::string brw_vel_topic_;
    std::string blw_vel_topic_;

    // Ackerman Topics - control action - steering - position
    std::string frw_pos_topic_;
    std::string flw_pos_topic_;

    // Joint names - traction - velocity
    std::string joint_front_right_wheel;
    std::string joint_front_left_wheel;
    std::string joint_back_left_wheel;
    std::string joint_back_right_wheel;

    // Joint names - steering - position
    std::string joint_front_right_steer;
    std::string joint_front_left_steer;

    // Indexes to joint_states
    int frw_vel_, flw_vel_, blw_vel_, brw_vel_;
    int frw_pos_, flw_pos_;

    // Robot Positions
    double _x_;
    double _y_;
    double _th_;
    double _vx_;
    double _vy_;
    double _vth_;

    // Robot Joint States
    sensor_msgs::JointState joint_state_;

    // Command reference
    ackermann_msgs::AckermannDriveStamped base_vel_msg_;

    // External speed references
    double v_ref_;
    double alfa_ref_;
    double pos_ref_pan_;
    double pos_ref_tilt_;

    // Flag to indicate if joint_state has been read
    bool read_state_;

    // Robot configuration parameters
    double wheel_diameter_;
    double wheelbase_;
    double max_speed_;
    double max_steering_;

    // diagnostic frequencies
    double min_command_rec_freq_;
    double max_command_rec_freq_;

    // accepted time deviation to process joint_sttate
    double joints_state_time_window_;

    // Parameter that defines if odom tf is published or not
    bool publish_odom_tf_;

    // Publisher for odom topic
    ros::Publisher odom_pub_;

    // Broadcaster for odom tf
    tf::TransformBroadcaster odom_broadcaster;


    SimController(ros::NodeHandle h) :  diagnostic_(),
                                        nh_(h), private_nh_("~"),
                                        desired_freq_(100),
                                        freq_diag_(diagnostic_updater::FrequencyStatusParam(&desired_freq_, &desired_freq_, 0.05)   ),
                                        command_freq_("Command frequency check", boost::bind(&SimController::check_command_subscriber, this, _1)) {

        ROS_INFO("sd_control_node - Init ");

        ros::NodeHandle sd_control_node_handle(nh_, "sd_control");

        // Get robot model from the parameters
        if (!private_nh_.getParam("model", robot_model_)) {
            ROS_ERROR("Robot model not defined.");
            exit(-1);
        }
        else ROS_INFO("Robot Model : %s", robot_model_.c_str());

        // Ackermann configuration - traction - topics
        private_nh_.param<std::string>("frw_vel_topic", frw_vel_topic_, robot_model_ + "/front_right_wheel_joint_controller/command");
        private_nh_.param<std::string>("flw_vel_topic", flw_vel_topic_, robot_model_ + "/front_left_wheel_joint_controller/command");
        private_nh_.param<std::string>("blw_vel_topic", blw_vel_topic_, robot_model_ + "/rear_left_wheel_joint_controller/command");
        private_nh_.param<std::string>("brw_vel_topic", brw_vel_topic_, robot_model_ + "/rear_right_wheel_joint_controller/command");

        // Ackermann configuration - traction - joint names
        private_nh_.param<std::string>("joint_front_right_wheel", joint_front_right_wheel, "front_right_wheel_joint");
        private_nh_.param<std::string>("joint_front_left_wheel", joint_front_left_wheel, "front_left_wheel_joint");
        private_nh_.param<std::string>("joint_back_left_wheel", joint_back_left_wheel, "rear_left_wheel_joint");
        private_nh_.param<std::string>("joint_back_right_wheel", joint_back_right_wheel, "rear_right_wheel_joint");

        // Ackermann configuration - direction - topics
        private_nh_.param<std::string>("frw_pos_topic", frw_pos_topic_, robot_model_ + "/front_right_steer_joint_controller/command");
        private_nh_.param<std::string>("flw_pos_topic", flw_pos_topic_, robot_model_ + "/front_left_steer_joint_controller/command");

        private_nh_.param<std::string>("joint_front_right_steer", joint_front_right_steer, "front_right_steer_joint");
        private_nh_.param<std::string>("joint_front_left_steer", joint_front_left_steer, "front_left_steer_joint");

        // Robot parameters
        if (private_nh_.getParam("wheelbase", wheelbase_)) {
            ROS_INFO("Wheelbase is set to %f", wheelbase_);
        } else {
            throw ros::Exception("No wheelbase_ specified. Quitting..");
        }

        if (private_nh_.getParam("wheel_diameter", wheel_diameter_)) {
            ROS_INFO("Wheel diameter is set to %f", wheel_diameter_);
        } else {
            throw ros::Exception("No wheel diameter specified. Quitting..");
        }

        private_nh_.param<bool>("publish_odom_tf", publish_odom_tf_, true);
        if (publish_odom_tf_)
            ROS_INFO("PUBLISHING odom->vehicle_base tf");
        else
            ROS_INFO("NOT PUBLISHING odom->vehicle_base tf");

        if (private_nh_.getParam("max_speed", max_speed_)) {
            ROS_INFO("Max speed is set to %f", max_speed_);
        } else {
            throw ros::Exception("No max speed specified. Quitting..");
        }

        if (private_nh_.getParam("max_steering", max_steering_)) {
            ROS_INFO("Max steering is set to %f", max_steering_);
        } else {
            throw ros::Exception("No max_steering specified. Quitting..");
        }

        private_nh_.param<double>("joints_state_time_window", joints_state_time_window_, 1.0);
        private_nh_.param<double>("min_command_rec_freq", min_command_rec_freq_, 5.0);
        private_nh_.param<double>("max_command_rec_freq", max_command_rec_freq_, 150.0);

        // Robot Positions
        _x_ = 0.0;
        _y_ = 0.0;
        _th_ = 0.0;
        _vx_ = 0.0;
        _vy_ = 0.0;
        _vth_ = 0.0;

        current_time = ros::Time::now();
        last_time = current_time;

        // Robot state space control references
        v_ref_ = 0.0;
        alfa_ref_ = 0.0;
        pos_ref_pan_ = 0.0;

        // Subscribers
        joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("joint_states", 1, &SimController::jointStateCallback, this);
        cmd_sub_ = sd_control_node_handle.subscribe<ackermann_msgs::AckermannDriveStamped>("command", 1, &SimController::commandCallback, this);

        // Adevertise reference topics for the controllers
        ref_vel_frw_ = nh_.advertise<std_msgs::Float64>( frw_vel_topic_, 50);
        ref_vel_flw_ = nh_.advertise<std_msgs::Float64>( flw_vel_topic_, 50);
        ref_vel_blw_ = nh_.advertise<std_msgs::Float64>( blw_vel_topic_, 50);
        ref_vel_brw_ = nh_.advertise<std_msgs::Float64>( brw_vel_topic_, 50);
        ref_pos_frw_ = nh_.advertise<std_msgs::Float64>( frw_pos_topic_, 50);
        ref_pos_flw_ = nh_.advertise<std_msgs::Float64>( flw_pos_topic_, 50);

        // Publish odometry
        odom_pub_ = private_nh_.advertise<nav_msgs::Odometry>("odom", 400);

        // Component frequency diagnostics
        diagnostic_.setHardwareID("sd_control - simulation");
        diagnostic_.add( freq_diag_ );
        diagnostic_.add( command_freq_ );

        subs_command_freq = new diagnostic_updater::HeaderlessTopicDiagnostic("~command", diagnostic_,
                            diagnostic_updater::FrequencyStatusParam(&min_command_rec_freq_, &max_command_rec_freq_, 0.1, 10));
        subs_command_freq->addTask(&command_freq_); // Adding an additional task to the control

        // Flag to indicate joint_state has been read
        read_state_ = false;
    }

    /// Controller startup in realtime
    int starting() {
        // Initialize joint indexes according to joint names
        if (read_state_) {
            vector<string> joint_names = joint_state_.name;
            frw_vel_ = find (joint_names.begin(),joint_names.end(), string(joint_front_right_wheel)) - joint_names.begin();
            flw_vel_ = find (joint_names.begin(),joint_names.end(), string(joint_front_left_wheel)) - joint_names.begin();
            blw_vel_ = find (joint_names.begin(),joint_names.end(), string(joint_back_left_wheel)) - joint_names.begin();
            brw_vel_ = find (joint_names.begin(),joint_names.end(), string(joint_back_right_wheel)) - joint_names.begin();
            frw_pos_ = find (joint_names.begin(),joint_names.end(), string(joint_front_right_steer)) - joint_names.begin();
            flw_pos_ = find (joint_names.begin(),joint_names.end(), string(joint_front_left_steer)) - joint_names.begin();
            return 0;
        } else {
            ROS_WARN("SimController::starting: joint_states are not being received");
            return -1;
        }
    }

    /// Controller update loop
    void UpdateControl() {
        // Compute state control actions
        // State feedback error 4 position loops / 4 velocity loops
        // Single steering
        double d1;
        double alfa_ref_left = 0.0;
        double alfa_ref_right = 0.0;
        if (alfa_ref_!=0.0) {  // div/0
            d1 =  wheelbase_ / tan (alfa_ref_);
            alfa_ref_left = atan2( wheelbase_, d1 - 0.105);
            alfa_ref_right = atan2( wheelbase_, d1 + 0.105);
            if (alfa_ref_<0.0) {
                alfa_ref_left = alfa_ref_left - PI;
                alfa_ref_right = alfa_ref_right - PI;
            }
        } else {
            alfa_ref_left = 0.0;
            alfa_ref_right = 0.0;
        }

        // Angular position ref publish
        std_msgs::Float64 frw_ref_pos_msg;
        std_msgs::Float64 flw_ref_pos_msg;
        std_msgs::Float64 brw_ref_pos_msg;
        std_msgs::Float64 blw_ref_pos_msg;

        flw_ref_pos_msg.data = alfa_ref_left;
        frw_ref_pos_msg.data = alfa_ref_right;

        // Linear speed ref publish (could be improved by setting correct speed to each wheel according to turning state
        // w = v_mps / (PI * D);   w_rad = w * 2.0 * PI
        double ref_speed_joint = 2.0 * v_ref_ / wheel_diameter_;

        std_msgs::Float64 frw_ref_vel_msg;
        std_msgs::Float64 flw_ref_vel_msg;
        std_msgs::Float64 brw_ref_vel_msg;
        std_msgs::Float64 blw_ref_vel_msg;
        frw_ref_vel_msg.data = -ref_speed_joint;
        flw_ref_vel_msg.data = -ref_speed_joint;
        brw_ref_vel_msg.data = -ref_speed_joint;
        blw_ref_vel_msg.data = -ref_speed_joint;

        // Publish msgs traction and direction
        ref_vel_frw_.publish( frw_ref_vel_msg );
        ref_vel_flw_.publish( flw_ref_vel_msg );
        ref_vel_blw_.publish( blw_ref_vel_msg );
        ref_vel_brw_.publish( brw_ref_vel_msg );
        ref_pos_frw_.publish( frw_ref_pos_msg );
        ref_pos_flw_.publish( flw_ref_pos_msg );
    }

    // Update robot odometry depending on kinematic configuration
    void UpdateOdometry() {
        // Get angles
        double a1, a2;

        if( (ros::Time::now() - joint_state_.header.stamp).toSec() > joints_state_time_window_){
            ROS_WARN_THROTTLE(2, "SimController::UpdateOdometry: joint_states are not being received");
            return;
        }

        a1 = joint_state_.position[frw_pos_];
        a2 = joint_state_.position[flw_pos_];

        // Linear speed of each wheel [mps]
        double v3, v4;
        // filtering noise from the Velocity controller when the speed is 0.0 (by using an open loop with desired speed)
        if( v_ref_ == 0.0) {
            v3 = 0.0;
            v4 = 0.0;
        } else {
            v3 = joint_state_.velocity[blw_vel_] * (wheel_diameter_ / 2.0);
            v4 = joint_state_.velocity[brw_vel_] * (wheel_diameter_ / 2.0);
        }
        // Turning angle front
        double fBetaRads = (a1 + a2) / 2.0;

        // Linear speed
        double fSamplePeriod = 1.0 / desired_freq_;  // Default sample period
        double v_mps = -(v3 + v4) / 2.0;

        current_time = ros::Time::now();

        _vx_ = v_mps * cos(_th_);
        _vy_ = v_mps * sin(_th_);
        _vth_ = (tan(fBetaRads) * v_mps) / wheelbase_;

        double dt = current_time.toSec() - last_time.toSec();
        double delta_x = _vx_ * dt;
        double delta_y = _vy_ * dt;
        double delta_th = _vth_ * dt;

        _x_ += delta_x;
        _y_ += delta_y;
        _th_ += delta_th;

        last_time = current_time;
    }

    // Publish robot odometry tf and topic depending
    void PublishOdometry() {
        //first, we'll publish the transform over tf
        // TODO change to tf_prefix
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "vehicle_base";

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(_th_);

        odom_trans.transform.translation.x = _x_;
        odom_trans.transform.translation.y = _y_;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        // send the transform over /tf
        // activate / deactivate with param
        // this tf in needed when not using robot_pose_ekf
        if (publish_odom_tf_) odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        // Position
        odom.pose.pose.position.x = _x_;
        odom.pose.pose.position.y = _y_;
        odom.pose.pose.position.z = 0.0;
        // Orientation
        odom.pose.pose.orientation = odom_quat;
        // Pose covariance
        for(int i = 0; i < 6; i++)
            odom.pose.covariance[i*6+i] = 0.1;  // test 0.001

        //set the velocity
        odom.child_frame_id = "vehicle_base";
        // Linear velocities
        odom.twist.twist.linear.x = _vx_;
        odom.twist.twist.linear.y = _vy_;
        odom.twist.twist.linear.z = 0.0;
        // Angular velocities
        odom.twist.twist.angular.z = _vth_;
        // Twist covariance
        for(int i = 0; i < 6; i++)
            odom.twist.covariance[6*i+i] = 0.1;  // test 0.001

        //publish the message
        odom_pub_.publish(odom);
    }

    /// Controller stopping
    void stopping()
    {}


/*
 *	\brief Checks that the robot is receiving at a correct frequency the command messages. Diagnostics
 */
    void check_command_subscriber(diagnostic_updater::DiagnosticStatusWrapper &stat) {
        ros::Time current_time = ros::Time::now();

        double diff = current_time.toSec() - last_time.toSec();

        if(diff > 1.0) {
            stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Topic is not receiving commands");
            //ROS_INFO("check_command_subscriber: %lf seconds without commands", diff);
            // If no command is received, set Speed References to 0
            // Turning angle can stay in the previous position.
            v_ref_ = 0.0;
        } else {
            stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Topic receiving commands");
        }
    }


    // Set the base velocity command
    void setCommand(const ackermann_msgs::AckermannDriveStamped &msg) {
        v_ref_ = saturation(msg.drive.speed, -max_speed_, max_speed_);
        alfa_ref_ = saturation(msg.drive.steering_angle, -max_steering_, max_steering_);
    }

    // Topic command
    void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg) {
        joint_state_ = *msg;
        read_state_ = true;
    }

    // Topic command
    void commandCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg) {
        // Safety check
        last_command_time_ = ros::Time::now();
        subs_command_freq->tick();			// For diagnostics

        base_vel_msg_ = *msg;
        this->setCommand(base_vel_msg_);
    }

    double saturation(double u, double min, double max) {
        if (u>max) u=max;
        if (u<min) u=min;
        return u;
    }

    double radnorm( double value ) {
        while (value > PI) value -= PI;
        while (value < -PI) value += PI;
        return value;
    }

    double radnorm2( double value ) {
        while (value > 2.0*PI) value -= 2.0*PI;
        while (value < -2.0*PI) value += 2.0*PI;
        return value;
    }

    bool spin()
    {
        ROS_INFO("sd_control::spin()");
        ros::Rate r(desired_freq_);

        while (!ros::isShuttingDown()) // Using ros::isShuttingDown to avoid restarting the node during a shutdown.
        {
            if (starting() == 0)
            {
                while(ros::ok() && nh_.ok()) {
                    UpdateControl();
                    UpdateOdometry();
                    PublishOdometry();
                    diagnostic_.update();
                    ros::spinOnce();
                    r.sleep();
                }
                ROS_INFO("END OF ros::ok() !!!");
            } else {
                // No need for diagnostic here since a broadcast occurs in start
                // when there is an error.
                usleep(1000000);
                ros::spinOnce();
            }
        }

        return true;
    }

}; // Class SimController

int main(int argc, char** argv) {
    ros::init(argc, argv, "sd_control");

    ros::NodeHandle n;
    SimController scc(n);
    scc.spin();

    return (0);
}
