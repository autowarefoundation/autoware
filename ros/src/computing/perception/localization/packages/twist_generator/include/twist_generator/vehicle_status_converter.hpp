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
 */

#include <ros/ros.h>

#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_msgs/VehicleStatus.h>
#include <nav_msgs/Odometry.h>

class VehicleStatusConverter
{
public:
    VehicleStatusConverter();
    ~VehicleStatusConverter();

private:
    ros::NodeHandle nh_, pnh_;
    ros::Subscriber sub_vehicle_status_, sub_estimate_twist_;
    ros::Publisher pub_twist_, pub_correction_coeff_;
    autoware_msgs::VehicleStatus current_vehicle_status_;

    /* parameters for twist calculation */
    double wheelbase_;
    double steering_gear_ratio_;

    /* for adaptive estimation */
    bool enable_adaptive_estimate_;
    double Pn_vx_, Pn_wz_, rho_, adaptive_coefficient_vx_, adaptive_coefficient_wz_;

    void callbackVehicleStatus(const autoware_msgs::VehicleStatus &msg);
    void callbackEstimateTwist(const geometry_msgs::TwistStamped &estimate_twist);
    void updateAdaptiveCoeffAngvel(const double &w_ndt, const double &vel, const double &steer);
    void updateAdaptiveCoeffVel(const double &v_ndt, const double &vel);
};
