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

#include <geometry_msgs/TwistStamped.h>
#include <autoware_msgs/VehicleStatus.h>
#include <nav_msgs/Odometry.h>

static const double KMPH2MPS = 1000.0 / 3600.0;
static const double DEG2RAD = 3.141592 / 180.0;

class TwistGenerator
{
public:
  TwistGenerator() : nh_(""), pnh_("~")
  {
    pub_twist_ = nh_.advertise<geometry_msgs::TwistStamped>("/can_twist", 1);
    pub_twist_adapted_ = nh_.advertise<geometry_msgs::TwistStamped>("/can_twist_adapted", 1);
    sub_vehicle_status_ = nh_.subscribe("/vehicle_status", 1, &TwistGenerator::callbackVehicleStatus, this);
    sub_estimate_twist_ = nh_.subscribe("/estimate_twist", 1, &TwistGenerator::callbackEstimateTwist, this);

    pnh_.param("wheelbase", wheelbase_, double(2.9));
    pnh_.param("steering_gear_ratio", steering_gear_ratio_, double(15.0));
    pnh_.param("enable_adaptive_estimate", enable_adaptive_estimate_, bool(false));

    if (wheelbase_ < 1.0E-5)
    {
      ROS_WARN("undesired wheelbase value : %f, set to 1.0", wheelbase_);
      wheelbase_ = 1.0;
    }
    if (steering_gear_ratio_ < 1.0E-5)
    {
      ROS_WARN("undesired steering_gear_ratio value : %f, set to 1.0", steering_gear_ratio_);
      steering_gear_ratio_ = 1.0;
    }
    ROS_INFO("set as\n * wheelbase: %f\n * steering_gear_ratio: %f", wheelbase_, steering_gear_ratio_);

    adaptive_coefficient_ = 1.0; // adaptive coefficient for angular velocity calculaton
    Pn_ = 1000.0;                // initial covariance
    rho_ = 0.999;                // forgetting factor
  };
  ~TwistGenerator(){};

private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_vehicle_status_, sub_estimate_twist_;
  ros::Publisher pub_twist_, pub_twist_adapted_;
  autoware_msgs::VehicleStatus current_vehicle_status_;

  /* parameters for twist calculation */
  double wheelbase_;
  double steering_gear_ratio_;

  /* for adaptive estimation */
  bool enable_adaptive_estimate_;
  double Pn_, rho_, adaptive_coefficient_;

  void callbackVehicleStatus(const autoware_msgs::VehicleStatus &msg)
  {
    current_vehicle_status_ = msg;
    const double vel_mps = msg.speed * KMPH2MPS;                         // convert from [km/h] to [m/s]
    const double steer_rad = msg.angle / steering_gear_ratio_ * DEG2RAD; // convert handle angle [deg] to tire angle [rad]

    geometry_msgs::TwistStamped twist_stamped;
    twist_stamped.header = msg.header;
    twist_stamped.twist.linear.x = vel_mps;
    twist_stamped.twist.angular.z = vel_mps * std::tan(steer_rad) / wheelbase_;
    if (enable_adaptive_estimate_)
    {
      twist_stamped.twist.angular.z *= adaptive_coefficient_;
    }
    pub_twist_.publish(twist_stamped);
  }

  void callbackEstimateTwist(const geometry_msgs::TwistStamped &estimate_twist)
  {
    const double w_ndt = estimate_twist.twist.angular.z;
    const double vel_mps = current_vehicle_status_.speed * KMPH2MPS;
    const double steer_rad = current_vehicle_status_.angle / steering_gear_ratio_ * DEG2RAD;

    if (vel_mps > 1.0) // update only when estimate angular vel is reliable enough
    {
      updateAdaptiveCoeff(w_ndt, vel_mps, steer_rad);
    }
  }

  void updateAdaptiveCoeff(const double &w_ndt, const double &vel, const double &steer)
  {
    /* estimate wheelbase by Recursive Least Squares Method */
    const double zn = vel * std::tan(steer) / wheelbase_;
    const double num = rho_ + zn * Pn_ * zn;
    Pn_ = (Pn_ - (Pn_ * zn * zn * Pn_) / num) / rho_; // update estimate variance
    const double temp = adaptive_coefficient_ + (Pn_ * zn / num) * (w_ndt - zn * adaptive_coefficient_);
    adaptive_coefficient_ = std::max(std::min(temp, 1.2), 0.8); // limit changes

    printf("adaptive_coefficient_ = %f, Pn = %f\n", adaptive_coefficient_, Pn_);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "twist_generator");
  TwistGenerator obj;
  ros::spin();
  return 0;
};