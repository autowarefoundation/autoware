// Copyright 2015-2019 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ndt_scan_matcher/util_func.hpp"

#include "ndt_scan_matcher/matrix_type.hpp"

static std::random_device seed_gen;

// ref by http://takacity.blog.fc2.com/blog-entry-69.html
std_msgs::msg::ColorRGBA exchange_color_crc(double x)
{
  std_msgs::msg::ColorRGBA color;

  x = std::max(x, 0.0);
  x = std::min(x, 0.9999);

  if (x <= 0.25) {
    color.b = 1.0;
    color.g = std::sin(x * 2.0 * M_PI);
    color.r = 0;
  } else if (x > 0.25 && x <= 0.5) {
    color.b = std::sin(x * 2 * M_PI);
    color.g = 1.0;
    color.r = 0;
  } else if (x > 0.5 && x <= 0.75) {
    color.b = 0;
    color.g = 1.0;
    color.r = -std::sin(x * 2.0 * M_PI);
  } else {
    color.b = 0;
    color.g = -std::sin(x * 2.0 * M_PI);
    color.r = 1.0;
  }
  color.a = 0.999;
  return color;
}

double calc_diff_for_radian(const double lhs_rad, const double rhs_rad)
{
  double diff_rad = lhs_rad - rhs_rad;
  if (diff_rad > M_PI) {
    diff_rad = diff_rad - 2 * M_PI;
  } else if (diff_rad < -M_PI) {
    diff_rad = diff_rad + 2 * M_PI;
  }
  return diff_rad;
}

Eigen::Map<const RowMatrixXd> makeEigenCovariance(const std::array<double, 36> & covariance)
{
  return Eigen::Map<const RowMatrixXd>(covariance.data(), 6, 6);
}

// x: roll, y: pitch, z: yaw
geometry_msgs::msg::Vector3 get_rpy(const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::Vector3 rpy;
  tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf2::Matrix3x3(q).getRPY(rpy.x, rpy.y, rpy.z);
  return rpy;
}

geometry_msgs::msg::Vector3 get_rpy(const geometry_msgs::msg::PoseStamped & pose)
{
  return get_rpy(pose.pose);
}

geometry_msgs::msg::Vector3 get_rpy(const geometry_msgs::msg::PoseWithCovarianceStamped & pose)
{
  return get_rpy(pose.pose.pose);
}

geometry_msgs::msg::Twist calc_twist(
  const geometry_msgs::msg::PoseStamped & pose_a, const geometry_msgs::msg::PoseStamped & pose_b)
{
  const rclcpp::Duration dt = rclcpp::Time(pose_b.header.stamp) - rclcpp::Time(pose_a.header.stamp);
  const double dt_s = dt.seconds();

  if (dt_s == 0) {
    return geometry_msgs::msg::Twist();
  }

  const auto pose_a_rpy = get_rpy(pose_a);
  const auto pose_b_rpy = get_rpy(pose_b);

  geometry_msgs::msg::Vector3 diff_xyz;
  geometry_msgs::msg::Vector3 diff_rpy;

  diff_xyz.x = pose_b.pose.position.x - pose_a.pose.position.x;
  diff_xyz.y = pose_b.pose.position.y - pose_a.pose.position.y;
  diff_xyz.z = pose_b.pose.position.z - pose_a.pose.position.z;
  diff_rpy.x = calc_diff_for_radian(pose_b_rpy.x, pose_a_rpy.x);
  diff_rpy.y = calc_diff_for_radian(pose_b_rpy.y, pose_a_rpy.y);
  diff_rpy.z = calc_diff_for_radian(pose_b_rpy.z, pose_a_rpy.z);

  geometry_msgs::msg::Twist twist;
  twist.linear.x = diff_xyz.x / dt_s;
  twist.linear.y = diff_xyz.y / dt_s;
  twist.linear.z = diff_xyz.z / dt_s;
  twist.angular.x = diff_rpy.x / dt_s;
  twist.angular.y = diff_rpy.y / dt_s;
  twist.angular.z = diff_rpy.z / dt_s;

  return twist;
}

void get_nearest_timestamp_pose(
  const std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr> &
    pose_cov_msg_ptr_array,
  const rclcpp::Time & time_stamp,
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr & output_old_pose_cov_msg_ptr,
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr & output_new_pose_cov_msg_ptr)
{
  for (const auto & pose_cov_msg_ptr : pose_cov_msg_ptr_array) {
    output_new_pose_cov_msg_ptr =
      std::const_pointer_cast<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_cov_msg_ptr);
    const rclcpp::Time pose_time_stamp = output_new_pose_cov_msg_ptr->header.stamp;
    if (pose_time_stamp > time_stamp) {
      // TODO(Tier IV): refactor
      const rclcpp::Time old_pose_time_stamp = output_old_pose_cov_msg_ptr->header.stamp;
      if (old_pose_time_stamp.seconds() == 0.0) {
        output_old_pose_cov_msg_ptr = output_new_pose_cov_msg_ptr;
      }
      break;
    }
    output_old_pose_cov_msg_ptr = output_new_pose_cov_msg_ptr;
  }
}

geometry_msgs::msg::PoseStamped interpolate_pose(
  const geometry_msgs::msg::PoseStamped & pose_a, const geometry_msgs::msg::PoseStamped & pose_b,
  const rclcpp::Time & time_stamp)
{
  const rclcpp::Time pose_a_time_stamp = pose_a.header.stamp;
  const rclcpp::Time pose_b_time_stamp = pose_b.header.stamp;
  if (
    (pose_a_time_stamp.seconds() == 0.0) || (pose_b_time_stamp.seconds() == 0.0) ||
    (time_stamp.seconds() == 0.0)) {
    return geometry_msgs::msg::PoseStamped();
  }

  const auto twist = calc_twist(pose_a, pose_b);
  const double dt = (time_stamp - pose_a_time_stamp).seconds();

  const auto pose_a_rpy = get_rpy(pose_a);

  geometry_msgs::msg::Vector3 xyz;
  geometry_msgs::msg::Vector3 rpy;
  xyz.x = pose_a.pose.position.x + twist.linear.x * dt;
  xyz.y = pose_a.pose.position.y + twist.linear.y * dt;
  xyz.z = pose_a.pose.position.z + twist.linear.z * dt;
  rpy.x = pose_a_rpy.x + twist.angular.x * dt;
  rpy.y = pose_a_rpy.y + twist.angular.y * dt;
  rpy.z = pose_a_rpy.z + twist.angular.z * dt;

  tf2::Quaternion tf_quaternion;
  tf_quaternion.setRPY(rpy.x, rpy.y, rpy.z);

  geometry_msgs::msg::PoseStamped pose;
  pose.header = pose_a.header;
  pose.header.stamp = time_stamp;
  pose.pose.position.x = xyz.x;
  pose.pose.position.y = xyz.y;
  pose.pose.position.z = xyz.z;
  pose.pose.orientation = tf2::toMsg(tf_quaternion);
  return pose;
}

geometry_msgs::msg::PoseStamped interpolate_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose_a,
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose_b, const rclcpp::Time & time_stamp)
{
  geometry_msgs::msg::PoseStamped tmp_pose_a;
  tmp_pose_a.header = pose_a.header;
  tmp_pose_a.pose = pose_a.pose.pose;

  geometry_msgs::msg::PoseStamped tmp_pose_b;
  tmp_pose_b.header = pose_b.header;
  tmp_pose_b.pose = pose_b.pose.pose;

  return interpolate_pose(tmp_pose_a, tmp_pose_b, time_stamp);
}

void pop_old_pose(
  std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr> &
    pose_cov_msg_ptr_array,
  const rclcpp::Time & time_stamp)
{
  while (!pose_cov_msg_ptr_array.empty()) {
    if (rclcpp::Time(pose_cov_msg_ptr_array.front()->header.stamp) >= time_stamp) {
      break;
    }
    pose_cov_msg_ptr_array.pop_front();
  }
}

Eigen::Affine3d pose_to_affine3d(const geometry_msgs::msg::Pose & ros_pose)
{
  Eigen::Affine3d eigen_pose;
  tf2::fromMsg(ros_pose, eigen_pose);
  return eigen_pose;
}

Eigen::Matrix4f pose_to_matrix4f(const geometry_msgs::msg::Pose & ros_pose)
{
  Eigen::Affine3d eigen_pose_affine = pose_to_affine3d(ros_pose);
  Eigen::Matrix4f eigen_pose_matrix = eigen_pose_affine.matrix().cast<float>();
  return eigen_pose_matrix;
}

Eigen::Vector3d point_to_vector3d(const geometry_msgs::msg::Point & ros_pos)
{
  Eigen::Vector3d eigen_pos;
  eigen_pos.x() = ros_pos.x;
  eigen_pos.y() = ros_pos.y;
  eigen_pos.z() = ros_pos.z;
  return eigen_pos;
}

geometry_msgs::msg::Pose matrix4f_to_pose(const Eigen::Matrix4f & eigen_pose_matrix)
{
  Eigen::Affine3d eigen_pose_affine;
  eigen_pose_affine.matrix() = eigen_pose_matrix.cast<double>();
  geometry_msgs::msg::Pose ros_pose = tf2::toMsg(eigen_pose_affine);
  return ros_pose;
}

std::vector<geometry_msgs::msg::Pose> create_random_pose_array(
  const geometry_msgs::msg::PoseWithCovarianceStamped & base_pose_with_cov, const int particle_num)
{
  std::default_random_engine engine(seed_gen());
  const Eigen::Map<const RowMatrixXd> covariance =
    makeEigenCovariance(base_pose_with_cov.pose.covariance);

  std::normal_distribution<> x_distribution(0.0, std::sqrt(covariance(0, 0)));
  std::normal_distribution<> y_distribution(0.0, std::sqrt(covariance(1, 1)));
  std::normal_distribution<> z_distribution(0.0, std::sqrt(covariance(2, 2)));
  std::normal_distribution<> roll_distribution(0.0, std::sqrt(covariance(3, 3)));
  std::normal_distribution<> pitch_distribution(0.0, std::sqrt(covariance(4, 4)));
  std::normal_distribution<> yaw_distribution(0.0, std::sqrt(covariance(5, 5)));

  const auto base_rpy = get_rpy(base_pose_with_cov);

  std::vector<geometry_msgs::msg::Pose> poses;
  for (int i = 0; i < particle_num; ++i) {
    geometry_msgs::msg::Vector3 xyz;
    geometry_msgs::msg::Vector3 rpy;

    xyz.x = base_pose_with_cov.pose.pose.position.x + x_distribution(engine);
    xyz.y = base_pose_with_cov.pose.pose.position.y + y_distribution(engine);
    xyz.z = base_pose_with_cov.pose.pose.position.z + z_distribution(engine);
    rpy.x = base_rpy.x + roll_distribution(engine);
    rpy.y = base_rpy.y + pitch_distribution(engine);
    rpy.z = base_rpy.z + yaw_distribution(engine);

    tf2::Quaternion tf_quaternion;
    tf_quaternion.setRPY(rpy.x, rpy.y, rpy.z);

    geometry_msgs::msg::Pose pose;
    pose.position.x = xyz.x;
    pose.position.y = xyz.y;
    pose.position.z = xyz.z;
    pose.orientation = tf2::toMsg(tf_quaternion);

    poses.push_back(pose);
  }

  return poses;
}

double norm(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2)
{
  const double dx = p1.x - p2.x;
  const double dy = p1.y - p2.y;
  const double dz = p1.z - p2.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}
