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

#ifndef DATA_STRUCTS_H
#define DATA_STRUCTS_H

#include <cmath>
#include <iostream>
#include <pcl/point_types.h>

struct Pose {
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;

  Pose() : x(0), y(0), z(0), roll(0), pitch(0), yaw(0){};
  Pose(double x, double y, double z, double roll, double pitch, double yaw)
      : x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw){};
  void clear() { x = y = z = roll = pitch = yaw = 0; };

  Pose operator+(const Pose &rhs_pose) const {
    Pose tmp_pose;
    tmp_pose.x = x + rhs_pose.x;
    tmp_pose.y = y + rhs_pose.y;
    tmp_pose.z = z + rhs_pose.z;
    tmp_pose.roll = roll + rhs_pose.roll;
    tmp_pose.pitch = pitch + rhs_pose.pitch;
    tmp_pose.yaw = yaw + rhs_pose.yaw;
    return tmp_pose;
  };

  Pose operator-(const Pose &rhs_pose) const {
    Pose tmp_pose;
    tmp_pose.x = x - rhs_pose.x;
    tmp_pose.y = y - rhs_pose.y;
    tmp_pose.z = z - rhs_pose.z;
    tmp_pose.roll = calcDiffForRadian(roll, rhs_pose.roll);
    tmp_pose.pitch = calcDiffForRadian(pitch, rhs_pose.pitch);
    tmp_pose.yaw = calcDiffForRadian(yaw, rhs_pose.yaw);
    return tmp_pose;
  };

  Pose operator*(const double val) const {
    Pose tmp_pose;
    tmp_pose.x = x * val;
    tmp_pose.y = y * val;
    tmp_pose.z = z * val;
    tmp_pose.roll = roll * val;
    tmp_pose.pitch = pitch * val;
    tmp_pose.yaw = yaw * val;
    return tmp_pose;
  };

  Pose operator/(const double val) const {
    Pose tmp_pose;
    if (val == 0)
      return tmp_pose;

    tmp_pose.x = x / val;
    tmp_pose.y = y / val;
    tmp_pose.z = z / val;
    tmp_pose.roll = roll / val;
    tmp_pose.pitch = pitch / val;
    tmp_pose.yaw = yaw / val;
    return tmp_pose;
  };

  bool operator==(const Pose &rhs_pose) const {
    return (x == rhs_pose.x && y == rhs_pose.y && z == rhs_pose.z &&
            roll == rhs_pose.roll && pitch == rhs_pose.pitch &&
            yaw == rhs_pose.yaw);
  };

  bool operator!=(const Pose &rhs_pose) const {
    return !(x == rhs_pose.x && y == rhs_pose.y && z == rhs_pose.z &&
             roll == rhs_pose.roll && pitch == rhs_pose.pitch &&
             yaw == rhs_pose.yaw);
  };

  friend std::ostream &operator<<(std::ostream &os, const Pose &pose) {
    os << "x:" << pose.x << " y:" << pose.y << " z:" << pose.z
       << " roll:" << pose.roll / M_PI * 180.0
       << "[deg] pitch:" << pose.pitch / M_PI * 180.0
       << "[deg] yaw:" << pose.yaw / M_PI * 180.0 << "[deg]";
    return os;
  };

private:
  double calcDiffForRadian(const double lhs_rad, const double rhs_rad) const {
    double diff_rad = lhs_rad - rhs_rad;
    if (diff_rad > M_PI) {
      diff_rad = diff_rad - 2 * M_PI;
    } else if (diff_rad < -M_PI) {
      diff_rad = diff_rad + 2 * M_PI;
    }
    return diff_rad;
  }
};

struct PoseStamped {
  Pose pose;
  double stamp;

  PoseStamped() : pose(), stamp(0){};
  PoseStamped(const Pose &pose, const double stamp)
      : pose(pose), stamp(stamp){};

  void clear() {
    pose.clear();
    stamp = 0;
  }

  bool operator==(const PoseStamped &rhs_pose_stamped) const {
    return (pose == rhs_pose_stamped.pose && stamp == rhs_pose_stamped.stamp);
  };

  friend std::ostream &operator<<(std::ostream &os,
                                  const PoseStamped &pose_stamped) {
    os << pose_stamped.pose << " stamp:" << pose_stamped.stamp;
    return os;
  };
};

struct Linear {
  double x;
  double y;
  double z;

  Linear() : x(0), y(0), z(0){};

  void clear() { x = y = z = 0; };

  Linear operator+(const Linear &rhs_l) const {
    Linear tmp_l;
    tmp_l.x = x + rhs_l.x;
    tmp_l.y = y + rhs_l.y;
    tmp_l.z = z + rhs_l.z;
    return tmp_l;
  }

  Linear operator-(const Linear &rhs_l) const {
    Linear tmp_l;
    tmp_l.x = x - rhs_l.x;
    tmp_l.y = y - rhs_l.y;
    tmp_l.z = z - rhs_l.z;
    return tmp_l;
  }

  Linear operator*(const double val) const {
    Linear tmp_l;

    if (val == 0)
      return tmp_l;

    tmp_l.x = x * val;
    tmp_l.y = y * val;
    tmp_l.z = z * val;
    return tmp_l;
  }

  Linear operator/(const double val) const {
    Linear tmp_l;

    if (val == 0)
      return tmp_l;

    tmp_l.x = x / val;
    tmp_l.y = y / val;
    tmp_l.z = z / val;
    return tmp_l;
  }

  friend std::ostream &operator<<(std::ostream &os, const Linear &linear) {
    os << "x:" << linear.x << " y:" << linear.y << " z:" << linear.z;
    return os;
  };
};

struct Angular {
  double x;
  double y;
  double z;

  Angular() : x(0), y(0), z(0){};

  void clear() { x = y = z = 0; };

  Angular operator+(const Angular &rhs_a) const {
    Angular tmp_a;
    tmp_a.x = x + rhs_a.x;
    tmp_a.y = y + rhs_a.y;
    tmp_a.z = z + rhs_a.z;
    return tmp_a;
  }

  Angular operator-(const Angular &rhs_a) const {
    Angular tmp_a;
    tmp_a.x = x - rhs_a.x;
    tmp_a.y = y - rhs_a.y;
    tmp_a.z = z - rhs_a.z;
    return tmp_a;
  }

  Angular operator*(const double val) const {
    Angular tmp_a;
    tmp_a.x = x * val;
    tmp_a.y = y * val;
    tmp_a.z = z * val;
    return tmp_a;
  }

  Angular operator/(const double val) const {
    Angular tmp_a;

    if (val == 0)
      return tmp_a;

    tmp_a.x = x / val;
    tmp_a.y = y / val;
    tmp_a.z = z / val;
    return tmp_a;
  }

  friend std::ostream &operator<<(std::ostream &os, const Angular &angular) {
    os << "x:" << angular.x << " y:" << angular.y << " z:" << angular.z;
    return os;
  };
};

struct Velocity {
  Linear linear;
  Angular angular;

  Velocity(){};

  Velocity(const Pose &previous_pose, const Pose &current_pose,
           double time_diff_sec) {
    if (time_diff_sec == 0) {
      clear();
      return;
    }

    const Pose diff_pose = current_pose - previous_pose;

    linear.x = diff_pose.x / time_diff_sec;
    linear.y = diff_pose.y / time_diff_sec;
    linear.z = diff_pose.z / time_diff_sec;
    angular.x = diff_pose.roll / time_diff_sec;
    angular.y = diff_pose.pitch / time_diff_sec;
    angular.z = diff_pose.yaw / time_diff_sec;
  };

  Velocity(const PoseStamped &previous_pose, const PoseStamped &current_pose) {
    const double time_diff_sec = current_pose.stamp - previous_pose.stamp;

    if (time_diff_sec == 0) {
      clear();
      return;
    }

    const Pose diff_pose = current_pose.pose - previous_pose.pose;

    linear.x = diff_pose.x / time_diff_sec;
    linear.y = diff_pose.y / time_diff_sec;
    linear.z = diff_pose.z / time_diff_sec;
    angular.x = diff_pose.roll / time_diff_sec;
    angular.y = diff_pose.pitch / time_diff_sec;
    angular.z = diff_pose.yaw / time_diff_sec;
  };

  void clear() {
    linear.clear();
    angular.clear();
  };

  Velocity operator+(const Velocity &rhs_v) const {
    Velocity tmp_v;
    tmp_v.linear = linear + rhs_v.linear;
    tmp_v.angular = angular + rhs_v.angular;
    return tmp_v;
  }

  Velocity operator-(const Velocity &rhs_v) const {
    Velocity tmp_v;
    tmp_v.linear = linear - rhs_v.linear;
    tmp_v.angular = angular - rhs_v.angular;
    return tmp_v;
  }

  Velocity operator*(const double val) const {
    Velocity tmp_v;
    tmp_v.linear = linear * val;
    tmp_v.angular = angular * val;
    return tmp_v;
  }

  Velocity operator/(const double val) const {
    Velocity tmp_v;
    tmp_v.linear = linear / val;
    tmp_v.angular = angular / val;
    return tmp_v;
  }

  friend std::ostream &operator<<(std::ostream &os, const Velocity &velocity) {
    os << "linear:" << velocity.linear << " angular:" << velocity.angular;
    return os;
  };
};

struct Accel {

  Linear linear;
  Angular angular;

  Accel(){};

  Accel(const Velocity &previous_velocity, const Velocity &current_velocity,
        double time_diff_sec) {
    if (time_diff_sec == 0) {
      clear();
      return;
    }

    const Velocity diff_velocity = current_velocity - previous_velocity;
    linear = diff_velocity.linear / time_diff_sec;
    angular = diff_velocity.angular / time_diff_sec;
  };

  void clear() {
    linear.clear();
    angular.clear();
  };

  Accel operator+(const Accel &rhs_a) const {
    Accel tmp_a;
    tmp_a.linear = linear + rhs_a.linear;
    tmp_a.angular = angular + rhs_a.angular;
    return tmp_a;
  }

  Accel operator-(const Accel &rhs_a) const {
    Accel tmp_a;
    tmp_a.linear = linear - rhs_a.linear;
    tmp_a.angular = angular - rhs_a.angular;
    return tmp_a;
  }

  Accel operator*(const double val) const {
    Accel tmp_a;
    tmp_a.linear = linear * val;
    tmp_a.angular = angular * val;
    return tmp_a;
  }

  Accel operator/(const double val) const {
    Accel tmp_a;
    tmp_a.linear = linear / val;
    tmp_a.angular = angular / val;
    return tmp_a;
  }
};

template<class PointType>
struct PointWithDistance
{
    PointWithDistance()
        : distance(0.0)
    {
    };

    PointType point;
    double distance;
};

template struct PointWithDistance<pcl::PointXYZ>;
template struct PointWithDistance<pcl::PointXYZI>;
template struct PointWithDistance<pcl::PointXYZRGB>;

struct HistogramWithRangeBin
{
    HistogramWithRangeBin()
        : min_value(0.0)
        , max_value(0.0)
        , count(0)
    {
    };

    double min_value;
    double max_value;
    size_t count;
};

#endif
