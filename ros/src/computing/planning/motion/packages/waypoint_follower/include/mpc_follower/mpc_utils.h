#pragma once
#include <cmath>
#include <vector>
#include <eigen3/Eigen/Core>

#include "autoware_msgs/ControlCommandStamped.h"
#include "autoware_msgs/Lane.h"
#include <autoware_msgs/VehicleStatus.h>
#include <tf2/utils.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include "mpc_follower/mpc_trajectory.h"

class MPCUtils
{
  public:
    MPCUtils(){};
    ~MPCUtils(){};

    // set into [-pi to pi]
    double intoSemicircle(const double a);

    void fillIncrease(std::vector<double>::iterator first, std::vector<double>::iterator last, double init, double diff);

    void convertEulerAngleToMonotonic(std::vector<double> &a);

    geometry_msgs::Quaternion getQuaternionFromYaw(const double &yaw);

    void fill_increase(std::vector<double>::iterator first, std::vector<double>::iterator last, double init, double diff);
    void filteringMovingAverate(std::vector<double> &u, const int num);

    // 1D interpolation
    bool interp1d(const std::vector<double> &index, const std::vector<double> &values, const double &ref, double &ret);
    bool interp1d(const Eigen::VectorXd &index, const Eigen::VectorXd &values, const double &ref, double &ret);

    void calcTrajectoryCurvature(MPCTrajectory &traj, int curvature_smoothing_num);
    void resamplePathToTrajByDistance(const autoware_msgs::Lane &path, const std::vector<double> &time,
                                      const double &dl, MPCTrajectory &ref_traj);
    void resamplePathToTrajByTime(const autoware_msgs::Lane &path, const std::vector<double> &time,
                                  const double &dt, MPCTrajectory &ref_traj_);

    void calcPathRelativeTime(const autoware_msgs::Lane &path, std::vector<double> &path_time);
    void calcNearestPose(const MPCTrajectory &traj, const geometry_msgs::Pose &self_pose, geometry_msgs::Pose &nearest_pose,
                         unsigned int &nearest_index, double &min_dist_error, double &nearest_yaw_error, double &nearest_time);
    void calcNearestPoseInterp(const MPCTrajectory &traj, const geometry_msgs::Pose &self_pose, geometry_msgs::Pose &nearest_pose,
                               unsigned int &nearest_index, double &min_dist_error, double &nearest_yaw_error, double &nearest_time);
};