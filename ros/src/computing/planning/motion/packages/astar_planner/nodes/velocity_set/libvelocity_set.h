#ifndef _VELOCITY_SET_H
#define _VELOCITY_SET_H

#include <math.h>
#include <iostream>
#include <map>
#include <unordered_map>
#include <vector>

#include <boost/circular_buffer.hpp>

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <vector_map/vector_map.h>

#include "waypoint_follower/libwaypoint_follower.h"

enum class EControl
{
  KEEP = -1,
  STOP = 1,
  STOPLINE = 2,
  DECELERATE = 3,
  OTHERS = 4,
};

enum class EObstacleType
{
  NONE = -1,
  ON_WAYPOINTS = 1,
  ON_CROSSWALK = 2,
  STOPLINE = 3,
};

struct CrossWalkPoints
{
  std::vector<geometry_msgs::Point> points;
  geometry_msgs::Point center;
  double width;
};

class CrossWalk
{
private:
  // detection_points_[bdID] has information of each crosswalk
  std::unordered_map<int, CrossWalkPoints> detection_points_;
  int detection_waypoint_;
  int detection_crosswalk_id_;
  std::vector<geometry_msgs::Point> obstacle_points_;
  std::vector<int> bdID_;

  bool enable_multiple_crosswalk_detection_;
  std::vector<int> detection_crosswalk_array_;

public:
  bool loaded_crosswalk;
  bool loaded_area;
  bool loaded_line;
  bool loaded_point;
  bool loaded_all;
  bool set_points;

  vector_map::CrossWalkArray crosswalk_;
  vector_map::AreaArray area_;
  vector_map::LineArray line_;
  vector_map::PointArray point_;

  void crossWalkCallback(const vector_map::CrossWalkArray &msg);
  void areaCallback(const vector_map::AreaArray &msg);
  void lineCallback(const vector_map::LineArray &msg);
  void pointCallback(const vector_map::PointArray &msg);

  int countAreaSize() const;
  void getAID(std::unordered_map<int, std::vector<int>> &aid_crosswalk) const;
  void calcDetectionArea(const std::unordered_map<int, std::vector<int>> &bdid2aid_map);
  geometry_msgs::Point calcCenterofGravity(const int &aid) const;
  double calcCrossWalkWidth(const int &aid) const;
  geometry_msgs::Point getPoint(const int &pid) const;
  void calcCenterPoints();
  void setCrossWalkPoints();
  int findClosestCrosswalk(const int closest_waypoint, const autoware_msgs::lane &lane, const int search_distance);
  int getSize() const
  {
    return detection_points_.size();
  }
  std::vector<int> getBDID() const
  {
    return bdID_;
  }
  CrossWalkPoints getDetectionPoints(const int &id) const
  {
    return detection_points_.at(id);
  }
  void setDetectionWaypoint(const int &num)
  {
    detection_waypoint_ = num;
  }
  int getDetectionWaypoint() const
  {
    return detection_waypoint_;
  }
  void setDetectionCrossWalkID(const int &id)
  {
    detection_crosswalk_id_ = id;
  }
  int getDetectionCrossWalkID() const
  {
    return detection_crosswalk_id_;
  }

  void initDetectionCrossWalkIDs()
  {
    return detection_crosswalk_array_.clear();
  }
  void addDetectionCrossWalkIDs(const int &id)
  {
    auto itr = std::find(detection_crosswalk_array_.begin(), detection_crosswalk_array_.end(), id);
    if (detection_crosswalk_array_.empty() || itr == detection_crosswalk_array_.end())
    {
      detection_crosswalk_array_.push_back(id);
    }
  }
  std::vector<int> getDetectionCrossWalkIDs() const
  {
    return detection_crosswalk_array_;
  }
  void setMultipleDetectionFlag(const bool _multiple_flag)
  {
    enable_multiple_crosswalk_detection_ = _multiple_flag;
  }
  bool isMultipleDetection() const
  {
    return enable_multiple_crosswalk_detection_;
  }

  CrossWalk()
    : detection_waypoint_(-1)
    , detection_crosswalk_id_(-1)
    , loaded_crosswalk(false)
    , loaded_area(false)
    , loaded_line(false)
    , loaded_point(false)
    , loaded_all(false)
    , set_points(false)
  {
  }
};

//////////////////////////////////////
// for visualization of obstacles
//////////////////////////////////////
class ObstaclePoints
{
private:
  std::vector<geometry_msgs::Point> stop_points_;
  std::vector<geometry_msgs::Point> decelerate_points_;
  geometry_msgs::Point previous_detection_;

public:
  void setStopPoint(const geometry_msgs::Point &p)
  {
    stop_points_.push_back(p);
  }
  void setDeceleratePoint(const geometry_msgs::Point &p)
  {
    decelerate_points_.push_back(p);
  }

  geometry_msgs::Point getObstaclePoint(const EControl &kind) const;
  geometry_msgs::Point getNearestObstaclePoint(const geometry_msgs::Point &current_position) const;

  void clearStopPoints()
  {
    stop_points_.clear();
  }
  void clearDeceleratePoints()
  {
    decelerate_points_.clear();
  }

  ObstaclePoints() : stop_points_(0), decelerate_points_(0)
  {
  }
};

class ObstacleTracker
{
private:
  enum class ETrackingState
  {
    INITIALIZE = 1,
    TRACKING = 2,
    LOST = 3,
  };

  class KalmanFilter
  {
  private:
    double x_, p_, k_, Q_, R_;
  public:
    KalmanFilter(double Q = 1e-5, double R = 1e-4)
    : x_(1e-0), p_(1e-1), k_(1e-1)
    { Q_ = Q; R_ = R; }
    void init(double x0) { x_ = x0; }
    void predict() {
      x_ = x_;
      p_ = p_ + Q_;
    }
    double update(const double z)
    {
      k_ = p_ / (p_ + R_);
      x_ = x_ + k_ * (z - x_);
      p_ = (1.0 - k_) * p_;
      return x_;
    }
  };

  bool use_tracking_;
  int frame_thres_;
  double moving_thres_;

  int tracking_counter_, lost_counter_;
  int waypoint_;
  double velocity_;
  tf::Vector3 position_;
  ETrackingState state_;
  ros::Time time_;

  boost::circular_buffer<tf::Vector3> position_buf_;
  boost::circular_buffer<ros::Time> time_buf_;

  KalmanFilter kf_;

  double calcVelocity()
  {
    tf::Vector3 dx = position_buf_[1]-position_buf_[0];
    ros::Duration dt = time_buf_[1]-time_buf_[0];
    kf_.predict();
    double v = kf_.update(dx.length()/dt.toSec());
    v = (v > moving_thres_) ? v : 0.0;
    return v;
  }

public:
  ObstacleTracker(const bool& use_tracking, double moving_thres)
  {
    position_buf_.set_capacity(2);
    time_buf_.set_capacity(2);
    use_tracking_ = use_tracking;
    moving_thres_ = moving_thres;
    reset();
  }

  void update(const int& stop_waypoint, const ObstaclePoints* obstacle_points, const double& waypoint_velocity, const geometry_msgs::Point current_position)
  {

    if (!use_tracking_)
    {
      waypoint_ = stop_waypoint;
      velocity_ = 0.0;
      return;
    }

    lost_counter_ = 0;
    tracking_counter_++;
    time_ = ros::Time::now();
    // tf::pointMsgToTF(obstacle_points->getObstaclePoint(EControl::STOP), position_);
    tf::pointMsgToTF(obstacle_points->getNearestObstaclePoint(current_position), position_);
    position_buf_.push_back(position_);
    time_buf_.push_back(time_);

    if (state_ == ETrackingState::INITIALIZE)
    {
      kf_.init(waypoint_velocity);
      waypoint_ = stop_waypoint;
      velocity_ = waypoint_velocity;
      if (tracking_counter_ >= 2)
        state_ = ETrackingState::TRACKING;
    }
    else if (state_ == ETrackingState::TRACKING)
    {
      waypoint_ = stop_waypoint;
      velocity_ = calcVelocity();
    }
  }

  void update()
  {
    lost_counter_++;
    kf_.predict();
    if (lost_counter_ >= 5)
    {
      lost_counter_ = 0;
      reset();
    }
  }

  void reset()
  {
    state_ = ETrackingState::INITIALIZE;
    tracking_counter_ = 0;
    lost_counter_ = 0;
    waypoint_ = -1;
    velocity_ = 0.0;
    position_ = tf::Vector3(0.0, 0.0, 0.0);
    position_buf_.clear();
    time_buf_.clear();
    kf_.init(0.0);
  }

  int getWaypointIdx()
  {
    return waypoint_;
  }

  double getVelocity()
  {
    return velocity_;
  }
};

inline double calcSquareOfLength(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2)
{
  return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
}

// Calculate waypoint index corresponding to distance from begin_waypoint
inline int calcWaypointIndexReverse(const autoware_msgs::lane &lane, const int begin_waypoint, const double distance)
{
  double dist_sum = 0;
  for (int i = begin_waypoint; i > 0; i--)
  {
    tf::Vector3 v1(lane.waypoints[i].pose.pose.position.x, lane.waypoints[i].pose.pose.position.y, 0);

    tf::Vector3 v2(lane.waypoints[i - 1].pose.pose.position.x, lane.waypoints[i - 1].pose.pose.position.y, 0);

    dist_sum += tf::tfDistance(v1, v2);

    if (dist_sum > distance)
      return i;
  }

  // reach the first waypoint
  return 0;
}

#endif /* _VELOCITY_SET_H */
