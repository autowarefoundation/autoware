#ifndef _VELOCITY_SET_H
#define _VELOCITY_SET_H

#include <iostream>
#include <vector>
#include <map>
#include <unordered_map>
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <vector_map/vector_map.h>

enum EControl
{
  KEEP = -1,
  STOP = 1,
  DECELERATE = 2,
  OTHERS = 3,
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
  bool decided_;

public:
  void setStopPoint(const geometry_msgs::Point &p)
  {
    stop_points_.push_back(p);
  }
  void setDeceleratePoint(const geometry_msgs::Point &p)
  {
    decelerate_points_.push_back(p);
  }
  void setDecided(const bool &b)
  {
    decided_ = b;
  }
  geometry_msgs::Point getObstaclePoint(const EControl &kind);
  void clearStopPoints()
  {
    stop_points_.clear();
  }
  void clearDeceleratePoints()
  {
    decelerate_points_.clear();
  }
  bool isDecided()
  {
    return decided_;
  }
  geometry_msgs::Point getPreviousDetection()
  {
    return previous_detection_;
  }

  ObstaclePoints() : stop_points_(0), decelerate_points_(0), decided_(false)
  {
  }
};

inline double calcSquareOfLength(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2)
{
  return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
}

#endif /* _VELOCITY_SET_H */
