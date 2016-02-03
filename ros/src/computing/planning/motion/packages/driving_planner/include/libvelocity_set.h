#ifndef _VELOCITY_SET_H
#define _VELOCITY_SET_H

#include <iostream>
#include <vector>
#include <map>
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <map_file/CrossWalkArray.h>
#include <map_file/AreaClassArray.h>
#include <map_file/LineClassArray.h>
#include <map_file/PointClassArray.h>

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
  // detection_points_[bdID] has points at each crosswalk
  // detection_points_[0] has no data
  std::vector<CrossWalkPoints> detection_points_;
  int detection_waypoint_;
  int detection_crosswalk_id_;
  std::vector<geometry_msgs::Point> obstacle_points_;

 public:
  bool loaded_crosswalk;
  bool loaded_areaclass;
  bool loaded_lineclass;
  bool loaded_pointclass;
  bool loaded_all;
  bool set_points;
  map_file::CrossWalkArray crosswalk_;
  map_file::AreaClassArray areaclass_;
  map_file::LineClassArray lineclass_;
  map_file::PointClassArray pointclass_;

  void CrossWalkCallback(const map_file::CrossWalkArray &msg);
  void AreaclassCallback(const map_file::AreaClassArray &msg);
  void LineclassCallback(const map_file::LineClassArray &msg);
  void PointclassCallback(const map_file::PointClassArray &msg);

  int countAreaSize() const;
  void getAID(std::vector< std::vector<int> > &aid_crosswalk) const;
  void calcDetectionArea(const std::vector< std::vector<int> > &aid_crosswalk);
  geometry_msgs::Point calcCenterofGravity(const int &aid) const;
  double calcCrossWalkWidth(const int &aid) const;
  geometry_msgs::Point getPoint(const int &pid) const;
  void calcCenterPoints();
  void setCrossWalkPoints();
  int getSize() const { return detection_points_.size(); }
  CrossWalkPoints getDetectionPoints(const int &id) const { return detection_points_[id]; }
  void setDetectionWaypoint(const int &num) { detection_waypoint_ = num; }
  int getDetectionWaypoint() const { return detection_waypoint_; }
  void setDetectionCrossWalkID(const int &id) { detection_crosswalk_id_ = id; }
  int getDetectionCrossWalkID() const { return detection_crosswalk_id_; }

  CrossWalk () :
    detection_waypoint_(-1),
    detection_crosswalk_id_(-1),
    loaded_crosswalk(false),
    loaded_areaclass(false),
    loaded_lineclass(false),
    loaded_pointclass(false),
    loaded_all(false),
    set_points(false) {}
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
  void setStopPoint(const geometry_msgs::Point &p) { stop_points_.push_back(p); }
  void setDeceleratePoint(const geometry_msgs::Point &p) { decelerate_points_.push_back(p); }
  void setDecided(const bool &b) { decided_ = b; }
  geometry_msgs::Point getObstaclePoint(const EControl &kind);
  void clearStopPoints() { stop_points_.clear(); }
  void clearDeceleratePoints() { decelerate_points_.clear(); }
  bool isDecided() { return decided_; }
  geometry_msgs::Point getPreviousDetection() { return previous_detection_; }

  ObstaclePoints () :
    stop_points_(0),
    decelerate_points_(0),
    decided_(false) {}
};


inline double CalcSquareOfLength(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2)
{
  return (p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z);
}


#endif /* _VELOCITY_SET_H */
