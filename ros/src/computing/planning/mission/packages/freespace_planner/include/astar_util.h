#ifndef ASTAR_UTIL_H
#define ASTAR_UTIL_H

#include <tf/transform_listener.h>

enum class STATUS : uint8_t
{
  NONE,
  OPEN,
  CLOSED,
  OBS
};

struct AstarNode
{
  double x, y, theta;                // Coordinate of each node
  STATUS status     = STATUS::NONE;  // NONE, OPEN, CLOSED or OBS
  double gc         = 0;             // Actual cost
  double hc         = 0;             // heuristic cost
  bool back;                         // true if the current direction of the vehicle is back
  uint8_t steering;                  // steering action of this node
  AstarNode *parent = NULL;          // parent node
};

struct WaveFrontNode
{
  int index_x;
  int index_y;
  double hc;

  WaveFrontNode();
  WaveFrontNode(int x, int y, double cost);
};

struct NodeUpdate
{
  double shift_x;
  double shift_y;
  double rotation;
  double step;
  int index_theta;
  bool curve;
  bool back;
};

// For open list and goal list
struct SimpleNode
{
  int index_x;
  int index_y;
  int index_theta;
  double cost;

  bool operator>(const SimpleNode& right) const {
    return cost > right.cost;
  }

  SimpleNode();
  SimpleNode(int x, int y, int theta, double gc, double hc);
};


namespace astar
{

inline double calcDistance(double x1, double y1, double x2, double y2)
{
  return std::hypot(x2 - x1, y2 - y1);
}

inline double modifyTheta(double theta)
{
  if (theta < 0)
    return theta + 2 * M_PI;
  if (theta >= 2 * M_PI)
    return theta - 2 * M_PI;

  return theta;
}

inline geometry_msgs::Pose transformPose(geometry_msgs::Pose &pose, tf::Transform &tf)
{
  // Convert ROS pose to TF pose
  tf::Pose tf_pose;
  tf::poseMsgToTF(pose, tf_pose);

  // Transform pose
  tf_pose = tf * tf_pose;

  // Convert TF pose to ROS pose
  geometry_msgs::Pose ros_pose;
  tf::poseTFToMsg(tf_pose, ros_pose);

  return ros_pose;
}

inline WaveFrontNode getWaveFrontNode(int x, int y, double cost)
{
  WaveFrontNode node(x, y, cost);

  return node;
}

inline geometry_msgs::Point calcRelativeCoordinate(geometry_msgs::Pose pose, tf::Point point)
{
  tf::Transform transform;
  tf::poseMsgToTF(pose, transform);
  transform = transform.inverse();

  point = transform * point;
  geometry_msgs::Point point_msg;
  tf::pointTFToMsg(point, point_msg);

  return point_msg;
}

inline double calcDiffOfRadian(double a, double b)
{
  double diff = std::fabs(a - b);
  if (diff < M_PI)
    return diff;
  else
    return 2 * M_PI - diff;
}

} // namespace astar

#endif
