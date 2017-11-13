#include <stdio.h>

#include <geometry_msgs/PoseStamped.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <autoware_msgs/lane.h>
#include <autoware_msgs/traffic_light.h>

#include <cross_road_area.hpp>
#include <decision_maker_node.hpp>
#include <euclidean_space.hpp>
#include <state.hpp>
#include <state_context.hpp>

namespace decision_maker
{
bool DecisionMakerNode::isLocalizationConvergence(double _x, double _y, double _z, double _roll, double _pitch,
                                                  double _yaw)
{
  static int _init_count = 0;
  static euclidean_space::point *a = new euclidean_space::point();
  static euclidean_space::point *b = new euclidean_space::point();

  static std::vector<double> distances;
  static int distances_count = 0;
  double avg_distances = 0.0;

  a->x = b->x;
  a->y = b->y;
  a->z = b->z;

  b->x = _x;
  b->y = _y;
  b->z = _z;

  distances.push_back(euclidean_space::EuclideanSpace::find_distance(a, b));
  if (++distances_count > param_convergence_count_)
  {
    distances.erase(distances.begin());
    distances_count--;
    avg_distances = std::accumulate(distances.begin(), distances.end(), 0) / distances.size();
    if (avg_distances <= param_convergence_threshold_)
      return ctx->setCurrentState(state_machine::DRIVE_STATE);
  }
  else
  {
    return false;
  }
}
}
