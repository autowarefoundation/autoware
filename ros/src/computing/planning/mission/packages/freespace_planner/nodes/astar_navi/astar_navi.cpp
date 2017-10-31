#include "astar_search.h"
#include "search_info_ros.h"
#include "autoware_msgs/LaneArray.h"

namespace
{

void publishPathAsWaypoints(const ros::Publisher& pub, const nav_msgs::Path& path, const double waypoint_velocity_kmph)
{
  autoware_msgs::lane lane;

  lane.header = path.header;
  lane.increment = 0;
  for (const auto& pose : path.poses) {
    autoware_msgs::waypoint wp;
    wp.pose = pose;
    wp.twist.twist.linear.x = waypoint_velocity_kmph / 3.6;

    lane.waypoints.push_back(wp);
  }

  autoware_msgs::LaneArray lane_array;
  lane_array.lanes.push_back(lane);
  pub.publish(lane_array);

  return;
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "astar_navi");
  ros::NodeHandle n;
  ros::NodeHandle private_nh_("~");

  double waypoint_velocity_kmph;
  std::string map_topic;
  private_nh_.param<double>("waypoint_velocity_kmph", waypoint_velocity_kmph, 5.0);
  private_nh_.param<std::string>("map_topic", map_topic, "ring_ogm");

  AstarSearch astar;
  SearchInfo search_info;

  // ROS subscribers
  ros::Subscriber map_sub = n.subscribe(map_topic, 1, &SearchInfo::mapCallback, &search_info);
  ros::Subscriber start_sub = n.subscribe("/current_pose", 1, &SearchInfo::currentPoseCallback, &search_info);
  ros::Subscriber goal_sub  = n.subscribe("/move_base_simple/goal", 1, &SearchInfo::goalCallback, &search_info);

  // ROS publishers
  ros::Publisher path_pub       = n.advertise<nav_msgs::Path>("astar_path", 1, true);
  ros::Publisher waypoints_pub  = n.advertise<autoware_msgs::LaneArray>("lane_waypoints_array", 1, true);
  ros::Publisher debug_pose_pub = n.advertise<geometry_msgs::PoseArray>("debug_pose_array", 1, true);

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    ros::spinOnce();

    if (!search_info.getMapSet() || !search_info.getStartSet() || !search_info.getGoalSet()) {
      loop_rate.sleep();
      continue;
    }

    // Reset flag
    search_info.reset();

    auto start = std::chrono::system_clock::now();

    // Execute astar search
    bool result = astar.makePlan(search_info.getStartPose().pose, search_info.getGoalPose().pose, search_info.getMap());

    auto end = std::chrono::system_clock::now();
    auto usec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    //std::cout << "astar msec: " << usec / 1000.0 << std::endl;
    ROS_INFO("astar msec: %lf", usec / 1000.0);

    if(result) {
      ROS_INFO("Found GOAL!");
      publishPathAsWaypoints(waypoints_pub, astar.getPath(), waypoint_velocity_kmph);

#if DEBUG
      astar.publishPoseArray(debug_pose_pub, "/map");
      path_pub.publish(astar.getPath());
      astar.broadcastPathTF();
#endif

    } else {
      ROS_INFO("can't find goal...");

#if DEBUG
      astar.publishPoseArray(debug_pose_pub, "/map"); // debug
      path_pub.publish(astar.getPath());
#endif

    }

    astar.reset();

    loop_rate.sleep();
  }

  return 0;
}
