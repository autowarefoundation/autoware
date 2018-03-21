/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float32.h>
#include <iostream>

#include "libvelocity_set.h"
#include "velocity_set_info.h"
#include "velocity_set_path.h"

namespace
{
constexpr int LOOP_RATE = 10;
constexpr double DECELERATION_SEARCH_DISTANCE = 30;
constexpr double STOP_SEARCH_DISTANCE = 60;

void obstacleColorByKind(const EObstacleType type, std_msgs::ColorRGBA &color, const double alpha=0.5)
{
  if (type == EObstacleType::ON_WAYPOINTS || type == EObstacleType::ON_CROSSWALK)
  {
    color.r = 1.0; color.g = 0.0; color.b = 0.0; color.a = alpha;  // red
  }
  else if (type == EObstacleType::STOPLINE)
  {
    color.r = 0.0; color.g = 0.0; color.b = 1.0; color.a = alpha;  // blue
  }
  else if (type == EObstacleType::ON_DECELERATE)
  {
    color.r = 1.0; color.g = 1.0; color.b = 0.0; color.a = alpha;  // yellow
  }
  else
  {
    color.r = 1.0; color.g = 1.0; color.b = 1.0; color.a = alpha;  // white
  }
}

// Display a detected obstacle
void displayObstacles(EControl& kind, const std::vector<ObstacleInfo>& obstacle_infos, const ros::Publisher& obstacles_pub)
{
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time();
  marker.ns = "obstacle";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  for (auto& info : obstacle_infos)
  {
    marker.pose.position = info.points.getObstaclePoint(kind);
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 2.0;
    marker.lifetime = ros::Duration(0.1);
    marker.frame_locked = true;
    obstacleColorByKind(info.type, marker.color, 0.7);
    marker_array.markers.push_back(marker);
  }

  obstacles_pub.publish(marker_array);
  marker_array.markers.clear();
}

void displayDetectionRange(const autoware_msgs::lane& lane, const CrossWalk& crosswalk,
                           const int closest_waypoint, const std::vector<ObstacleInfo>& obstacle_infos,
                           const double stop_range, const double deceleration_range,
                           const ros::Publisher& detection_range_pub)
{
  // set up for marker array
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker crosswalk_marker;
  visualization_msgs::Marker waypoint_marker_stop;
  visualization_msgs::Marker waypoint_marker_decelerate;
  visualization_msgs::Marker stop_line;
  visualization_msgs::Marker stop_velocity;
  crosswalk_marker.header.frame_id = "/map";
  crosswalk_marker.header.stamp = ros::Time();
  crosswalk_marker.id = 0;
  crosswalk_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  crosswalk_marker.action = visualization_msgs::Marker::ADD;
  waypoint_marker_stop = crosswalk_marker;
  waypoint_marker_decelerate = crosswalk_marker;
  stop_line = crosswalk_marker;
  stop_line.type = visualization_msgs::Marker::CUBE;
  stop_velocity = crosswalk_marker;
  stop_velocity.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

  // set each namespace
  crosswalk_marker.ns = "Crosswalk Detection";
  waypoint_marker_stop.ns = "Stop Detection";
  waypoint_marker_decelerate.ns = "Decelerate Detection";
  stop_line.ns = "Stop Line";
  stop_velocity.ns = "Stop Velocity";

  // set scale and color
  double scale = 2 * stop_range;
  waypoint_marker_stop.scale.x = scale;
  waypoint_marker_stop.scale.y = scale;
  waypoint_marker_stop.scale.z = scale;
  waypoint_marker_stop.color.a = 0.2;
  waypoint_marker_stop.color.r = 0.0;
  waypoint_marker_stop.color.g = 1.0;
  waypoint_marker_stop.color.b = 0.0;
  waypoint_marker_stop.frame_locked = true;

  scale = 2 * (stop_range + deceleration_range);
  waypoint_marker_decelerate.scale.x = scale;
  waypoint_marker_decelerate.scale.y = scale;
  waypoint_marker_decelerate.scale.z = scale;
  waypoint_marker_decelerate.color.a = 0.15;
  waypoint_marker_decelerate.color.r = 1.0;
  waypoint_marker_decelerate.color.g = 1.0;
  waypoint_marker_decelerate.color.b = 0.0;
  waypoint_marker_decelerate.frame_locked = true;

  bool found_obstacle = false, found_stopline = false;
  for (auto& info : obstacle_infos)
  {
    // use nearest obstacle and stopline
    if ((info.type == EObstacleType::ON_WAYPOINTS ||
         info.type == EObstacleType::ON_CROSSWALK))
    {
      if (!found_obstacle) found_obstacle = true;
      else continue;
    }
    if (info.type == EObstacleType::STOPLINE)
    {
      if (!found_stopline) found_stopline = true;
      else continue;
    }

    // geo-fence
    stop_line.pose.position = lane.waypoints[info.waypoint].pose.pose.position;
    stop_line.pose.orientation = lane.waypoints[info.waypoint].pose.pose.orientation;

    // hack: normalizing quaternion for fixing rviz error
    // "Marker 'Stop Line/0' contains unnormalized quaternions."
    tf::Quaternion quat;
    tf::quaternionMsgToTF(stop_line.pose.orientation, quat);  // normalized same time
    tf::quaternionTFToMsg(quat, stop_line.pose.orientation);

    stop_line.pose.position.z += 1.0;
    stop_line.scale.x = 0.1;
    stop_line.scale.y = 15.0;
    stop_line.scale.z = 2.0;
    stop_line.lifetime = ros::Duration(0.1);
    stop_line.frame_locked = true;
    obstacleColorByKind(info.type, stop_line.color, 0.3);
    marker_array.markers.push_back(stop_line);

    // velocity [kmph]
    stop_velocity.id = stop_line.id;
    stop_velocity.pose = stop_line.pose;
    stop_velocity.pose.position.z += 1.5;
    stop_velocity.scale.z = 1.0;
    stop_velocity.color = stop_line.color;
    stop_velocity.lifetime = ros::Duration(0.1);
    stop_velocity.frame_locked = true;
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1) << 3.6*info.velocity << " [km/h]";
    stop_velocity.text = oss.str();
    marker_array.markers.push_back(stop_velocity);

    stop_line.id++;
  }

  int crosswalk_id = crosswalk.getDetectionCrossWalkID();
  if (crosswalk_id > 0)
    scale = crosswalk.getDetectionPoints(crosswalk_id).width;
  crosswalk_marker.scale.x = scale;
  crosswalk_marker.scale.y = scale;
  crosswalk_marker.scale.z = scale;
  crosswalk_marker.color.a = 0.5;
  crosswalk_marker.color.r = 0.0;
  crosswalk_marker.color.g = 1.0;
  crosswalk_marker.color.b = 0.0;
  crosswalk_marker.frame_locked = true;

  // set marker points coordinate
  for (int i = 0; i < STOP_SEARCH_DISTANCE; i++)
  {
    if (closest_waypoint < 0 || i + closest_waypoint > static_cast<int>(lane.waypoints.size()) - 1)
      break;

    geometry_msgs::Point point;
    point = lane.waypoints[closest_waypoint + i].pose.pose.position;

    waypoint_marker_stop.points.push_back(point);

    if (i > DECELERATION_SEARCH_DISTANCE)
      continue;
    waypoint_marker_decelerate.points.push_back(point);
  }

  if (crosswalk_id > 0)
  {
    if (!crosswalk.isMultipleDetection())
    {
      for (const auto& p : crosswalk.getDetectionPoints(crosswalk_id).points)
        crosswalk_marker.points.push_back(p);
    }
    else
    {
      for (const auto& c_id : crosswalk.getDetectionCrossWalkIDs())
      {
        for (const auto& p : crosswalk.getDetectionPoints(c_id).points)
        {
          scale = crosswalk.getDetectionPoints(c_id).width;
          crosswalk_marker.points.push_back(p);
        }
      }
    }
  }

  // publish marker
  marker_array.markers.push_back(crosswalk_marker);
  marker_array.markers.push_back(waypoint_marker_stop);
  marker_array.markers.push_back(waypoint_marker_decelerate);
  detection_range_pub.publish(marker_array);
  marker_array.markers.clear();
}

// obstacle detection for crosswalk
EControl crossWalkDetection(const pcl::PointCloud<pcl::PointXYZ>& points, const CrossWalk& crosswalk,
                            const geometry_msgs::PoseStamped& localizer_pose, const int points_threshold,
                            ObstaclePoints* obstacle_points)
{
  int crosswalk_id = crosswalk.getDetectionCrossWalkID();
  double search_radius = crosswalk.getDetectionPoints(crosswalk_id).width / 2;
  // std::vector<int> crosswalk_ids crosswalk.getDetectionCrossWalkIDs();

  // Search each calculated points in the crosswalk
  for (const auto& c_id : crosswalk.getDetectionCrossWalkIDs())
  {
    for (const auto& p : crosswalk.getDetectionPoints(c_id).points)
    {
      geometry_msgs::Point detection_point = calcRelativeCoordinate(p, localizer_pose.pose);
      tf::Vector3 detection_vector = point2vector(detection_point);
      detection_vector.setZ(0.0);

      int stop_count = 0;  // the number of points in the detection area
      for (const auto& p : points)
      {
        tf::Vector3 point_vector(p.x, p.y, 0.0);
        double distance = tf::tfDistance(point_vector, detection_vector);
        if (distance < search_radius)
        {
          stop_count++;
          geometry_msgs::Point point_temp;
          point_temp.x = p.x;
          point_temp.y = p.y;
          point_temp.z = p.z;
          obstacle_points->setStopPoint(calcAbsoluteCoordinate(point_temp, localizer_pose.pose));
        }
        if (stop_count > points_threshold)
          return EControl::STOP;
      }
    }

    obstacle_points->clearStopPoints();
    if (!crosswalk.isMultipleDetection())
      break;
  }
  return EControl::KEEP;  // find no obstacles
}

void detectStopObstacle(const pcl::PointCloud<pcl::PointXYZ>& points, const int closest_waypoint,
                       const autoware_msgs::lane& lane, const CrossWalk& crosswalk, double stop_range,
                       double points_threshold, const geometry_msgs::PoseStamped& localizer_pose,
                       std::vector<ObstacleInfo>* obstacle_infos, const int wpidx_detection_result_by_other_nodes)
{
  // start search from the closest waypoint
  for (int i = closest_waypoint; i < closest_waypoint + STOP_SEARCH_DISTANCE; i++)
  {
    // reach the end of waypoints
    if (i >= static_cast<int>(lane.waypoints.size()))
      break;

    // detection another nodes
    if (wpidx_detection_result_by_other_nodes >= 0 &&
        lane.waypoints.at(i).gid == wpidx_detection_result_by_other_nodes)
    {
      ObstacleInfo stopline_info(i, 0.0, EObstacleType::STOPLINE);
      stopline_info.points.setStopPoint(lane.waypoints.at(i).pose.pose.position); // for vizuialization
      obstacle_infos->push_back(stopline_info);
    }

    // Detection for cross walk
    if (i == crosswalk.getDetectionWaypoint())
    {
      // found an obstacle in the cross walk
      ObstaclePoints obstacle_points;
      if (crossWalkDetection(points, crosswalk, localizer_pose, points_threshold, &obstacle_points) == EControl::STOP)
      {
        ObstacleInfo crosswalk_obstacle_info(i, 0.0, EObstacleType::ON_CROSSWALK);
        crosswalk_obstacle_info.points = obstacle_points;
        obstacle_infos->push_back(crosswalk_obstacle_info);
      }
    }

    // waypoint seen by localizer
    geometry_msgs::Point waypoint = calcRelativeCoordinate(lane.waypoints[i].pose.pose.position, localizer_pose.pose);
    tf::Vector3 tf_waypoint = point2vector(waypoint);
    tf_waypoint.setZ(0);

    int stop_point_count = 0;
    ObstacleInfo stop_obstacle_info(i, 0.0, EObstacleType::ON_WAYPOINTS);

    for (const auto& p : points)
    {
      tf::Vector3 point_vector(p.x, p.y, 0);

      // 2D distance between waypoint and points (obstacle)
      double dt = tf::tfDistance(point_vector, tf_waypoint);
      if (dt < stop_range)
      {
        stop_point_count++;
        geometry_msgs::Point point_temp;
        point_temp.x = p.x;
        point_temp.y = p.y;
        point_temp.z = p.z;
        stop_obstacle_info.points.setStopPoint(calcAbsoluteCoordinate(point_temp, localizer_pose.pose));
      }
    }

    // there is an obstacle if the number of points exceeded the threshold
    if (stop_point_count > points_threshold)
    {
      obstacle_infos->push_back(stop_obstacle_info);
    }

    // check next waypoint...
  }
}

void detectDecelerateObstacle(const pcl::PointCloud<pcl::PointXYZ>& points, const int closest_waypoint,
                             const autoware_msgs::lane& lane, const double stop_range, const double deceleration_range,
                             const double points_threshold, const geometry_msgs::PoseStamped& localizer_pose,
                             std::vector<ObstacleInfo>* obstacle_infos)
{
  // start search from the closest waypoint
  for (int i = closest_waypoint; i < closest_waypoint + DECELERATION_SEARCH_DISTANCE; i++)
  {
    // reach the end of waypoints
    if (i >= static_cast<int>(lane.waypoints.size()))
      break;

    // waypoint seen by localizer
    geometry_msgs::Point waypoint = calcRelativeCoordinate(lane.waypoints[i].pose.pose.position, localizer_pose.pose);
    tf::Vector3 tf_waypoint = point2vector(waypoint);
    tf_waypoint.setZ(0);

    int decelerate_point_count = 0;
    ObstacleInfo decelerate_obstacle_info(i, 0.0, EObstacleType::ON_DECELERATE);

    for (const auto& p : points)
    {
      tf::Vector3 point_vector(p.x, p.y, 0);

      // 2D distance between waypoint and points (obstacle)
      double dt = tf::tfDistance(point_vector, tf_waypoint);
      if (dt > stop_range && dt < stop_range + deceleration_range)
      {
        decelerate_point_count++;
        geometry_msgs::Point point_temp;
        point_temp.x = p.x;
        point_temp.y = p.y;
        point_temp.z = p.z;
        decelerate_obstacle_info.points.setDeceleratePoint(calcAbsoluteCoordinate(point_temp, localizer_pose.pose));
      }
    }

    // there is an obstacle if the number of points exceeded the threshold
    if (decelerate_point_count > points_threshold)
    {
      obstacle_infos->push_back(decelerate_obstacle_info);
    }

    // check next waypoint...
  }
}

// Detect an obstacle by using pointcloud
EControl pointsDetection(const pcl::PointCloud<pcl::PointXYZ>& points, const int closest_waypoint,
                         const autoware_msgs::lane& lane, const CrossWalk& crosswalk, const VelocitySetInfo& vs_info, std::vector<ObstacleInfo>* obstacle_infos, ObstacleTracker *tracker)
{
  // no input for detection || no closest waypoint
  if ((points.empty() == true && vs_info.getDetectionResultByOtherNodes() == -1) || closest_waypoint < 0)
  {
    // equal NONE
  }
  else
  {
    detectStopObstacle(points, closest_waypoint, lane, crosswalk, vs_info.getStopRange(),
                       vs_info.getPointsThreshold(), vs_info.getLocalizerPose(),
                       obstacle_infos, vs_info.getDetectionResultByOtherNodes());
  }

  // tracking "nearest" vehicle ON_WAYPOINTS
  bool obstacle_is_observed = false;
  for (auto& info : *obstacle_infos)
  {
    if (info.type == EObstacleType::ON_WAYPOINTS)
    {
      double waypoint_velocity = lane.waypoints.at(info.waypoint).twist.twist.linear.x;
      tracker->update(info.waypoint, &info.points, waypoint_velocity, vs_info.getControlPose().pose.position);
      info.waypoint = tracker->getWaypointIdx();
      info.velocity = tracker->getVelocity();
      obstacle_is_observed = true;
      break;
    }
  }

  // obstacle ON_WAYPOINTS is not observed
  if (obstacle_infos->size() == 0 || !obstacle_is_observed)
  {
    tracker->update();
    if (tracker->getWaypointIdx() != -1)  // tracking is continued
    {
      obstacle_infos->push_back(ObstacleInfo(tracker->getWaypointIdx(),
        tracker->getVelocity(), EObstacleType::ON_WAYPOINTS));
    }
  }

  // skip searching deceleration range
  if (vs_info.getDecelerationRange() < 0.01)
  {
    // return EControl based on nearest object
    if (obstacle_infos->size() == 0)
      return EControl::KEEP;
    else if (obstacle_infos->at(0).type == EObstacleType::ON_WAYPOINTS || obstacle_infos->at(0).type == EObstacleType::ON_CROSSWALK)
      return EControl::STOP;
    else if (obstacle_infos->at(0).type == EObstacleType::STOPLINE)
      return EControl::STOPLINE;
    else
      return EControl::OTHERS;
  }

  detectDecelerateObstacle(points, closest_waypoint, lane,
    vs_info.getStopRange(), vs_info.getDecelerationRange(),
    vs_info.getPointsThreshold(), vs_info.getLocalizerPose(), obstacle_infos);

  // search nearest stop/decelerate waypoint
  double stop_obstacle_waypoint = -1;
  double decelerate_obstacle_waypoint = -1;
  for (auto& info : *obstacle_infos)
  {
    if (info.type == EObstacleType::NONE)
      continue;
    else if (info.type == EObstacleType::ON_DECELERATE && decelerate_obstacle_waypoint == -1)
      decelerate_obstacle_waypoint = info.waypoint;
    else if (stop_obstacle_waypoint == -1)
      stop_obstacle_waypoint = info.waypoint;
  }

  // stop obstacle was not found
  if (stop_obstacle_waypoint < 0)
  {
    return (decelerate_obstacle_waypoint < 0) ? EControl::KEEP : EControl::DECELERATE;
  }

  // stop obstacle was found but decelerate obstacle was not found
  if (decelerate_obstacle_waypoint < 0)
  {
    return EControl::STOP;
  }

  // about 5.0 meter
  double waypoint_interval =
      getPlaneDistance(lane.waypoints[0].pose.pose.position, lane.waypoints[1].pose.pose.position);
  int stop_decelerate_threshold = 5 / waypoint_interval;

  // both were found
  if (stop_obstacle_waypoint - decelerate_obstacle_waypoint > stop_decelerate_threshold)
  {
    return EControl::DECELERATE;
  }
  else
  {
    return EControl::STOP;
  }
}

EControl obstacleDetection(int closest_waypoint, const autoware_msgs::lane& lane, const CrossWalk& crosswalk,
                           const VelocitySetInfo vs_info, const ros::Publisher& detection_range_pub,
                           const ros::Publisher& obstacles_pub, std::vector<ObstacleInfo>* obstacle_infos, ObstacleTracker* tracker)
{
  EControl detection_result = pointsDetection(vs_info.getPoints(), closest_waypoint, lane, crosswalk,
                                              vs_info, obstacle_infos, tracker);
  displayDetectionRange(lane, crosswalk, closest_waypoint, *obstacle_infos, vs_info.getStopRange(), vs_info.getDecelerationRange(), detection_range_pub);

  static EControl prev_detection = EControl::KEEP;

  // keep prev detection if EControl::OTHERS
  detection_result = (detection_result == EControl::OTHERS) ? prev_detection : detection_result;

  // stop or decelerate because we found obstacles
  if (detection_result == EControl::STOP || detection_result == EControl::STOPLINE || detection_result == EControl::DECELERATE)
  {
    displayObstacles(detection_result, *obstacle_infos, obstacles_pub);
    prev_detection = detection_result;
    return detection_result;
  }

  // there are no obstacles, so we move forward
  prev_detection = EControl::KEEP;
  return detection_result;
}

void changeWaypoints(const VelocitySetInfo& vs_info, const EControl& detection_result, int closest_waypoint,
                     std::vector<ObstacleInfo> obstacle_infos, const ros::Publisher& final_waypoints_pub,
                     VelocitySetPath* vs_path)
{
  // load global waypoints
  vs_path->initializeNewWaypoints();

  // STOP for obstacle/stopline
  // stop_waypoint is about stop_distance meter away from obstacles/stoplines
  if (detection_result == EControl::STOP || detection_result == EControl::STOPLINE)
  {
    int stop_distance = 0;
    int stop_waypoint = 0;
    double deceleration = 0.0;

    // search nearest stop obstacle and apply speed planning
    for (auto& info : obstacle_infos)
    {
      if (info.type == EObstacleType::ON_WAYPOINTS || info.type == EObstacleType::ON_CROSSWALK)
      {
        // change waypoints to stop by the stop_waypoint
        stop_distance = vs_info.getStopDistanceObstacle();
        deceleration = vs_info.getDecelerationObstacle();
        stop_waypoint = calcWaypointIndexReverse(vs_path->getPrevWaypoints(), info.waypoint, stop_distance);
        vs_path->changeWaypointsForStopping(stop_waypoint, info.waypoint, info.velocity, closest_waypoint, deceleration);
        break;
      }
    }

    autoware_msgs::lane obstacle_waypoints = vs_path->getNewWaypoints();

    // search nearest stopline and apply speed planning
    for (auto& info : obstacle_infos)
    {
      if (info.type == EObstacleType::STOPLINE)
      {
        // change waypoints to stop by the stop_waypoint
        stop_distance = vs_info.getStopDistanceStopline();
        deceleration = vs_info.getDecelerationStopline();
        stop_waypoint = calcWaypointIndexReverse(vs_path->getPrevWaypoints(), info.waypoint, stop_distance);
        vs_path->changeWaypointsForStopping(stop_waypoint, info.waypoint, info.velocity, closest_waypoint, deceleration);
        break;
      }
    }

    autoware_msgs::lane stopline_waypoints = vs_path->getNewWaypoints();

    // v = min(v_obstacle, v_stopline)
    autoware_msgs::lane new_waypoints = obstacle_waypoints;
    for (unsigned int i = 0; i < obstacle_waypoints.waypoints.size(); ++i)
    {
      double velocity_obstacle = obstacle_waypoints.waypoints[i].twist.twist.linear.x;
      double velocity_stopline = stopline_waypoints.waypoints[i].twist.twist.linear.x;
      new_waypoints.waypoints[i].twist.twist.linear.x = std::min(velocity_obstacle, velocity_stopline);
    }

    vs_path->setNewWaypoints(new_waypoints);

    vs_path->avoidSuddenAcceleration(deceleration, closest_waypoint);
    vs_path->avoidSuddenDeceleration(vs_info.getVelocityChangeLimit(), deceleration, closest_waypoint);
    vs_path->setTemporalWaypoints(vs_info.getTemporalWaypointsSize(), closest_waypoint, vs_info.getControlPose());
    final_waypoints_pub.publish(vs_path->getTemporalWaypoints());
  }
  // DECELERATE for obstacles
  else if (detection_result == EControl::DECELERATE)
  {
    // search nearest decelerate obstacle and apply speed planning
    for (auto& info : obstacle_infos)
    {
      if (info.type == EObstacleType::ON_DECELERATE)
      {
        vs_path->changeWaypointsForDeceleration(vs_info.getDecelerationObstacle(), closest_waypoint, info.waypoint);
        break;
      }
    }
    vs_path->avoidSuddenDeceleration(vs_info.getVelocityChangeLimit(), vs_info.getDecelerationObstacle(), closest_waypoint);
    vs_path->avoidSuddenAcceleration(vs_info.getDecelerationObstacle(), closest_waypoint);
    vs_path->setTemporalWaypoints(vs_info.getTemporalWaypointsSize(), closest_waypoint, vs_info.getControlPose());
    final_waypoints_pub.publish(vs_path->getTemporalWaypoints());
  }
  else
  {  // ACCELERATE or KEEP
    vs_path->avoidSuddenAcceleration(vs_info.getDecelerationObstacle(), closest_waypoint);
    vs_path->avoidSuddenDeceleration(vs_info.getVelocityChangeLimit(), vs_info.getDecelerationObstacle(), closest_waypoint);
    vs_path->setTemporalWaypoints(vs_info.getTemporalWaypointsSize(), closest_waypoint, vs_info.getControlPose());
    final_waypoints_pub.publish(vs_path->getTemporalWaypoints());
  }
}

}  // end namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velocity_set");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  bool use_crosswalk_detection;
  bool enable_multiple_crosswalk_detection;
  bool enablePlannerDynamicSwitch;
  bool enable_tracking_on_waypoints;

  std::string points_topic;
  int tracking_moving_thres;

  private_nh.param<bool>("use_crosswalk_detection", use_crosswalk_detection, true);
  private_nh.param<bool>("enable_multiple_crosswalk_detection", enable_multiple_crosswalk_detection, true);
  private_nh.param<bool>("enable_tracking_on_waypoints", enable_tracking_on_waypoints, true);
  private_nh.param<bool>("enablePlannerDynamicSwitch", enablePlannerDynamicSwitch, false);
  private_nh.param<std::string>("points_topic", points_topic, "points_lanes");
  private_nh.param<int>("tracking_moving_thres", tracking_moving_thres, 2.78); // < 10 [km/h]

  // class
  CrossWalk crosswalk;
  VelocitySetPath vs_path;
  VelocitySetInfo vs_info;
  ObstacleTracker tracker(enable_tracking_on_waypoints, tracking_moving_thres);

  // velocity set subscriber
  ros::Subscriber waypoints_sub = nh.subscribe("safety_waypoints", 1, &VelocitySetPath::waypointsCallback, &vs_path);
  ros::Subscriber current_vel_sub =
      nh.subscribe("current_velocity", 1, &VelocitySetPath::currentVelocityCallback, &vs_path);



  // velocity set info subscriber
  ros::Subscriber config_sub = nh.subscribe("config/velocity_set", 1, &VelocitySetInfo::configCallback, &vs_info);
  ros::Subscriber points_sub = nh.subscribe(points_topic, 1, &VelocitySetInfo::pointsCallback, &vs_info);
  ros::Subscriber localizer_sub = nh.subscribe("localizer_pose", 1, &VelocitySetInfo::localizerPoseCallback, &vs_info);
  ros::Subscriber control_pose_sub = nh.subscribe("current_pose", 1, &VelocitySetInfo::controlPoseCallback, &vs_info);
  ros::Subscriber obstacle_sim_points_sub = nh.subscribe("obstacle_sim_pointcloud", 1, &VelocitySetInfo::obstacleSimCallback, &vs_info);
  ros::Subscriber detectionresult_sub = nh.subscribe("/state/stopline_wpidx", 1, &VelocitySetInfo::detectionCallback, &vs_info);

  // vector map subscriber
  ros::Subscriber sub_dtlane = nh.subscribe("vector_map_info/cross_walk", 1, &CrossWalk::crossWalkCallback, &crosswalk);
  ros::Subscriber sub_area = nh.subscribe("vector_map_info/area", 1, &CrossWalk::areaCallback, &crosswalk);
  ros::Subscriber sub_line = nh.subscribe("vector_map_info/line", 1, &CrossWalk::lineCallback, &crosswalk);
  ros::Subscriber sub_point = nh.subscribe("vector_map_info/point", 1, &CrossWalk::pointCallback, &crosswalk);

  // publisher
  ros::Publisher detection_range_pub = nh.advertise<visualization_msgs::MarkerArray>("detection_range", 1);
  ros::Publisher obstacles_pub = nh.advertise<visualization_msgs::MarkerArray>("obstacles", 1);
  ros::Publisher obstacle_waypoint_pub = nh.advertise<std_msgs::Int32>("obstacle_waypoint", 1, true);

  ros::Publisher final_waypoints_pub;
  if (enablePlannerDynamicSwitch)
	  final_waypoints_pub = nh.advertise<autoware_msgs::lane>("astar/final_waypoints", 1, true);
  else
	  final_waypoints_pub = nh.advertise<autoware_msgs::lane>("final_waypoints", 1, true);

  ros::Rate loop_rate(LOOP_RATE);
  while (ros::ok())
  {
    ros::spinOnce();

    int closest_waypoint = 0;

    if (crosswalk.loaded_all && !crosswalk.set_points)
      crosswalk.setCrossWalkPoints();

    if (!vs_info.getSetPose() || !vs_path.getSetPath())
    {
      loop_rate.sleep();
      continue;
    }

    crosswalk.setMultipleDetectionFlag(enable_multiple_crosswalk_detection);

    if (use_crosswalk_detection)
      crosswalk.setDetectionWaypoint(
          crosswalk.findClosestCrosswalk(closest_waypoint, vs_path.getPrevWaypoints(), STOP_SEARCH_DISTANCE));

    std::vector<ObstacleInfo> obstacle_infos;
    EControl detection_result = obstacleDetection(closest_waypoint,
      vs_path.getPrevWaypoints(), crosswalk, vs_info,
      detection_range_pub, obstacles_pub, &obstacle_infos, &tracker);

    changeWaypoints(vs_info, detection_result, closest_waypoint,
                    obstacle_infos, final_waypoints_pub, &vs_path);

    vs_info.clearPoints();

    // publish nearest obstacle waypoint index
    std_msgs::Int32 obstacle_waypoint_index;
    obstacle_waypoint_index.data = (obstacle_infos.size() == 0) ? -1 : obstacle_infos.at(0).waypoint;
    obstacle_waypoint_pub.publish(obstacle_waypoint_index);

    vs_path.resetFlag();

    loop_rate.sleep();
  }

  return 0;
}
