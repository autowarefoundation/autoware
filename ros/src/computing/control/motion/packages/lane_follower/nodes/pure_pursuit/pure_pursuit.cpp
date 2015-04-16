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
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <lane_follower/lane.h>
#include <visualization_msgs/Marker.h>
#include <runtime_manager/ConfigLaneFollower.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <sstream>

#include "geo_pos_conv.hh"

// parameter servers
static double _initial_velocity_kmh = 5; // km/h
static double _lookahead_threshold = 4.0;
static double _threshold_ratio = 1.0;
static double _end_distance = 2.0;
static double _error_distance = 2.5;
static std::string _mobility_frame = "/base_link"; // why is this default?
static std::string _current_pose_topic = "ndt";

static const std::string PATH_FRAME = "/map";

static geometry_msgs::PoseStamped _current_pose; // current pose by the global plane.
static geometry_msgs::Twist _current_velocity;
static lane_follower::lane _current_path;

// ID (index) of the next waypoint.
static int _next_waypoint = 0;

static ros::Publisher _vis_pub;
static ros::Publisher _circle_pub;
static ros::Publisher _stat_pub;
static std_msgs::Bool _lf_stat;
static bool _fix_flag = false;
static bool _param_set = false;
static tf::Transform _transform;
static tf::Vector3 _origin_v(0, 0, 0);

static void ConfigCallback(const runtime_manager::ConfigLaneFollowerConstPtr config)
{
    _initial_velocity_kmh = config->velocity;
    _lookahead_threshold = config->lookahead_threshold;
    _param_set = true;
}

static void OdometryPoseCallback(const nav_msgs::OdometryConstPtr &msg)
{
    //std::cout << "odometry callback" << std::endl;
    _current_velocity = msg->twist.twist;

    //
    // effective for testing.
    //
    if (_current_pose_topic == "odometry") {
        _current_pose.header = msg->header;
        _current_pose.pose = msg->pose.pose;

        tf::Transform inverse;
        tf::poseMsgToTF(msg->pose.pose,inverse);
_transform = inverse.inverse();
     //   std::cout << "transform2 (" << _transform2.getOrigin().x() << " " <<  _transform2.getOrigin().y() << " " <<  _transform2.getOrigin().z() << ")" << std::endl;
    } //else
      //      std::cout << "pose is not odometry" << std::endl;

}

/*static geometry_msgs::PoseStamped _prev_pose;
static geometry_msgs::Quaternion _quat;
static void GNSSCallback(const sensor_msgs::NavSatFixConstPtr &msg)
{
    //std::cout << "gnss callback" << std::endl;
    if (_current_pose_topic == "gnss") {
        // transform to the rectangular plane.
        geo_pos_conv geo;
        geo.set_plane(7);
        geo.llh_to_xyz(msg->latitude, msg->longitude, msg->altitude);
        _current_pose.header = msg->header;
        _current_pose.pose.position.x = geo.y();
        _current_pose.pose.position.y = geo.x();
        _current_pose.pose.position.z = geo.z();
        double distance = sqrt(pow(_current_pose.pose.position.y - _prev_pose.pose.position.y, 2) + pow(_current_pose.pose.position.x - _prev_pose.pose.position.x, 2));
        std::cout << "distance : " << distance << std::endl;
        if (distance > 0.2) {
            double yaw = atan2(_current_pose.pose.position.y - _prev_pose.pose.position.y, _current_pose.pose.position.x - _prev_pose.pose.position.x);
            _quat = tf::createQuaternionMsgFromYaw(yaw);
            _prev_pose = _current_pose;
        }
        _current_pose.pose.orientation = _quat;

    } //else
      //      std::cout << "pose is not gnss" << std::endl;
}
*/
static void NDTCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    //std::cout << "gnss callback" << std::endl;
    if (_current_pose_topic == "ndt") {
        _current_pose.header = msg->header;
        _current_pose.pose = msg->pose;
        tf::Transform inverse;
        tf::poseMsgToTF(msg->pose,inverse);
        _transform = inverse.inverse();

    } //else
      //     std::cout << "pose is not ndt" << std::endl;
}

static void WayPointCallback(const lane_follower::laneConstPtr &msg)
{
    //std::cout << "waypoint callback" << std::endl;
    _current_path = *msg;

    //std::cout << "_current_path frame_id = " << _current_path.header.frame_id
    //       << std::endl;
}

/////////////////////////////////////////////////////////////////
// obtain the threshold where the next waypoint may be selected.
/////////////////////////////////////////////////////////////////
static double GetLookAheadThreshold()
{
    //  std::cout << "get lookahead threshold" << std::endl;

    if (_fix_flag)
        return _lookahead_threshold;

    double current_velocity_mps = _current_path.waypoints[_next_waypoint].twist.twist.linear.x;
    double current_velocity_kmph = current_velocity_mps * 3.6;
    if(current_velocity_kmph * 0.5 < 5)
        return 5;
    else
        return current_velocity_kmph * 0.5;
}

static double GetEvaluation(int closest, int i)
{
    int num = i - closest;
    double ratio = 0.05;
    return num * ratio;
}

/////////////////////////////////////////////////////////////////
// transform the waypoint to the vehicle plane.
/////////////////////////////////////////////////////////////////
geometry_msgs::Point TransformWaypoint(int i)
{
    tf::Vector3 original(_current_path.waypoints[i].pose.pose.position.x, _current_path.waypoints[i].pose.pose.position.y, _current_path.waypoints[i].pose.pose.position.z);

    geometry_msgs::Point point;
    tf::Vector3 transformed = _transform * original;
    point.x = transformed.getX();
    point.y = transformed.getY();
    point.z = transformed.getZ();

   // std::cout << "transformed pose (" << point.x << " " << point.y << " " << point.z << ") transformed2 pose : (" << point2.x << " " << point2.y << " " << point2.z << ")" << std::endl;

    return point;

}

/////////////////////////////////////////////////////////////////
// obtain the distance to @waypoint. ignore z position
/////////////////////////////////////////////////////////////////
double GetLookAheadDistance(int waypoint)
{
    //std::cout << "get lookahead distance" << std::endl;

    // position of @waypoint.
    tf::Vector3 v2(_current_path.waypoints[waypoint].pose.pose.position.x, _current_path.waypoints[waypoint].pose.pose.position.y, _current_path.waypoints[waypoint].pose.pose.position.z);
    tf::Vector3 tf_v2 = _transform * v2;
    tf_v2.setZ(0);
     //std::cout << "tf_v2 : (" << tf_v2.getX() << " " << tf_v2.getY() << " " << tf_v2.getZ() << ")" << std::endl;

    //return tf::tfDistance(v1, v2);
    return tf::tfDistance(_origin_v, tf_v2);
}

/////////////////////////////////////////////////////////////////
// obtain the velocity(m/s) waypoint under the vehicle has.
/////////////////////////////////////////////////////////////////
int GetClosestWaypointNum()
{
    //std::cout << "search waypoint nearest the vehicle" << std::endl;
    double distance = 10000; //meter
    double waypoint = 0;

    for (int i = 1; i < _current_path.waypoints.size(); i++) {
        //std::cout << waypoint << std::endl;

        // position of @waypoint.
        tf::Vector3 v2(_current_path.waypoints[i].pose.pose.position.x, _current_path.waypoints[i].pose.pose.position.y, 0);
        tf::Vector3 tf_v2 = _transform * v2;
        tf_v2.setZ(0);
        //std::cout << "current path (" << _current_path.waypoints[i].pose.pose.position.x << " " << _current_path.waypoints[i].pose.pose.position.y << " " << _current_path.waypoints[i].pose.pose.position.z << ")" << std::endl;

        //double dt = tf::tfDistance(v1, v2);
        double dt = tf::tfDistance(_origin_v, tf_v2);
        //  std::cout << i  << " "<< dt << std::endl;
        if (dt < distance) {
            distance = dt;
            waypoint = i;
            // std::cout << "waypoint = " << i  << "  distance = "<< dt << std::endl;
        }
    }
    return waypoint;
}

/////////////////////////////////////////////////////////////////
// obtain the velocity(m/s) waypoint under the vehicle has.
/////////////////////////////////////////////////////////////////
double GetWaypointVelocity()
{
    //std::cout << "get velocity from waypoint near the vehicle" << std::endl;
    static double velocity = _current_path.waypoints[1].twist.twist.linear.x;

    return _current_path.waypoints[GetClosestWaypointNum()].twist.twist.linear.x;
}

double CalcRadius(int waypoint)
{
    //  std::cout << "current_pose : (" << _current_pose.pose.position.x << " " << _current_pose.pose.position.y << " " << _current_pose.pose.position.z << ")" << std::endl;
    double lookahead_distance = GetLookAheadDistance(waypoint);

    //std::cout << "Lookahead Distance = " << lookahead_distance << std::endl;

    // transform the waypoint to the car plane.
    geometry_msgs::Point transformed_waypoint = TransformWaypoint(waypoint);

    //std::cout << "current path (" << _current_path.waypoints[waypoint].pose.pose.position.x << " " << _current_path.waypoints[waypoint].pose.pose.position.y << " " << _current_path.waypoints[waypoint].pose.pose.position.z << ") ---> transformed_path : (" << transformed_waypoint.x << " " << transformed_waypoint.y << " " << transformed_waypoint.z << ")" << std::endl;

    double radius = pow(lookahead_distance, 2) / (2 * transformed_waypoint.y);
    //std::cout << "radius = " << radius << std::endl;
    return radius;

}




/////////////////////////////////////////////////////////////////
// obtain the next "effective" waypoint. 
// the vehicle drives itself toward this waypoint.
/////////////////////////////////////////////////////////////////
int GetNextWayPoint()
{
    // if waypoints are not given, do nothing.
    if (_current_path.waypoints.empty() == true) {
        _lf_stat.data = false;
        _stat_pub.publish(_lf_stat);
        return 0;
    }

    // seek for the first effective waypoint.
    if (_next_waypoint == 0) {
        do {
            // if there exists a close waypoint, start from this waypoint.
            if (GetLookAheadDistance(_next_waypoint) < _error_distance) {
                _next_waypoint++; // why is this needed?
                break;
            }
        } while (_next_waypoint++ < _current_path.waypoints.size());

        // if no waypoint founded close enough, fall back!
        if (_next_waypoint == _current_path.waypoints.size()) {
            std::cout << "no waypoint on the path!" << std::endl;
            _next_waypoint = 0;
            _lf_stat.data = false;
            _stat_pub.publish(_lf_stat);
            return 0;
        }
    }
    // the next waypoint must be outside of this threthold.

    int minimum_th = 3;
    double lookahead_threshold = GetLookAheadThreshold();
    int closest = GetClosestWaypointNum();
    std::cout << "nearest waypoint = " << closest << std::endl;

    // look for the next waypoint.
    for (int i = closest; i < _current_path.waypoints.size(); i++) {

        double Distance = GetLookAheadDistance(i);

        // if there exists an effective waypoint
        if (Distance > lookahead_threshold) {

            if (!_fix_flag) {

                double radius = CalcRadius(i);
                if (radius < 0)
                    radius = (-1) * radius;
                //std::cout << "waypoint = " << i << std::endl;
                //std::cout << "threshold = " << lookahead_threshold << std::endl;
                //std::cout << "distance = " << Distance << std::endl;

                tf::Vector3 center;

                if (TransformWaypoint(i).y > 0) {
                    center = tf::Vector3(0, 0 + radius, 0);
                } else {
                    center = tf::Vector3(0, 0 - radius, 0);
                }

                geometry_msgs::Point point;
                tf::Vector3 inv_center = _transform.inverse() * center;

                point.x = inv_center.getX();
                point.y = inv_center.getY();
                point.z = inv_center.getZ();
                // display the next waypoint by markers.
                visualization_msgs::Marker circle;
                circle.header.frame_id = PATH_FRAME;
                circle.header.stamp = ros::Time::now();
                circle.ns = "circle";
                circle.id = 0;
                circle.type = visualization_msgs::Marker::SPHERE;
                circle.action = visualization_msgs::Marker::ADD;
                circle.pose.position = point;
                circle.scale.x = radius * 2;
                circle.scale.y = radius * 2;
                circle.scale.z = 1.0;
                circle.color.a = 0.3;
                circle.color.r = 1.0;
                circle.color.g = 0.0;
                circle.color.b = 0.0;
                _circle_pub.publish(circle);

                //evaluation waypoint
                double evaluation = 0;
                for (int j = closest + 1; j < i; j++) {
                    tf::Vector3 waypoint(_current_path.waypoints[j].pose.pose.position.x, _current_path.waypoints[j].pose.pose.position.y, _current_path.waypoints[j].pose.pose.position.z);

                    //  std::cout << "center(" << center.x() << " " << center.y() << " " << center.z() << ")" << std::endl;
                    //  std::cout << "waypoint (" << waypoint.x() << " " << waypoint.y() << " " << waypoint.z() << ")" << std::endl;

                    tf::Vector3 tf_waypoint  = _transform * waypoint;
                    tf_waypoint.setZ(0);
                    double dt = fabs(tf::tfDistance(center, tf_waypoint));
                    double dt_diff = fabs(dt - radius);

                    evaluation += dt_diff;
                }

                double eval_value = GetEvaluation(closest, i);
                //std::cout << "evaluation (" << evaluation << " " << eval_value << "）" << std::endl;

                if (evaluation < eval_value) {
                    //     std::cout << "ok!" << std::endl;
                } else {
                    //     std::cout << "bad" << std::endl;

                    if (lookahead_threshold > minimum_th) {
                        lookahead_threshold -= 0.1;
                        i = closest;
                        continue;
                    } else
                        lookahead_threshold = minimum_th;
                }
            }

            std::cout << "threshold = " << lookahead_threshold << std::endl;
            // display the next waypoint by markers.
            visualization_msgs::Marker marker;
            marker.header.frame_id = PATH_FRAME;
            marker.header.stamp = ros::Time::now();
            marker.ns = "my_namespace";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position = _current_path.waypoints[i].pose.pose.position;
            marker.pose.orientation = _current_path.waypoints[i].pose.pose.orientation;
            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;

            _vis_pub.publish(marker);

            //status turns true
            _lf_stat.data = true;
            _stat_pub.publish(_lf_stat);
            return i;
        }
    }

    // if the program reaches here, it means we lost the waypoint.
    // so let's try again with the first waypoint.
    std::cout << "lost waypoint!" << std::endl;
    std::cout << "seeking from the first waypoint..." << std::endl;
    _next_waypoint = 0;
    _lf_stat.data = false;
    _stat_pub.publish(_lf_stat);
    return 0;
}


/////////////////////////////////////////////////////////////////
// obtain the linear/angular velocity toward the next waypoint.
/////////////////////////////////////////////////////////////////
static geometry_msgs::Twist CalculateCmdTwist()
{
    std::cout << "calculate" << std::endl;

    geometry_msgs::Twist twist;

    double radius = CalcRadius(_next_waypoint);
    double initial_velocity_ms = 0;
    if (_fix_flag == false)
        initial_velocity_ms = GetWaypointVelocity();
    else
        initial_velocity_ms = _initial_velocity_kmh / 3.6;

    std::cout << "set velocity kmh =" << initial_velocity_ms * 3.6 << std::endl;
    //std::cout << "initial_velocity_ms : " << initial_velocity_ms << std::endl;
    double angular_velocity;

    if (radius > 0 || radius < 0) {
        angular_velocity = initial_velocity_ms / radius;
    } else {
        angular_velocity = 0;
    }

    double linear_velocity = initial_velocity_ms;

    twist.linear.x = linear_velocity;
    twist.angular.z = angular_velocity;

    return twist;
}

/////////////////////////////////////////////////////////////////
// Safely stop the vehicle.
/////////////////////////////////////////////////////////////////
static int end_loop = 1;
static double end_ratio = 0.2;
static double end_velocity_kmh = 2.0;
static geometry_msgs::Twist EndControl()
{
    std::cout << "end control" << std::endl;
    geometry_msgs::Twist twist;

    std::cout << "End Distance = " << _end_distance << std::endl;

    double lookahead_distance = GetLookAheadDistance(_current_path.waypoints.size() - 1);
    std::cout << "Lookahead Distance = " << lookahead_distance << std::endl;

    double velocity_kmh;

    if (_fix_flag == true) {
        velocity_kmh = _initial_velocity_kmh - end_ratio * pow(end_loop,2);
    } else {
        velocity_kmh = (_current_path.waypoints[_current_path.waypoints.size() - 1].twist.twist.linear.x * 3.6 - end_ratio * pow(end_loop,2));
    }

    double velocity_ms = velocity_kmh / 3.6;

    if (lookahead_distance < _end_distance) { // EndControl completed

        twist.linear.x = 0;
        twist.angular.z = 0;

        _next_waypoint = 0;

        _lf_stat.data = false;
        _stat_pub.publish(_lf_stat);

	} else {

        if (velocity_kmh < end_velocity_kmh)
            velocity_ms = end_velocity_kmh / 3.6;

        std::cout << "set velocity (kmh) = " << velocity_ms * 3.6 << std::endl;

        double radius = CalcRadius(_next_waypoint);
        double angular_velocity;

        if (radius > 0 || radius < 0) {
            angular_velocity = velocity_ms / radius;
        } else {
            angular_velocity = 0;
        }

        double linear_velocity = velocity_ms;

        twist.linear.x = linear_velocity;
        twist.angular.z = angular_velocity;
        end_loop++;
    }
    return twist;
}

int main(int argc, char **argv)
{
    std::cout << "lane follower start" << std::endl;

// set up ros
    ros::init(argc, argv, "lane_follower");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

// setting params
    private_nh.getParam("current_pose_topic", _current_pose_topic);
    std::cout << "current_pose_topic : " << _current_pose_topic << std::endl;

    private_nh.getParam("mobility_frame", _mobility_frame);
    std::cout << "mobility_frame : " << _mobility_frame << std::endl;

    private_nh.getParam("fix_flag", _fix_flag);
    std::cout << "fix_flag : " << _fix_flag << std::endl;

    // private_nh.getParam("velocity_kmh", _initial_velocity_kmh);
    // std::cout << "initial_velocity : " << _initial_velocity_kmh << std::endl;
    //
    // private_nh.getParam("lookahead_threshold", _lookahead_threshold);
    // std::cout << "lookahead_threshold : " << _lookahead_threshold << std::endl;

    private_nh.getParam("threshold_ratio", _threshold_ratio);
    std::cout << "threshold_ratio : " << _threshold_ratio << std::endl;

    private_nh.getParam("end_distance", _end_distance);
    std::cout << "end_distance : " << _end_distance << std::endl;

    private_nh.getParam("error_distance", _error_distance);
    std::cout << "error_distance : " << _error_distance << std::endl;

//publish topic
    ros::Publisher cmd_velocity_publisher = nh.advertise<geometry_msgs::TwistStamped>("twist_cmd", 1000);

    _vis_pub = nh.advertise<visualization_msgs::Marker>("target_waypoint_mark", 0);
    _circle_pub = nh.advertise<visualization_msgs::Marker>("circle_mark", 0);
    _stat_pub = nh.advertise<std_msgs::Bool>("lf_stat", 0);

//subscribe topic
    ros::Subscriber waypoint_subcscriber = nh.subscribe("ruled_waypoint", 1000, WayPointCallback);
    ros::Subscriber odometry_subscriber = nh.subscribe("odom_pose", 1000, OdometryPoseCallback);

    //ros::Subscriber gnss_subscriber = nh.subscribe("fix", 1000, GNSSCallback);

    ros::Subscriber ndt_subscriber = nh.subscribe("control_pose", 1000, NDTCallback);

    ros::Subscriber config_subscriber = nh.subscribe("config/lane_follower", 1000, ConfigCallback);

    geometry_msgs::TwistStamped twist;

    ros::Rate loop_rate(10); // by Hz
    bool endflag = false;
    while (ros::ok()) {
        ros::spinOnce();

        if (_fix_flag == true && _param_set == false) {
            std::cout << "parameter waiting..." << std::endl;
            loop_rate.sleep();
            continue;
        }

        if (endflag == false) {

            // get the waypoint.
            _next_waypoint = GetNextWayPoint();
            std::cout << "next waypoint = " << _next_waypoint;
            std::cout << "/" << _current_path.waypoints.size() << std::endl;

            if (_next_waypoint > 0) {
                // obtain the linear/angular velocity.
                twist.twist = CalculateCmdTwist();
            } else {
                twist.twist.linear.x = 0;
                twist.twist.angular.z = 0;
            }

        } else {
            twist.twist = EndControl();

            // after stopped or fed out, let's get ready for the restart.
            if (_next_waypoint == 0) {
                endflag = false;
            }
        }

        if (_next_waypoint == _current_path.waypoints.size() - 1) {
            endflag = true;
        }

        std::cout << "twist.linear.x = " << twist.twist.linear.x << std::endl;
        std::cout << "twist.angular.z = " << twist.twist.angular.z << std::endl;
        std::cout << std::endl;

        twist.header.stamp = ros::Time::now();
        cmd_velocity_publisher.publish(twist);

        loop_rate.sleep();
    }

    return 0;
}
