#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <lane_follower/lane.h>
#include <visualization_msgs/Marker.h>

#include <iostream>
#include <sstream>

#include "geo_pos_conv.hh"

//parameter server
//double _initial_velocity_kmh = 5; // km/h
//double _lookahead_threshold = 4.0;
double _threshold_ratio = 1.0;
double _end_distance = 2.0;
std::string _mobility_frame = "/base_link";
std::string _current_pose_topic = "odometry";

const std::string PATH_FRAME = "/path";

geometry_msgs::PoseStamped _current_pose; //グローバル座標系での現在位置
geometry_msgs::Twist _current_velocity;
lane_follower::lane _current_path;

//参照するwaypointの番号
int _next_waypoint = 0;

ros::Publisher vis_pub;


void OdometryPoseCallback(const nav_msgs::OdometryConstPtr &msg)
{
    //std::cout << "odometry callback" << std::endl;
    _current_velocity = msg->twist.twist;

    //テスト版位置情報

    if (_current_pose_topic == "odometry") {
        _current_pose.header = msg->header;
        _current_pose.pose = msg->pose.pose;
    } //else
  //      std::cout << "pose is not odometry" << std::endl;

}

geometry_msgs::PoseStamped _prev_pose;
geometry_msgs::Quaternion _quat;
void GNSSCallback(const sensor_msgs::NavSatFixConstPtr &msg)
{
    //std::cout << "gnss callback" << std::endl;
    if (_current_pose_topic == "gnss") {
        //平面直角座標への変換
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

void NDTCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    //std::cout << "gnss callback" << std::endl;
    if (_current_pose_topic == "ndt") {
        _current_pose.header = msg->header;
        _current_pose.pose = msg->pose;

    } //else
   //     std::cout << "pose is not ndt" << std::endl;

}

void AmclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    if (_current_pose_topic == "amcl") {
        _current_pose.header = msg->header;
        _current_pose.pose = msg->pose.pose;
    }// else
      //  std::cout << "pose is not amcl" << std::endl;
}

void WayPointCallback(const lane_follower::laneConstPtr &msg)
{
    //std::cout << "waypoint callback" << std::endl;
    _current_path = *msg;

    //std::cout << "_current_path frame_id = " << _current_path.header.frame_id
    //       << std::endl;
}

//しきい値を計算
double GetLookAheadThreshold()
{
    //  std::cout << "get lookahead threshold" << std::endl;


    double current_velocity_mps = _current_path.waypoints[_next_waypoint].twist.twist.linear.x;
    double current_velocity_kmph = current_velocity_mps * 3.6;


    if ( current_velocity_kmph > 0 && current_velocity_kmph < 5.0)
        return 2.0 * _threshold_ratio;
    else if( current_velocity_kmph >= 5.0 && current_velocity_kmph < 10)
        return 3.0 * _threshold_ratio;
    else if( current_velocity_kmph >= 10.0 && current_velocity_kmph < 20.0)
            return 6.0 * _threshold_ratio;
    else if( current_velocity_kmph >= 20.0 && current_velocity_kmph < 30.0)
            return 9.0 * _threshold_ratio;
    else if( current_velocity_kmph >= 30.0 && current_velocity_kmph < 40.0)
            return 12.0 * _threshold_ratio;
    else if( current_velocity_kmph >= 40.0)
            return current_velocity_mps * _threshold_ratio;
    else
        return 0;
}

//車の座標系に変換
geometry_msgs::PoseStamped TransformWaypoint(int i)
{
    static tf::TransformListener tfListener;
    geometry_msgs::PoseStamped transformed_waypoint;

    //waypointを車の座標系に変換
    try {
        ros::Time now = ros::Time::now();
        tfListener.waitForTransform(_mobility_frame, _current_path.header.frame_id, now, ros::Duration(0.05));

        tfListener.transformPose(_mobility_frame, _current_path.waypoints[i].pose, transformed_waypoint);

    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());

    }
    return transformed_waypoint;
}

//waypointまでの直線距離を計算
double GetLookAheadDistance(int waypoint)
{
    //std::cout << "get lookahead distance" << std::endl;

    //現在位置
    tf::Vector3 v1(_current_pose.pose.position.x, _current_pose.pose.position.y, _current_pose.pose.position.z);

    tf::Vector3 v2(_current_path.waypoints[waypoint].pose.pose.position.x, _current_path.waypoints[waypoint].pose.pose.position.y, _current_path.waypoints[waypoint].pose.pose.position.z);

    return tf::tfDistance(v1, v2);

}

//waypoint探索
int GetNextWayPoint()
{
    // std::cout << "get nextwaypoint" << std::endl;
    static tf::TransformListener tfListener;

    if (_current_path.waypoints.empty() == false) {

        //初期自己位置と経路の最初の点のズレを計算
        if (_next_waypoint == 0) {
            double distance = GetLookAheadDistance(_next_waypoint);
            if (distance < 2.0) {
                _next_waypoint++;
            } else {
                std::cout << "error!! first waypoint is  so far !!" << std::endl;
                exit(-1);
            }
        }

        //しきい値取得
        double lookahead_threshold = GetLookAheadThreshold();

        //waypoint探索
        for (int i = _next_waypoint; i < _current_path.waypoints.size(); i++) {

            double Distance = GetLookAheadDistance(i);

//            if (_transformed_waypoint.pose.position.x < 0)
            //              continue;

            if (Distance > lookahead_threshold) {
                std::cout << "threshold = " << lookahead_threshold << std::endl;
                std::cout << "distance = " << Distance << std::endl;

                //waypoint をマーカーで表示
                visualization_msgs::Marker marker;
                marker.header.frame_id = PATH_FRAME;
                marker.header.stamp = ros::Time::now();
                marker.ns = "my_namespace";
                marker.id = 0;
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position = _current_path.waypoints[i].pose.pose.position;
                marker.pose.orientation = _current_path.waypoints[i].pose.pose.orientation;
                marker.scale.x = 0.5;
                marker.scale.y = 0.5;
                marker.scale.z = 0.5;
                marker.color.a = 1.0;
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
                vis_pub.publish( marker );
                //ここまで

                return i;

            }

        }
        return 0;

    } else
        // std::cout << "nothing waypoint" << std::endl;
        return 0;
}

//waypointまで到達するための速度を計算
geometry_msgs::Twist CalculateCmdTwist()
{
    std::cout << "calculate" << std::endl;
    geometry_msgs::Twist twist;

    //  std::cout << "current_pose : (" << _current_pose.pose.position.x << " " << _current_pose.pose.position.y << " " << _current_pose.pose.position.z << ")" << std::endl;
    double lookahead_distance = GetLookAheadDistance(_next_waypoint);

    std::cout << "Lookahead Distance = " << lookahead_distance << std::endl;

    //車の座標系に変換したwaypoint
    geometry_msgs::PoseStamped transformed_waypoint = TransformWaypoint(_next_waypoint);

    std::cout << "current path (" << _current_path.waypoints[_next_waypoint].pose.pose.position.x << " " << _current_path.waypoints[_next_waypoint].pose.pose.position.y << " " << _current_path.waypoints[_next_waypoint].pose.pose.position.z << ") ---> transformed_path : (" << transformed_waypoint.pose.position.x << " " << transformed_waypoint.pose.position.y << " " << transformed_waypoint.pose.position.z << ")" << std::endl;

    double radius = pow(lookahead_distance, 2) / (2 * transformed_waypoint.pose.position.y);

    std::cout << "set velocity kmh =" << _current_path.waypoints[_next_waypoint].twist.twist.linear.x * 3.6 << std::endl;

    double initial_velocity_ms = _current_path.waypoints[_next_waypoint].twist.twist.linear.x;
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

static int end_loop = 1;
static double end_ratio = 0.1;
static double end_velocity_kmh = 2.0;
geometry_msgs::Twist EndControl()
{
    std::cout << "end control" << std::endl;
    geometry_msgs::Twist twist;

    std::cout << "End Distance = " << _end_distance << std::endl;
    double lookahead_distance = GetLookAheadDistance(_current_path.waypoints.size() - 1);
    std::cout << "Lookahead Distance = " << lookahead_distance << std::endl;
    double initial_velocity_kmh = (_current_path.waypoints[_current_path.waypoints.size() - 1].twist.twist.linear.x * 3.6 - end_ratio * end_loop);
    std::cout << "set velocity (kmh) = " << initial_velocity_kmh << std::endl;
    double initial_velocity_ms = initial_velocity_kmh / 3.6;

    if (lookahead_distance < _end_distance) {
        twist.linear.x = 0;
        twist.angular.z = 0;
    } else {

        if (initial_velocity_kmh < end_velocity_kmh)
            initial_velocity_ms = end_velocity_kmh / 3.6;

        //車の座標系に変換したwaypoint
        geometry_msgs::PoseStamped transformed_waypoint = TransformWaypoint(_current_path.waypoints.size() - 1);

        double radius = pow(lookahead_distance, 2) / (2 * transformed_waypoint.pose.position.y);
        double angular_velocity;

        if (radius > 0 || radius < 0) {
            angular_velocity = initial_velocity_ms / radius;
        } else {
            angular_velocity = 0;
        }

        double linear_velocity = initial_velocity_ms;

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

//setting params

    private_nh.getParam("current_pose_topic", _current_pose_topic);
    std::cout << "current_pose_topic : " << _current_pose_topic << std::endl;

    private_nh.getParam("mobility_frame", _mobility_frame);
    std::cout << "mobility_frame : " << _mobility_frame << std::endl;

    /* private_nh.getParam("velocity_kmh", _initial_velocity_kmh);
     std::cout << "initial_velocity : " << _initial_velocity_kmh << std::endl;

     private_nh.getParam("lookahead_threshold", _lookahead_threshold);
     std::cout << "lookahead_threshold : " << _lookahead_threshold << std::endl;
     */
    private_nh.getParam("threshold_ratio", _threshold_ratio);
    std::cout << "threshold_ratio : " << _threshold_ratio << std::endl;

    private_nh.getParam("end_distance", _end_distance);
    std::cout << "end_distance : " << _end_distance << std::endl;

//publish topic
    ros::Publisher cmd_velocity_publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

vis_pub = nh.advertise<visualization_msgs::Marker>( "waypoint_marker", 0 );


//subscribe topic
    ros::Subscriber waypoint_subcscriber = nh.subscribe("ruled_waypoint", 1000, WayPointCallback);
    ros::Subscriber odometry_subscriber = nh.subscribe("pose", 1000, OdometryPoseCallback);

    ros::Subscriber gnss_subscriber = nh.subscribe("fix", 1000, GNSSCallback);

    ros::Subscriber amcl_subscriber = nh.subscribe("amcl_pose", 1000, AmclCallback);

    ros::Subscriber ndt_subscriber = nh.subscribe("ndt_pose", 1000, NDTCallback);

    geometry_msgs::Twist twist;


    ros::Rate loop_rate(10); //Hzで指定
    bool endflag = false;
    while (ros::ok()) {
        ros::spinOnce();

        if (endflag == false) {

            //waypoint取得
            std::cout << "waypoint count =" << _current_path.waypoints.size() << std::endl;
            _next_waypoint = GetNextWayPoint();
            std::cout << "nextwaypoint = " << _next_waypoint << std::endl;

            if (_next_waypoint > 0) {

                //速度を計算
                twist = CalculateCmdTwist();

            } else {
                twist.linear.x = 0;
                twist.angular.z = 0;
            }

        } else
            twist = EndControl();

        if (_next_waypoint == _current_path.waypoints.size() - 1)
            endflag = true;

        std::cout << "linear.x = " << twist.linear.x << " angular.z = " << twist.angular.z << std::endl << std::endl;
        cmd_velocity_publisher.publish(twist);

        loop_rate.sleep();
    }

    return 0;

}
