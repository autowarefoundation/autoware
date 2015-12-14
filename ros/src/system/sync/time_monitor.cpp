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
/* ----header---- */
/* common header */
#include "ros/ros.h"
#include <sstream>
#include <vector>
#include <boost/circular_buffer.hpp>
#include "std_msgs/Time.h"
/* user header */
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "cv_tracker/image_obj.h"
#include "points2image/PointsImage.h"
#include "cv_tracker/image_obj_ranged.h"
#include "cv_tracker/image_obj_tracked.h"
#include "geometry_msgs/PoseStamped.h"
#include "cv_tracker/obj_label.h"
#include "lidar_tracker/centroids.h"
#include "visualization_msgs/MarkerArray.h"
#include "synchronization/time_monitor.h"
#include "synchronization/time_diff.h"

/* ----var---- */
/* common var */

/* user var */
class KeyPair
{
    boost::circular_buffer<ros::Time> sensor;
    boost::circular_buffer<ros::Time> execution;

public:
    KeyPair() {
        sensor.resize(1);
        execution.resize(1);
    }
    void resize(int size) {
        sensor.resize(size);
        execution.resize(size);
    }
    void push_front(const ros::Time& sensor_time, const ros::Time& execution_time) {
        sensor.push_front(sensor_time);
        execution.push_front(execution_time);
    }

    ros::Time find(ros::Time sensor_time) {
        boost::circular_buffer<ros::Time>::iterator it = sensor.begin();
        for (int i = 0; it != sensor.end(); it++, i++) {
            if (it->sec == sensor_time.sec && it->nsec == sensor_time.nsec) {
                return execution.at(i); // find
            }
        }
        ros::Time failed;
        failed.sec = 0;
        failed.nsec = 0;
        return failed; // not find
    }
};


class TimeManager
{
    KeyPair image_raw_;
    KeyPair points_raw_;
    KeyPair vscan_points_;
    KeyPair vscan_image_;
    KeyPair image_obj_;
    KeyPair image_obj_ranged_;
    KeyPair image_obj_tracked_;
    KeyPair current_pose_;
    KeyPair obj_label_;
    KeyPair cluster_centroids_;
    KeyPair obj_pose_;
    KeyPair time_diff_;

    ros::NodeHandle nh;
    ros::Subscriber image_raw_sub;
    ros::Subscriber points_raw_sub;
    ros::Subscriber vscan_points_sub;
    ros::Subscriber vscan_image_sub;
    ros::Subscriber image_obj_sub;
    ros::Subscriber image_obj_ranged_sub;
    ros::Subscriber image_obj_tracked_sub;
    ros::Subscriber current_pose_sub;
    ros::Subscriber obj_label_sub;
    ros::Subscriber cluster_centroids_sub;
    ros::Subscriber obj_pose_sub;
    ros::Subscriber time_diff_sub;

    ros::Publisher time_monitor_pub;

    double ros_time2msec(ros::Time time) {
        return (double)time.sec*1000L + (double)time.nsec/1000000L;
    }
    double time_diff(ros::Time sensor_time, ros::Time execution_time) {
        if (execution_time.sec == 0 && execution_time.nsec == 0) { // not find
            return 0.0;
        }

        double time_diff = ros_time2msec(execution_time) - ros_time2msec(sensor_time);
        if (time_diff > 0)
            return time_diff;
        else
            return 0.0;
    }

public:
    TimeManager(int);
    void image_raw_callback(const sensor_msgs::Image::ConstPtr& image_raw_msg);
    void points_raw_callback(const sensor_msgs::PointCloud2::ConstPtr& points_raw_msg);
    void vscan_points_callback(const sensor_msgs::PointCloud2::ConstPtr& vscan_points_msg);
    void vscan_image_callback(const points2image::PointsImage::ConstPtr& vscan_image_msg);
    void image_obj_callback(const cv_tracker::image_obj::ConstPtr& image_obj_msg);
    void image_obj_ranged_callback(const cv_tracker::image_obj_ranged::ConstPtr& image_obj_ranged_msg);
    void image_obj_tracked_callback(const cv_tracker::image_obj_tracked::ConstPtr& image_obj_tracked_msg);
    void current_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& current_pose_msg);
    void obj_label_callback(const cv_tracker::obj_label::ConstPtr& obj_label_msg) ;
    void cluster_centroids_callback(const lidar_tracker::centroids::ConstPtr& cluster_centroids_msg);
    void obj_pose_callback(const visualization_msgs::MarkerArray::ConstPtr& obj_pose_msg);
    void time_diff_callback(const synchronization::time_diff::ConstPtr& time_diff_msg);
    void run();
};

TimeManager::TimeManager(int buffer_size) {
    ros::NodeHandle private_nh("~");
    time_monitor_pub = nh.advertise<std_msgs::Time> ("/times", 1);
    image_raw_sub = nh.subscribe("/image_raw", 1, &TimeManager::image_raw_callback, this);
    points_raw_sub = nh.subscribe("/points_raw", 1, &TimeManager::points_raw_callback, this);
    vscan_points_sub = nh.subscribe("/vscan_points", 1, &TimeManager::vscan_points_callback, this);
    vscan_image_sub = nh.subscribe("/vscan_image", 1, &TimeManager::vscan_image_callback, this);
    image_obj_sub = nh.subscribe("/obj_car/image_obj", 1, &TimeManager::image_obj_callback, this);
    image_obj_ranged_sub = nh.subscribe("/obj_car/image_obj_ranged", 1, &TimeManager::image_obj_ranged_callback, this);
    image_obj_tracked_sub = nh.subscribe("/obj_car/image_obj_tracked", 1, &TimeManager::image_obj_tracked_callback, this);
    current_pose_sub = nh.subscribe("current_pose", 1, &TimeManager::current_pose_callback, this);
    obj_label_sub = nh.subscribe("/obj_car/obj_label", 1, &TimeManager::obj_label_callback, this);
    cluster_centroids_sub = nh.subscribe("/cluster_centroids", 1, &TimeManager::cluster_centroids_callback, this);
    obj_pose_sub = nh.subscribe("/obj_car/obj_pose", 1, &TimeManager::obj_pose_callback, this);
    time_diff_sub = nh.subscribe("/time_diff", 1, &TimeManager::time_diff_callback, this);

    image_raw_.resize(buffer_size);
    points_raw_.resize(buffer_size);
    vscan_points_.resize(buffer_size);
    vscan_image_.resize(buffer_size);
    image_obj_.resize(buffer_size);
    image_obj_ranged_.resize(buffer_size);
    image_obj_tracked_.resize(buffer_size);
    current_pose_.resize(buffer_size);
    obj_label_.resize(buffer_size);
    cluster_centroids_.resize(buffer_size);
    time_diff_.resize(buffer_size);
}

void TimeManager::image_raw_callback(const sensor_msgs::Image::ConstPtr& image_raw_msg) {
    image_raw_.push_front(image_raw_msg->header.stamp, ros::Time::now());
}

void TimeManager::points_raw_callback(const sensor_msgs::PointCloud2::ConstPtr& points_raw_msg) {
    points_raw_.push_front(points_raw_msg->header.stamp, ros::Time::now());
}

void TimeManager::vscan_points_callback(const sensor_msgs::PointCloud2::ConstPtr& vscan_points_msg) {
    vscan_points_.push_front(vscan_points_msg->header.stamp, ros::Time::now());
}

void TimeManager::vscan_image_callback(const points2image::PointsImage::ConstPtr& vscan_image_msg) {
    vscan_image_.push_front(vscan_image_msg->header.stamp, ros::Time::now());
}

void TimeManager::image_obj_callback(const cv_tracker::image_obj::ConstPtr& image_obj_msg) {
    image_obj_.push_front(image_obj_msg->header.stamp, ros::Time::now());
}

void TimeManager::image_obj_ranged_callback(const cv_tracker::image_obj_ranged::ConstPtr& image_obj_ranged_msg) {
    image_obj_ranged_.push_front(image_obj_ranged_msg->header.stamp, ros::Time::now());
}

void TimeManager::image_obj_tracked_callback(const cv_tracker::image_obj_tracked::ConstPtr& image_obj_tracked_msg) {
    image_obj_tracked_.push_front(image_obj_tracked_msg->header.stamp, ros::Time::now());
}

void TimeManager::current_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& current_pose_msg) {
    current_pose_.push_front(current_pose_msg->header.stamp, ros::Time::now());
}

void TimeManager::obj_label_callback(const cv_tracker::obj_label::ConstPtr& obj_label_msg) {
    obj_label_.push_front(obj_label_msg->header.stamp, ros::Time::now());
}

void TimeManager::cluster_centroids_callback(const lidar_tracker::centroids::ConstPtr& cluster_centroids_msg) {
    cluster_centroids_.push_front(cluster_centroids_msg->header.stamp, ros::Time::now());
}

void TimeManager::time_diff_callback(const synchronization::time_diff::ConstPtr& time_diff_msg) {
    ros::Time time_diff;
    time_diff.sec = time_diff_msg->lidar.sec - time_diff_msg->camera.sec;
    time_diff.nsec = time_diff_msg->lidar.nsec - time_diff_msg->camera.nsec;
    time_diff_.push_front(time_diff_msg->header.stamp, time_diff);
}

void TimeManager::obj_pose_callback(const visualization_msgs::MarkerArray::ConstPtr& obj_pose_msg) {
    obj_pose_.push_front(obj_pose_msg->header.stamp, ros::Time::now());
    static ros::Time pre_sensor_time;
    synchronization::time_monitor time_monitor_msg;
    time_monitor_msg.header.frame_id = "0";
    time_monitor_msg.header.stamp = obj_pose_msg->header.stamp;

    time_monitor_msg.image_raw = time_diff(obj_pose_msg->header.stamp, image_raw_.find(obj_pose_msg->header.stamp));
    time_monitor_msg.points_raw = time_diff(obj_pose_msg->header.stamp, points_raw_.find(obj_pose_msg->header.stamp));
    time_monitor_msg.vscan_points = time_diff(obj_pose_msg->header.stamp, vscan_points_.find(obj_pose_msg->header.stamp));
    time_monitor_msg.vscan_image = time_diff(obj_pose_msg->header.stamp, vscan_image_.find(obj_pose_msg->header.stamp));
    time_monitor_msg.image_obj = time_diff(obj_pose_msg->header.stamp, image_obj_.find(obj_pose_msg->header.stamp));
    time_monitor_msg.image_obj_ranged = time_diff(obj_pose_msg->header.stamp, image_obj_ranged_.find(obj_pose_msg->header.stamp));
    time_monitor_msg.image_obj_tracked = time_diff(obj_pose_msg->header.stamp, image_obj_tracked_.find(obj_pose_msg->header.stamp));
    time_monitor_msg.current_pose = time_diff(obj_pose_msg->header.stamp, current_pose_.find(obj_pose_msg->header.stamp));
    time_monitor_msg.obj_label = time_diff(obj_pose_msg->header.stamp, obj_label_.find(obj_pose_msg->header.stamp));
    time_monitor_msg.cluster_centroids = time_diff(obj_pose_msg->header.stamp, cluster_centroids_.find(obj_pose_msg->header.stamp));
    time_monitor_msg.obj_pose = time_diff(obj_pose_msg->header.stamp, obj_pose_.find(obj_pose_msg->header.stamp));
    time_monitor_msg.execution_time = time_diff(obj_pose_msg->header.stamp, obj_pose_.find(obj_pose_msg->header.stamp)); // execution time
    time_monitor_msg.cycle_time = time_diff(pre_sensor_time, obj_pose_msg->header.stamp); // cycle time
    time_monitor_msg.time_diff = ros_time2msec(time_diff_.find(obj_pose_msg->header.stamp)); // time difference
    time_monitor_pub.publish(time_monitor_msg);
    pre_sensor_time = obj_pose_msg->header.stamp;
}

void TimeManager::run() {
    ros::spin();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "time_monitor");

  TimeManager time_manager(32);
  time_manager.run();

  return 0;
}
