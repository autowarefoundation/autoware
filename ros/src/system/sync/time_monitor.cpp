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
    void push_front(ros::Time sensor_time, ros::Time execution_time) {
        sensor.push_front(sensor_time);
        execution.push_front(execution_time);
    }
    ros::Time front(void) {
        return execution.front();
    }

    ros::Time find(ros::Time sensor_time) {
        boost::circular_buffer<ros::Time>::iterator it = sensor.begin();
        for (int i = 0; it != sensor.end(); it++, i++) {
            if (it->sec == sensor_time.sec && it->nsec == sensor_time.nsec) {
                return execution.at(i); // find
            }
        }
        ROS_ERROR("error:not found a pair");
        ros::Time failed;
        failed.sec = 0;
        failed.nsec = 0;
        return failed; // not find
    }
};


class TimeManager
{
    /*
     * KeyPair
     */
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
    // sync
    KeyPair sync_image_obj_ranged_;
    KeyPair sync_image_obj_tracked_;
    KeyPair sync_obj_label_;
    KeyPair sync_obj_pose_;
    // time difference
    KeyPair time_diff_;

    /*
     * Nodehandle, Subscriber, Publisher
     */
    // Nodehandle
    ros::NodeHandle nh;
    // Subscriber
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
    ros::Subscriber sync_image_obj_ranged_sub; // sync
    ros::Subscriber sync_image_obj_tracked_sub; // sync
    ros::Subscriber sync_obj_label_sub; // sync
    ros::Subscriber sync_obj_pose_sub; // sync
    // Publisher
    ros::Publisher time_monitor_pub;

    ros::Time get_walltime_now() {
        ros::WallTime non_sim_current_time = ros::WallTime::now();
        ros::Time casted_time;
        casted_time.sec = non_sim_current_time.sec;
        casted_time.nsec = non_sim_current_time.nsec;
        return casted_time;
    }


    double ros_time2msec(ros::Time time) {
        return (double)time.sec*1000L + (double)time.nsec/1000000L;
    }
    double time_diff(ros::Time sensor_time, ros::Time execution_time) {
        if (execution_time.sec == 0 && execution_time.nsec == 0) { // not find
            ROS_ERROR("error:execution time is 0");
            return 0.0;
        }
        if (sensor_time.sec == 0 && sensor_time.nsec == 0) { // not find
            ROS_ERROR("error:execution time is 0");
            return 0.0;
        }

        double time_diff = ros_time2msec(execution_time) - ros_time2msec(sensor_time);
        if (time_diff >= 0)
            return time_diff;
        else
            ROS_ERROR("error:time difference is less than 0");
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
//    void obj_pose_callback(const visualization_msgs::MarkerArray::ConstPtr& obj_pose_msg);
    void obj_pose_callback(const std_msgs::Time::ConstPtr& obj_pose_timestamp_msg);
    // sync
    void sync_image_obj_ranged_callback(const cv_tracker::image_obj::ConstPtr& sync_image_obj_msg);
    void sync_image_obj_tracked_callback(const cv_tracker::image_obj_ranged::ConstPtr& sync_image_obj_ranged_msg);
    void sync_obj_label_callback(const cv_tracker::image_obj_tracked::ConstPtr& sync_image_obj_tracked_msg);
    void sync_obj_pose_callback(const cv_tracker::obj_label::ConstPtr& sync_obj_label_msg);
    // time difference
    void time_diff_callback(const synchronization::time_diff::ConstPtr& time_diff_msg);
    void run();
};

TimeManager::TimeManager(int buffer_size) {
    ros::NodeHandle private_nh("~");

    time_monitor_pub = nh.advertise<synchronization::time_monitor> ("/times", 1);
    image_raw_sub = nh.subscribe("/sync_drivers/image_raw", 10, &TimeManager::image_raw_callback, this);
    points_raw_sub = nh.subscribe("/sync_drivers/points_raw", 10, &TimeManager::points_raw_callback, this);
    vscan_points_sub = nh.subscribe("/vscan_points", 10, &TimeManager::vscan_points_callback, this);
    vscan_image_sub = nh.subscribe("/vscan_image", 10, &TimeManager::vscan_image_callback, this);
    image_obj_sub = nh.subscribe("/image_obj", 10, &TimeManager::image_obj_callback, this);
    image_obj_ranged_sub = nh.subscribe("/image_obj_ranged", 10, &TimeManager::image_obj_ranged_callback, this);
    image_obj_tracked_sub = nh.subscribe("/image_obj_tracked", 10, &TimeManager::image_obj_tracked_callback, this);
    current_pose_sub = nh.subscribe("/current_pose", 10, &TimeManager::current_pose_callback, this);
    obj_label_sub = nh.subscribe("/obj_label", 10, &TimeManager::obj_label_callback, this);
    cluster_centroids_sub = nh.subscribe("/cluster_centroids", 10, &TimeManager::cluster_centroids_callback, this);
    obj_pose_sub = nh.subscribe("/obj_pose_timestamp", 10, &TimeManager::obj_pose_callback, this);
//    obj_pose_sub = nh.subscribe("/obj_car/obj_pose", 10, &TimeManager::obj_pose_callback, this);
    // sync
    sync_image_obj_ranged_sub = nh.subscribe("/sync_ranging/image_obj", 10 , &TimeManager::sync_image_obj_ranged_callback, this);
    sync_image_obj_tracked_sub = nh.subscribe("/sync_tracking/image_obj_ranged", 10, &TimeManager::sync_image_obj_tracked_callback, this);
    sync_obj_label_sub = nh.subscribe("/sync_reprojection/image_obj_tracked", 10, &TimeManager::sync_obj_label_callback, this);
    sync_obj_pose_sub = nh.subscribe("/sync_obj_fusion/obj_label", 10, &TimeManager::sync_obj_pose_callback, this);
    // time difference
    time_diff_sub = nh.subscribe("/time_difference", 10, &TimeManager::time_diff_callback, this);

    image_raw_.resize(buffer_size);
    points_raw_.resize(buffer_size);
    vscan_points_.resize(buffer_size);
    vscan_image_.resize(buffer_size);
    image_obj_.resize(buffer_size);
    image_obj_ranged_.resize(buffer_size);
    image_obj_tracked_.resize(buffer_size);
    current_pose_.resize(buffer_size);
    obj_label_.resize(buffer_size);
    obj_pose_.resize(buffer_size);
    cluster_centroids_.resize(buffer_size);
    sync_image_obj_ranged_.resize(buffer_size);
    sync_image_obj_tracked_.resize(buffer_size);
    sync_obj_label_.resize(buffer_size);
    sync_obj_pose_.resize(buffer_size);
    time_diff_.resize(buffer_size);
}

void TimeManager::image_raw_callback(const sensor_msgs::Image::ConstPtr& image_raw_msg) {
//    ROS_INFO("image_raw: \t\t\t%d.%d", image_raw_msg->header.stamp.sec, image_raw_msg->header.stamp.nsec);
    image_raw_.push_front(image_raw_msg->header.stamp, get_walltime_now());
}

void TimeManager::points_raw_callback(const sensor_msgs::PointCloud2::ConstPtr& points_raw_msg) {
//    ROS_INFO("points_raw: \t\t\t%d.%d", points_raw_msg->header.stamp.sec, points_raw_msg->header.stamp.nsec);
    points_raw_.push_front(points_raw_msg->header.stamp, get_walltime_now());
}

void TimeManager::vscan_points_callback(const sensor_msgs::PointCloud2::ConstPtr& vscan_points_msg) {
//    ROS_INFO("vscan_points: \t\t\t%d.%d", vscan_points_msg->header.stamp.sec, vscan_points_msg->header.stamp.nsec);
    vscan_points_.push_front(vscan_points_msg->header.stamp, get_walltime_now());
}

void TimeManager::vscan_image_callback(const points2image::PointsImage::ConstPtr& vscan_image_msg) {
//    ROS_INFO("vscan_image: \t\t\t%d.%d", vscan_image_msg->header.stamp.sec, vscan_image_msg->header.stamp.nsec);
    vscan_image_.push_front(vscan_image_msg->header.stamp, get_walltime_now());
}

void TimeManager::image_obj_callback(const cv_tracker::image_obj::ConstPtr& image_obj_msg) {
//    ROS_INFO("image_obj: \t\t\t%d.%d", image_obj_msg->header.stamp.sec, image_obj_msg->header.stamp.nsec);
    image_obj_.push_front(image_obj_msg->header.stamp, get_walltime_now());
}

void TimeManager::image_obj_ranged_callback(const cv_tracker::image_obj_ranged::ConstPtr& image_obj_ranged_msg) {
//    ROS_INFO("image_obj_ranged: \t\t%d.%d", image_obj_ranged_msg->header.stamp.sec, image_obj_ranged_msg->header.stamp.nsec);
    image_obj_ranged_.push_front(image_obj_ranged_msg->header.stamp, get_walltime_now());
}

void TimeManager::image_obj_tracked_callback(const cv_tracker::image_obj_tracked::ConstPtr& image_obj_tracked_msg) {
//    ROS_INFO("image_obj_tracked: \t\t%d.%d", image_obj_tracked_msg->header.stamp.sec, image_obj_tracked_msg->header.stamp.nsec);
    image_obj_tracked_.push_front(image_obj_tracked_msg->header.stamp, get_walltime_now());
}

void TimeManager::current_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& current_pose_msg) {
//    ROS_INFO("current_pose: \t\t\t%d.%d", current_pose_msg->header.stamp.sec, current_pose_msg->header.stamp.nsec);
    current_pose_.push_front(current_pose_msg->header.stamp, get_walltime_now());
}

void TimeManager::obj_label_callback(const cv_tracker::obj_label::ConstPtr& obj_label_msg) {
//    ROS_INFO("obj_label: \t\t\t%d.%d", obj_label_msg->header.stamp.sec, obj_label_msg->header.stamp.nsec);
    obj_label_.push_front(obj_label_msg->header.stamp, get_walltime_now());
}

void TimeManager::cluster_centroids_callback(const lidar_tracker::centroids::ConstPtr& cluster_centroids_msg) {
//    ROS_INFO("cluster_centroids: \t\t%d.%d", cluster_centroids_msg->header.stamp.sec, cluster_centroids_msg->header.stamp.nsec);
    cluster_centroids_.push_front(cluster_centroids_msg->header.stamp, get_walltime_now());
}

/* sync */
void TimeManager::sync_image_obj_ranged_callback(const cv_tracker::image_obj::ConstPtr& sync_image_obj_msg) {
//    ROS_INFO("sync_image_obj_ranged: \t\t%d.%d", sync_image_obj_msg->header.stamp.sec, sync_image_obj_msg->header.stamp.nsec);
    sync_image_obj_ranged_.push_front(sync_image_obj_msg->header.stamp, get_walltime_now());
}

void TimeManager::sync_image_obj_tracked_callback(const cv_tracker::image_obj_ranged::ConstPtr& sync_image_obj_ranged_msg) {
//    ROS_INFO("sync_image_obj_tracked: \t%d.%d", sync_image_obj_ranged_msg->header.stamp.sec, sync_image_obj_ranged_msg->header.stamp.nsec);
    sync_image_obj_tracked_.push_front(sync_image_obj_ranged_msg->header.stamp, get_walltime_now());
}

void TimeManager::sync_obj_label_callback(const cv_tracker::image_obj_tracked::ConstPtr& sync_image_obj_tracked_msg) {
//    ROS_INFO("sync_obj_label: \t\t%d.%d", sync_image_obj_tracked_msg->header.stamp.sec, sync_image_obj_tracked_msg->header.stamp.nsec);
    sync_obj_label_.push_front(sync_image_obj_tracked_msg->header.stamp, get_walltime_now());
}

void TimeManager::sync_obj_pose_callback(const cv_tracker::obj_label::ConstPtr& sync_obj_label_msg) {
//    ROS_INFO("sync_obj_pose: \t\t\t%d.%d", sync_obj_label_msg->header.stamp.sec, sync_obj_label_msg->header.stamp.nsec);
    sync_obj_pose_.push_front(sync_obj_label_msg->header.stamp, get_walltime_now());
}

/* time difference */
void TimeManager::time_diff_callback(const synchronization::time_diff::ConstPtr& time_diff_msg) {
    ros::Time sensors_time_diff;
    double lidar = (double)time_diff_msg->lidar.sec + (double)time_diff_msg->lidar.nsec/1000000000.0L;
    double camera = (double)time_diff_msg->camera.sec + (double)time_diff_msg->camera.nsec/1000000000.0L;

    if (time_diff_msg->lidar.sec > time_diff_msg->camera.sec) {
        sensors_time_diff.sec = time_diff_msg->lidar.sec - time_diff_msg->camera.sec;
        if (time_diff_msg->lidar.nsec >= time_diff_msg->camera.nsec) {
            sensors_time_diff.nsec = time_diff_msg->lidar.nsec - time_diff_msg->camera.nsec;
        } else {
            sensors_time_diff.sec -= 1;
            sensors_time_diff.nsec = 1000000000L + time_diff_msg->lidar.nsec - time_diff_msg->camera.nsec;
        }
    } else if  (time_diff_msg->lidar.sec < time_diff_msg->camera.sec) {
        sensors_time_diff.sec = time_diff_msg->camera.sec - time_diff_msg->lidar.sec;
        if (time_diff_msg->camera.nsec >= time_diff_msg->lidar.nsec) {
            sensors_time_diff.nsec = time_diff_msg->camera.nsec - time_diff_msg->lidar.nsec;
        } else {
            sensors_time_diff.sec -= 1;
            sensors_time_diff.nsec = 1000000000L + time_diff_msg->camera.nsec - time_diff_msg->lidar.nsec;
        }
    } else if (time_diff_msg->lidar.sec == time_diff_msg->camera.sec) {
        sensors_time_diff.sec = 0;
        if (time_diff_msg->lidar.nsec >= time_diff_msg->camera.nsec) {
            sensors_time_diff.nsec = time_diff_msg->lidar.nsec - time_diff_msg->camera.nsec;
        } else {
            sensors_time_diff.nsec = time_diff_msg->camera.nsec - time_diff_msg->lidar.nsec;
        }
    } else {
        // not impl
        ROS_ERROR("Exception error");
    }
    time_diff_.push_front(time_diff_msg->header.stamp, sensors_time_diff);
}

void TimeManager::obj_pose_callback(const std_msgs::Time::ConstPtr& obj_pose_timestamp_msg) {
//    ROS_INFO("obj_pose: \t\t\t%d.%d", obj_pose_timestamp_msg->data.sec, obj_pose_timestamp_msg->data.nsec);
    obj_pose_.push_front(obj_pose_timestamp_msg->data, get_walltime_now());
    static ros::Time pre_sensor_time;

    synchronization::time_monitor time_monitor_msg;
    time_monitor_msg.header.frame_id = "0";
    time_monitor_msg.header.stamp = ros::Time::now();

//    ROS_INFO("-------------------------------------");
//    ROS_INFO("image_raw");
    time_monitor_msg.image_raw = time_diff(obj_pose_timestamp_msg->data,
                                           image_raw_.find(obj_pose_timestamp_msg->data));
//    ROS_INFO("points_raw");
    time_monitor_msg.points_raw = time_diff(obj_pose_timestamp_msg->data,
                                            points_raw_.find(obj_pose_timestamp_msg->data));
//    ROS_INFO("vscan_points");
    time_monitor_msg.vscan_points = time_diff(points_raw_.find(obj_pose_timestamp_msg->data),
                                              vscan_points_.find(obj_pose_timestamp_msg->data));
//    ROS_INFO("vscan_image");
    time_monitor_msg.vscan_image = time_diff(vscan_points_.find(obj_pose_timestamp_msg->data),
                                             vscan_image_.find(obj_pose_timestamp_msg->data));
//    ROS_INFO("image_obj");
    time_monitor_msg.image_obj = time_diff(image_raw_.find(obj_pose_timestamp_msg->data),
                                           image_obj_.find(obj_pose_timestamp_msg->data));
//    ROS_INFO("image_obj_ranged");
    time_monitor_msg.image_obj_ranged = time_diff(sync_image_obj_ranged_.find(obj_pose_timestamp_msg->data),
                                                  image_obj_ranged_.find(obj_pose_timestamp_msg->data));
//    ROS_INFO("image_obj_tracked");
    time_monitor_msg.image_obj_tracked = time_diff(sync_image_obj_tracked_.find(obj_pose_timestamp_msg->data),
                                                   image_obj_tracked_.find(obj_pose_timestamp_msg->data));
//    ROS_INFO("current_pose");
    time_monitor_msg.current_pose = time_diff(points_raw_.find(obj_pose_timestamp_msg->data),
                                              current_pose_.find(obj_pose_timestamp_msg->data));
//    ROS_INFO("obj_label");
    time_monitor_msg.obj_label = time_diff(sync_obj_label_.find(obj_pose_timestamp_msg->data),
                                           obj_label_.find(obj_pose_timestamp_msg->data));
//    ROS_INFO("cluster_centroids");
    time_monitor_msg.cluster_centroids = time_diff(points_raw_.find(obj_pose_timestamp_msg->data),
                                                   cluster_centroids_.find(obj_pose_timestamp_msg->data));
//    ROS_INFO("obj_pose");
    time_monitor_msg.obj_pose = time_diff(sync_obj_pose_.find(obj_pose_timestamp_msg->data),
                                          obj_pose_.find(obj_pose_timestamp_msg->data));
//    ROS_INFO("-------------------------------------");

    if (ros::Time::isSimTime()) {
        time_monitor_msg.execution_time = time_diff(points_raw_.find(obj_pose_timestamp_msg->data), obj_pose_.find(obj_pose_timestamp_msg->data));
    } else {
        time_monitor_msg.execution_time = time_diff(obj_pose_timestamp_msg->data, obj_pose_.find(obj_pose_timestamp_msg->data));
    }

    time_monitor_msg.cycle_time = time_diff(pre_sensor_time, obj_pose_timestamp_msg->data); // cycle time
    time_monitor_msg.time_diff = ros_time2msec(time_diff_.find(obj_pose_timestamp_msg->data)); // time difference
    time_monitor_pub.publish(time_monitor_msg);

    pre_sensor_time = obj_pose_timestamp_msg->data;
}

void TimeManager::run() {
    ros::spin();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "time_monitor");

  TimeManager time_manager(64);
  time_manager.run();

  return 0;
}
