/*
 *  Copyright (c) 2017, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *this
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
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************
 *  v1.0: amc-nu (abrahammonrroy@yahoo.com)
*/
#include <iostream>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <velodyne_pointcloud/point_types.h>

#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_listener.h>

class CloudTransformerNode {
private:
  ros::NodeHandle node_handle_;
  ros::Subscriber points_node_sub_;
  ros::Publisher transformed_points_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string input_point_topic_;
  std::string target_frame_;
  std::string output_point_topic_;

  bool transform_ok_;

  void
  publish_cloud(const ros::Publisher &in_publisher,
                const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr
                    &in_cloud_msg) {
    in_publisher.publish(in_cloud_msg);
  }

  void transformAsMatrix(const geometry_msgs::TransformStamped &tf,
                         Eigen::Matrix4f &out_mat) {
    double mv[12];
    tf2::Stamped<tf2::Transform> bt;
    tf2::fromMsg(tf, bt);
    bt.getBasis().getOpenGLSubMatrix(mv);
    tf2::Vector3 origin = bt.getOrigin();
    out_mat(0, 0) = mv[0];
    out_mat(0, 1) = mv[4];
    out_mat(0, 2) = mv[8];
    out_mat(1, 0) = mv[1];
    out_mat(1, 1) = mv[5];
    out_mat(1, 2) = mv[9];
    out_mat(2, 0) = mv[2];
    out_mat(2, 1) = mv[6];
    out_mat(2, 2) = mv[10];

    out_mat(3, 0) = out_mat(3, 1) = out_mat(3, 2) = 0;
    out_mat(3, 3) = 1;
    out_mat(0, 3) = origin.x();
    out_mat(1, 3) = origin.y();
    out_mat(2, 3) = origin.z();
  }

  void transformXYZIRCloud(
      const pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &in_cloud,
      pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &out_cloud,
      const geometry_msgs::TransformStamped &in_tf_stamped_transform) {
    Eigen::Matrix4f transform;
    transformAsMatrix(in_tf_stamped_transform, transform);

    if (&in_cloud != &out_cloud) {
      out_cloud.header = in_cloud.header;
      out_cloud.is_dense = in_cloud.is_dense;
      out_cloud.width = in_cloud.width;
      out_cloud.height = in_cloud.height;
      out_cloud.points.reserve(out_cloud.points.size());
      out_cloud.points.assign(in_cloud.points.begin(), in_cloud.points.end());
      out_cloud.sensor_orientation_ = in_cloud.sensor_orientation_;
      out_cloud.sensor_origin_ = in_cloud.sensor_origin_;
    }
    if (in_cloud.is_dense) {
      for (size_t i = 0; i < out_cloud.points.size(); ++i) {
        // out_cloud.points[i].getVector3fMap () = transform *
        // in_cloud.points[i].getVector3fMap ();
        Eigen::Matrix<float, 3, 1> pt(in_cloud[i].x, in_cloud[i].y,
                                      in_cloud[i].z);
        out_cloud[i].x = static_cast<float>(transform(0, 0) * pt.coeffRef(0) +
                                            transform(0, 1) * pt.coeffRef(1) +
                                            transform(0, 2) * pt.coeffRef(2) +
                                            transform(0, 3));
        out_cloud[i].y = static_cast<float>(transform(1, 0) * pt.coeffRef(0) +
                                            transform(1, 1) * pt.coeffRef(1) +
                                            transform(1, 2) * pt.coeffRef(2) +
                                            transform(1, 3));
        out_cloud[i].z = static_cast<float>(transform(2, 0) * pt.coeffRef(0) +
                                            transform(2, 1) * pt.coeffRef(1) +
                                            transform(2, 2) * pt.coeffRef(2) +
                                            transform(2, 3));
      }
    } else {
      // Dataset might contain NaNs and Infs, so check for them first,
      for (size_t i = 0; i < out_cloud.points.size(); ++i) {
        if (!pcl_isfinite(in_cloud.points[i].x) ||
            !pcl_isfinite(in_cloud.points[i].y) ||
            !pcl_isfinite(in_cloud.points[i].z)) {
          continue;
        }
        // out_cloud.points[i].getVector3fMap () = transform *
        // in_cloud.points[i].getVector3fMap ();
        Eigen::Matrix<float, 3, 1> pt(in_cloud[i].x, in_cloud[i].y,
                                      in_cloud[i].z);
        out_cloud[i].x = static_cast<float>(transform(0, 0) * pt.coeffRef(0) +
                                            transform(0, 1) * pt.coeffRef(1) +
                                            transform(0, 2) * pt.coeffRef(2) +
                                            transform(0, 3));
        out_cloud[i].y = static_cast<float>(transform(1, 0) * pt.coeffRef(0) +
                                            transform(1, 1) * pt.coeffRef(1) +
                                            transform(1, 2) * pt.coeffRef(2) +
                                            transform(1, 3));
        out_cloud[i].z = static_cast<float>(transform(2, 0) * pt.coeffRef(0) +
                                            transform(2, 1) * pt.coeffRef(1) +
                                            transform(2, 2) * pt.coeffRef(2) +
                                            transform(2, 3));
      }
    }
  }

  void
  CloudCallback(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr
                    &in_sensor_cloud) {
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr transformed_cloud_ptr(
        new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);

    bool do_transform = false;
    geometry_msgs::TransformStamped transform_stamped;
    if (target_frame_ != in_sensor_cloud->header.frame_id) {
      try {
        transform_stamped = tf_buffer_.lookupTransform(
            target_frame_, in_sensor_cloud->header.frame_id, ros::Time(0),
            ros::Duration(1.0));
        do_transform = true;
      } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        do_transform = false;
        transform_ok_ = false;
        return;
      }
    }
    if (do_transform) {
      transformXYZIRCloud(*in_sensor_cloud, *transformed_cloud_ptr,
                          transform_stamped);
      transformed_cloud_ptr->header.frame_id = target_frame_;
      if (!transform_ok_) {
        ROS_INFO("cloud_transformer: Correctly Transformed");
        transform_ok_ = true;
      }
    } else {
      pcl::copyPointCloud(*in_sensor_cloud, *transformed_cloud_ptr);
    }

    publish_cloud(transformed_points_pub_, transformed_cloud_ptr);
  }

public:
  CloudTransformerNode() : node_handle_("~"), tf_listener_(tf_buffer_) {
    transform_ok_ = false;
  }
  void Run() {
    ROS_INFO("Initializing Cloud Transformer, please wait...");
    node_handle_.param<std::string>("input_point_topic", input_point_topic_,
                                    "/points_raw");
    ROS_INFO("Input point_topic: %s", input_point_topic_.c_str());

    node_handle_.param<std::string>("target_frame", target_frame_, "velodyne");
    ROS_INFO("Target Frame in TF (target_frame) : %s", target_frame_.c_str());

    node_handle_.param<std::string>("output_point_topic", output_point_topic_,
                                    "/points_transformed");
    ROS_INFO("output_point_topic: %s", output_point_topic_.c_str());

    ROS_INFO("Subscribing to... %s", input_point_topic_.c_str());
    points_node_sub_ = node_handle_.subscribe(
        input_point_topic_, 1, &CloudTransformerNode::CloudCallback, this);

    transformed_points_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>(
        output_point_topic_, 2);

    ROS_INFO("Ready");

    ros::spin();
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "cloud_transformer");
  CloudTransformerNode app;
  app.Run();
  return 0;
}
