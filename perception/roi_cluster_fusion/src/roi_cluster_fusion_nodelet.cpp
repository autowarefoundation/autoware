// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "roi_cluster_fusion/roi_cluster_fusion_nodelet.hpp"

#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <algorithm>
#include <chrono>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace roi_cluster_fusion
{
Debugger::Debugger(rclcpp::Node * node, const int camera_num) : node_(node)
{
  image_buffers_.resize(camera_num);
  for (int id = 0; id < camera_num; ++id) {
    auto sub = image_transport::create_subscription(
      node, "input/image_raw" + std::to_string(id),
      boost::bind(&Debugger::imageCallback, this, _1, id), "raw");
    image_subs_.push_back(sub);
    if (node->has_parameter("format")) {
      node->undeclare_parameter("format");
    }
    if (node->has_parameter("jpeg_quality")) {
      node->undeclare_parameter("jpeg_quality");
    }
    if (node->has_parameter("png_level")) {
      node->undeclare_parameter("png_level");
    }
    auto pub = image_transport::create_publisher(node, "output/image_raw" + std::to_string(id));
    image_pubs_.push_back(pub);
    image_buffers_.at(id).set_capacity(5);
  }
}

void Debugger::imageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & input_image_msg, const int id)
{
  image_buffers_.at(id).push_front(input_image_msg);
}

void Debugger::showImage(
  const int id, const rclcpp::Time & time,
  const std::vector<sensor_msgs::msg::RegionOfInterest> & image_rois,
  const std::vector<sensor_msgs::msg::RegionOfInterest> & pointcloud_rois,
  const std::vector<Eigen::Vector2d> & points)
{
  const boost::circular_buffer<sensor_msgs::msg::Image::ConstSharedPtr> & image_buffer =
    image_buffers_.at(id);
  const image_transport::Publisher & image_pub = image_pubs_.at(id);
  for (size_t i = 0; i < image_buffer.size(); ++i) {
    if (image_buffer.at(i)->header.stamp == time) {
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(image_buffer.at(i), image_buffer.at(i)->encoding);
      for (const auto & point : points) {
        cv::circle(
          cv_ptr->image, cv::Point(static_cast<int>(point.x()), static_cast<int>(point.y())), 2,
          cv::Scalar(255, 255, 255), 3, 4);
      }
      for (const auto & image_roi : image_rois) {
        cv::line(
          cv_ptr->image, cv::Point(image_roi.x_offset, image_roi.y_offset),
          cv::Point(image_roi.x_offset + image_roi.width, image_roi.y_offset),
          cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
        cv::line(
          cv_ptr->image, cv::Point(image_roi.x_offset, image_roi.y_offset),
          cv::Point(image_roi.x_offset, image_roi.y_offset + image_roi.height),
          cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
        cv::line(
          cv_ptr->image, cv::Point(image_roi.x_offset + image_roi.width, image_roi.y_offset),
          cv::Point(image_roi.x_offset + image_roi.width, image_roi.y_offset + image_roi.height),
          cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
        cv::line(
          cv_ptr->image, cv::Point(image_roi.x_offset, image_roi.y_offset + image_roi.height),
          cv::Point(image_roi.x_offset + image_roi.width, image_roi.y_offset + image_roi.height),
          cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
      }
      for (const auto & pointcloud_roi : pointcloud_rois) {
        cv::line(
          cv_ptr->image, cv::Point(pointcloud_roi.x_offset, pointcloud_roi.y_offset),
          cv::Point(pointcloud_roi.x_offset + pointcloud_roi.width, pointcloud_roi.y_offset),
          cv::Scalar(255, 0, 0), 1, cv::LINE_AA);
        cv::line(
          cv_ptr->image, cv::Point(pointcloud_roi.x_offset, pointcloud_roi.y_offset),
          cv::Point(pointcloud_roi.x_offset, pointcloud_roi.y_offset + pointcloud_roi.height),
          cv::Scalar(255, 0, 0), 1, cv::LINE_AA);
        cv::line(
          cv_ptr->image,
          cv::Point(pointcloud_roi.x_offset + pointcloud_roi.width, pointcloud_roi.y_offset),
          cv::Point(
            pointcloud_roi.x_offset + pointcloud_roi.width,
            pointcloud_roi.y_offset + pointcloud_roi.height),
          cv::Scalar(255, 0, 0), 1, cv::LINE_AA);
        cv::line(
          cv_ptr->image,
          cv::Point(pointcloud_roi.x_offset, pointcloud_roi.y_offset + pointcloud_roi.height),
          cv::Point(
            pointcloud_roi.x_offset + pointcloud_roi.width,
            pointcloud_roi.y_offset + pointcloud_roi.height),
          cv::Scalar(255, 0, 0), 1, cv::LINE_AA);
      }
      image_pub.publish(cv_ptr->toImageMsg());
      // cv::imshow("ROI" + std::to_string(id), cv_ptr->image);
      // cv::waitKey(2);

      break;
    }
  }
}

RoiClusterFusionNodelet::RoiClusterFusionNodelet(const rclcpp::NodeOptions & options)
: Node("roi_cluster_fusion_node", options),
  tf_buffer_(this->get_clock()),
  tf_listener_ptr_(tf_buffer_)
{
  use_iou_x_ = declare_parameter("use_iou_x", true);
  use_iou_y_ = declare_parameter("use_iou_y", false);
  use_iou_ = declare_parameter("use_iou", false);
  use_cluster_semantic_type_ = declare_parameter("use_cluster_semantic_type", false);
  iou_threshold_ = declare_parameter("iou_threshold", 0.1);
  int rois_number = declare_parameter("rois_number", 1);
  if (rois_number < 1) {
    RCLCPP_WARN(this->get_logger(), "minimum roi_num is 1. current roi_num is %d", rois_number);
    rois_number = 1;
  }
  if (8 < rois_number) {
    RCLCPP_WARN(this->get_logger(), "maximum roi_num is 8. current roi_num is %d", rois_number);
    rois_number = 8;
  }

  cluster_sub_.subscribe(this, "clusters", rclcpp::QoS{1}.get_rmw_qos_profile());
  for (int id = 0; id < rois_number; ++id) {
    std::function<void(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)> fcn =
      std::bind(&RoiClusterFusionNodelet::cameraInfoCallback, this, std::placeholders::_1, id);
    v_camera_info_sub_.push_back(this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "input/camera_info" + std::to_string(id), rclcpp::QoS{1}.best_effort(), fcn));
  }
  v_roi_sub_.resize(rois_number);
  for (int id = 0; id < static_cast<int>(v_roi_sub_.size()); ++id) {
    v_roi_sub_.at(id) = std::make_shared<
      message_filters::Subscriber<autoware_perception_msgs::msg::DetectedObjectsWithFeature>>(
      this, "input/rois" + std::to_string(id), rclcpp::QoS{1}.get_rmw_qos_profile());
  }
  // add dummy callback to enable passthrough filter
  v_roi_sub_.at(0)->registerCallback(bind(&RoiClusterFusionNodelet::dummyCallback, this, _1));
  switch (rois_number) {
    case 1:
      sync_ptr_ = std::make_shared<Sync>(
        SyncPolicy(10), cluster_sub_, *v_roi_sub_.at(0), passthrough_, passthrough_, passthrough_,
        passthrough_, passthrough_, passthrough_, passthrough_);
      break;
    case 2:
      sync_ptr_ = std::make_shared<Sync>(
        SyncPolicy(10), cluster_sub_, *v_roi_sub_.at(0), *v_roi_sub_.at(1), passthrough_,
        passthrough_, passthrough_, passthrough_, passthrough_, passthrough_);
      break;
    case 3:
      sync_ptr_ = std::make_shared<Sync>(
        SyncPolicy(10), cluster_sub_, *v_roi_sub_.at(0), *v_roi_sub_.at(1), *v_roi_sub_.at(2),
        passthrough_, passthrough_, passthrough_, passthrough_, passthrough_);
      break;
    case 4:
      sync_ptr_ = std::make_shared<Sync>(
        SyncPolicy(10), cluster_sub_, *v_roi_sub_.at(0), *v_roi_sub_.at(1), *v_roi_sub_.at(2),
        *v_roi_sub_.at(3), passthrough_, passthrough_, passthrough_, passthrough_);
      break;
    case 5:
      sync_ptr_ = std::make_shared<Sync>(
        SyncPolicy(10), cluster_sub_, *v_roi_sub_.at(0), *v_roi_sub_.at(1), *v_roi_sub_.at(2),
        *v_roi_sub_.at(3), *v_roi_sub_.at(4), passthrough_, passthrough_, passthrough_);
      break;
    case 6:
      sync_ptr_ = std::make_shared<Sync>(
        SyncPolicy(10), cluster_sub_, *v_roi_sub_.at(0), *v_roi_sub_.at(1), *v_roi_sub_.at(2),
        *v_roi_sub_.at(3), *v_roi_sub_.at(4), *v_roi_sub_.at(5), passthrough_, passthrough_);
      break;
    case 7:
      sync_ptr_ = std::make_shared<Sync>(
        SyncPolicy(10), cluster_sub_, *v_roi_sub_.at(0), *v_roi_sub_.at(1), *v_roi_sub_.at(2),
        *v_roi_sub_.at(3), *v_roi_sub_.at(4), *v_roi_sub_.at(5), *v_roi_sub_.at(6), passthrough_);
      break;
    case 8:
      sync_ptr_ = std::make_shared<Sync>(
        SyncPolicy(10), cluster_sub_, *v_roi_sub_.at(0), *v_roi_sub_.at(1), *v_roi_sub_.at(2),
        *v_roi_sub_.at(3), *v_roi_sub_.at(4), *v_roi_sub_.at(5), *v_roi_sub_.at(6),
        *v_roi_sub_.at(7));
      break;
    default:
      return;
  }
  // sync_ptr_->registerCallback(
  // boost::bind(&RoiClusterFusionNodelet::fusionCallback, this, _1, _2, _3, _4, _5, _6, _7,
  // _8, _9));
  sync_ptr_->registerCallback(std::bind(
    &RoiClusterFusionNodelet::fusionCallback, this, std::placeholders::_1, std::placeholders::_2,
    std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6,
    std::placeholders::_7, std::placeholders::_8, std::placeholders::_9));
  labeled_cluster_pub_ =
    this->create_publisher<autoware_perception_msgs::msg::DetectedObjectsWithFeature>(
      "output/labeled_clusters", rclcpp::QoS{1});

  const bool debug_mode = declare_parameter("debug_mode", false);
  if (debug_mode) {
    debugger_ = std::make_shared<Debugger>(this, rois_number);
  }
}

void RoiClusterFusionNodelet::cameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr input_camera_info_msg, const int id)
{
  m_camera_info_[id] = *input_camera_info_msg;
}

void RoiClusterFusionNodelet::fusionCallback(
  autoware_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_cluster_msg,
  autoware_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_roi0_msg,
  autoware_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_roi1_msg,
  autoware_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_roi2_msg,
  autoware_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_roi3_msg,
  autoware_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_roi4_msg,
  autoware_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_roi5_msg,
  autoware_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_roi6_msg,
  autoware_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_roi7_msg)
{
  // Guard
  if (labeled_cluster_pub_->get_subscription_count() < 1) {
    return;
  }

  // build output msg
  autoware_perception_msgs::msg::DetectedObjectsWithFeature output_msg;
  output_msg = *input_cluster_msg;

  // reset cluster semantic type
  if (!use_cluster_semantic_type_) {
    for (auto & feature_object : output_msg.feature_objects) {
      feature_object.object.classification.front().label =
        autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN;
      feature_object.object.existence_probability = 0.0;
    }
  }

  // check camera info
  for (int id = 0; id < static_cast<int>(v_roi_sub_.size()); ++id) {
    // debug variable
    std::vector<sensor_msgs::msg::RegionOfInterest> debug_image_rois;
    std::vector<sensor_msgs::msg::RegionOfInterest> debug_pointcloud_rois;
    std::vector<Eigen::Vector2d> debug_image_points;
    // cannot find camera info
    if (m_camera_info_.find(id) == m_camera_info_.end()) {
      RCLCPP_WARN(this->get_logger(), "no camera info. id is %d", id);
      continue;
    }

    // projection matrix
    Eigen::Matrix4d projection;
    projection << m_camera_info_.at(id).p.at(0), m_camera_info_.at(id).p.at(1),
      m_camera_info_.at(id).p.at(2), m_camera_info_.at(id).p.at(3), m_camera_info_.at(id).p.at(4),
      m_camera_info_.at(id).p.at(5), m_camera_info_.at(id).p.at(6), m_camera_info_.at(id).p.at(7),
      m_camera_info_.at(id).p.at(8), m_camera_info_.at(id).p.at(9), m_camera_info_.at(id).p.at(10),
      m_camera_info_.at(id).p.at(11);

    // get transform from cluster frame id to camera optical frame id
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_.lookupTransform(
        /*target*/ m_camera_info_.at(id).header.frame_id,
        /*src*/ input_cluster_msg->header.frame_id, tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      return;
    }

    // build cluster roi
    std::map<size_t, sensor_msgs::msg::RegionOfInterest> m_cluster_roi;
    for (size_t i = 0; i < input_cluster_msg->feature_objects.size(); ++i) {
      if (input_cluster_msg->feature_objects.at(i).feature.cluster.data.empty()) {
        continue;
      }

      sensor_msgs::msg::PointCloud2 transformed_cluster;
      tf2::doTransform(
        input_cluster_msg->feature_objects.at(i).feature.cluster, transformed_cluster,
        transform_stamped);

      // for reduce calc cost
      // Eigen::Vector3d centroid_point;
      // centroid_point << 0, 0, 0;
      // for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed_cluster, "x"),
      // iter_y(transformed_cluster,
      // "y"), iter_z(transformed_cluster, "z");
      //      iter_x != iter_x.end();
      //      ++iter_x, ++iter_y, ++iter_z)
      // {
      //     centroid_point += Eigen::Vector3d(*iter_x, *iter_y, *iter_z);
      // }
      // centroid_point *= 1.0 / static_cast<double>(
      //  input_cluster_msg->feature_objects.at(i).feature.cluster.data.size());
      //     if (centroid_point.z() <= 0.0)
      //         continue;

      std::vector<Eigen::Vector2d> projected_points;
      projected_points.reserve(transformed_cluster.data.size());
      int min_x(m_camera_info_.at(id).width), min_y(m_camera_info_.at(id).height), max_x(0),
        max_y(0);
      for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed_cluster, "x"),
           iter_y(transformed_cluster, "y"), iter_z(transformed_cluster, "z");
           iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        if (*iter_z <= 0.0) {
          continue;
        }
        Eigen::Vector4d projected_point =
          projection * Eigen::Vector4d(*iter_x, *iter_y, *iter_z, 1.0);
        Eigen::Vector2d normalized_projected_point = Eigen::Vector2d(
          projected_point.x() / projected_point.z(), projected_point.y() / projected_point.z());
        if (
          0 <= static_cast<int>(normalized_projected_point.x()) &&
          static_cast<int>(normalized_projected_point.x()) <=
            static_cast<int>(m_camera_info_.at(id).width) - 1 &&
          0 <= static_cast<int>(normalized_projected_point.y()) &&
          static_cast<int>(normalized_projected_point.y()) <=
            static_cast<int>(m_camera_info_.at(id).height) - 1) {
          min_x = std::min(static_cast<int>(normalized_projected_point.x()), min_x);
          min_y = std::min(static_cast<int>(normalized_projected_point.y()), min_y);
          max_x = std::max(static_cast<int>(normalized_projected_point.x()), max_x);
          max_y = std::max(static_cast<int>(normalized_projected_point.y()), max_y);
          projected_points.push_back(normalized_projected_point);
          debug_image_points.push_back(normalized_projected_point);
        }
      }
      if (projected_points.empty()) {
        continue;
      }

      sensor_msgs::msg::RegionOfInterest roi;
      // roi.do_rectify = m_camera_info_.at(id).do_rectify;
      roi.x_offset = min_x;
      roi.y_offset = min_y;
      roi.width = max_x - min_x;
      roi.height = max_y - min_y;
      m_cluster_roi.insert(std::make_pair(i, roi));
      debug_pointcloud_rois.push_back(roi);
    }

    // calc iou
    autoware_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_roi_msg;
    if (id == 0) {
      input_roi_msg = input_roi0_msg;
    } else if (id == 1) {
      input_roi_msg = input_roi1_msg;
    } else if (id == 2) {
      input_roi_msg = input_roi2_msg;
    } else if (id == 3) {
      input_roi_msg = input_roi3_msg;
    } else if (id == 4) {
      input_roi_msg = input_roi4_msg;
    } else if (id == 5) {
      input_roi_msg = input_roi5_msg;
    } else if (id == 6) {
      input_roi_msg = input_roi6_msg;
    } else if (id == 7) {
      input_roi_msg = input_roi7_msg;
    }
    for (size_t i = 0; i < input_roi_msg->feature_objects.size(); ++i) {
      int index = 0;
      double max_iou = 0.0;
      for (auto m_cluster_roi_itr = m_cluster_roi.begin(); m_cluster_roi_itr != m_cluster_roi.end();
           ++m_cluster_roi_itr) {
        double iou(0.0), iou_x(0.0), iou_y(0.0);
        if (use_iou_) {
          iou =
            calcIoU(m_cluster_roi_itr->second, input_roi_msg->feature_objects.at(i).feature.roi);
        }
        if (use_iou_x_) {
          iou_x =
            calcIoUX(m_cluster_roi_itr->second, input_roi_msg->feature_objects.at(i).feature.roi);
        }
        if (use_iou_y_) {
          iou_y =
            calcIoUY(m_cluster_roi_itr->second, input_roi_msg->feature_objects.at(i).feature.roi);
        }
        if (max_iou < iou + iou_x + iou_y) {
          index = m_cluster_roi_itr->first;
          max_iou = iou + iou_x + iou_y;
        }
      }
      if (
        iou_threshold_ < max_iou &&
        output_msg.feature_objects.at(index).object.existence_probability <=
          input_roi_msg->feature_objects.at(i).object.existence_probability &&
        input_roi_msg->feature_objects.at(i).object.classification.front().label !=
          autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN) {
        output_msg.feature_objects.at(index).object.classification =
          input_roi_msg->feature_objects.at(i).object.classification;
      }
      debug_image_rois.push_back(input_roi_msg->feature_objects.at(i).feature.roi);
    }
    if (debugger_) {
      debugger_->showImage(
        id, input_roi_msg->header.stamp, debug_image_rois, debug_pointcloud_rois,
        debug_image_points);
    }
  }
  // publish output msg
  labeled_cluster_pub_->publish(output_msg);
}

double RoiClusterFusionNodelet::calcIoU(
  const sensor_msgs::msg::RegionOfInterest & roi_1,
  const sensor_msgs::msg::RegionOfInterest & roi_2)
{
  double s_1, s_2;
  s_1 = static_cast<double>(roi_1.width * static_cast<double>(roi_1.height));
  s_2 = static_cast<double>(roi_2.width * static_cast<double>(roi_2.height));

  double overlap_s;
  double overlap_max_x, overlap_max_y, overlap_min_x, overlap_min_y;
  overlap_min_x = roi_1.x_offset < roi_2.x_offset ? roi_2.x_offset : roi_1.x_offset;
  overlap_min_y = roi_1.y_offset < roi_2.y_offset ? roi_2.y_offset : roi_1.y_offset;
  overlap_max_x = roi_1.x_offset + roi_1.width < roi_2.x_offset + roi_2.width
                    ? roi_1.x_offset + roi_1.width
                    : roi_2.x_offset + roi_2.width;
  overlap_max_y = roi_1.y_offset + roi_1.height < roi_2.y_offset + roi_2.height
                    ? roi_1.y_offset + roi_1.height
                    : roi_2.y_offset + roi_2.height;
  overlap_s = (overlap_max_x - overlap_min_x) * (overlap_max_y - overlap_min_y);
  if (overlap_max_x < overlap_min_x || overlap_max_y < overlap_min_y) {
    return 0.0;
  }
  return overlap_s / (s_1 + s_2 - overlap_s);
}
double RoiClusterFusionNodelet::calcIoUX(
  const sensor_msgs::msg::RegionOfInterest & roi_1,
  const sensor_msgs::msg::RegionOfInterest & roi_2)
{
  double s_1, s_2;
  s_1 = static_cast<double>(roi_1.width);
  s_2 = static_cast<double>(roi_2.width);
  double overlap_s;
  double overlap_max_x, overlap_max_y, overlap_min_x, overlap_min_y;
  overlap_min_x = roi_1.x_offset < roi_2.x_offset ? roi_2.x_offset : roi_1.x_offset;
  overlap_min_y = roi_1.y_offset < roi_2.y_offset ? roi_2.y_offset : roi_1.y_offset;
  overlap_max_x = roi_1.x_offset + roi_1.width < roi_2.x_offset + roi_2.width
                    ? roi_1.x_offset + roi_1.width
                    : roi_2.x_offset + roi_2.width;
  overlap_max_y = roi_1.y_offset + roi_1.height < roi_2.y_offset + roi_2.height
                    ? roi_1.y_offset + roi_1.height
                    : roi_2.y_offset + roi_2.height;
  overlap_s = (overlap_max_x - overlap_min_x);
  if (overlap_max_x < overlap_min_x || overlap_max_y < overlap_min_y) {
    return 0.0;
  }
  return overlap_s / (s_1 + s_2 - overlap_s);
}
double RoiClusterFusionNodelet::calcIoUY(
  const sensor_msgs::msg::RegionOfInterest & roi_1,
  const sensor_msgs::msg::RegionOfInterest & roi_2)
{
  double s_1, s_2;
  s_1 = static_cast<double>(roi_1.height);
  s_2 = static_cast<double>(roi_2.height);
  double overlap_s;
  double overlap_max_x, overlap_max_y, overlap_min_x, overlap_min_y;
  overlap_min_x = roi_1.x_offset < roi_2.x_offset ? roi_2.x_offset : roi_1.x_offset;
  overlap_min_y = roi_1.y_offset < roi_2.y_offset ? roi_2.y_offset : roi_1.y_offset;
  overlap_max_x = roi_1.x_offset + roi_1.width < roi_2.x_offset + roi_2.width
                    ? roi_1.x_offset + roi_1.width
                    : roi_2.x_offset + roi_2.width;
  overlap_max_y = roi_1.y_offset + roi_1.height < roi_2.y_offset + roi_2.height
                    ? roi_1.y_offset + roi_1.height
                    : roi_2.y_offset + roi_2.height;
  overlap_s = (overlap_max_y - overlap_min_y);
  if (overlap_max_x < overlap_min_x || overlap_max_y < overlap_min_y) {
    return 0.0;
  }
  return overlap_s / (s_1 + s_2 - overlap_s);
}
}  // namespace roi_cluster_fusion

RCLCPP_COMPONENTS_REGISTER_NODE(roi_cluster_fusion::RoiClusterFusionNodelet)
