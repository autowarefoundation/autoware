/*
 * Copyright 2018 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************
 *  v1.0: amc-nu (abrahammonrroy@yahoo.com)
 *
 * roi_object_filter.cpp
 *
 *  Created on: October, 23rd, 2018
 */

#include "roi_object_filter/roi_object_filter.h"

void
RosRoiObjectFilterApp::SyncedDetectionsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr &in_detections,
                                                const grid_map_msgs::GridMap::ConstPtr &in_gridmap)
{
  if (nullptr == in_detections
      || nullptr == in_gridmap)
  {
    ROS_INFO("[%s] Empty input messages, for details check the topics : %s and %s.", __APP_NAME__,
             wayarea_gridmap_topic_.c_str(), objects_src_topic_.c_str());
    return;
  }

  grid_map::GridMap current_grid;
  grid_map::GridMapRosConverter::fromMessage(*in_gridmap, current_grid);
  autoware_msgs::DetectedObjectArray in_roi_objects;
  autoware_msgs::DetectedObjectArray out_roi_objects;

  in_roi_objects.header = in_detections->header;
  out_roi_objects.header = in_detections->header;

  if (current_grid.exists(gridmap_layer_))
  {
    // check if centroids are inside the drivable area
    cv::Mat grid_image;
    grid_map::GridMapCvConverter::toImage<unsigned char, 1>(current_grid, gridmap_layer_, CV_8UC1,
                                                            GRID_MIN_VALUE, GRID_MAX_VALUE, grid_image);

    #pragma omp for
    for (unsigned int i = 0; i < in_detections->objects.size(); i++)
    {
      geometry_msgs::Point original_centroid_point, final_centroid_point;
      original_centroid_point = in_detections->objects[i].pose.position;

      if (current_grid.getFrameId() != in_detections->header.frame_id)
      {
        tf::StampedTransform grid_sensor_tf = FindTransform(current_grid.getFrameId(), in_detections->header.frame_id);
        final_centroid_point = TransformPoint(original_centroid_point, grid_sensor_tf);
      }
      else
      {
        final_centroid_point = original_centroid_point;
      }

      bool point_in_grid = CheckPointInGrid(current_grid, grid_image, final_centroid_point);

      std::vector<std::string>::const_iterator found = std::find(exception_list_.begin(), exception_list_.end(),
                                                                 in_detections->objects[i].label);
      bool is_exception = (found != exception_list_.end());

      if (is_exception || point_in_grid)
      {
        in_roi_objects.objects.push_back(in_detections->objects[i]);
      }
      else
      {
        out_roi_objects.objects.push_back(in_detections->objects[i]);
      }
    }
  }
  else
  {
    ROS_INFO("[%s] %s layer not contained in the OccupancyGrid", __APP_NAME__, gridmap_layer_.c_str());
  }

  roi_objects_publisher_.publish(in_roi_objects);

  wayarea_gridmap_ = nullptr;
  object_detections_ = nullptr;
}

tf::StampedTransform
RosRoiObjectFilterApp::FindTransform(const std::string &in_target_frame, const std::string &in_source_frame)
{
  tf::StampedTransform transform;

  try
  {
    transform_listener_->lookupTransform(in_target_frame, in_source_frame, ros::Time(0), transform);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    return transform;
  }

  return transform;
}

geometry_msgs::Point
RosRoiObjectFilterApp::TransformPoint(const geometry_msgs::Point &in_point, const tf::Transform &in_tf)
{
  tf::Point tf_point;
  tf::pointMsgToTF(in_point, tf_point);

  tf_point = in_tf * tf_point;

  geometry_msgs::Point ros_point;
  tf::pointTFToMsg(tf_point, ros_point);

  return ros_point;
}

void
RosRoiObjectFilterApp::DetectionsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr &in_detections)
{
  if (!processing_ && !in_detections->objects.empty())
  {
    processing_ = true;
    object_detections_ = in_detections;
    SyncedDetectionsCallback(in_detections, wayarea_gridmap_);
    processing_ = false;
  }
}

void RosRoiObjectFilterApp::WayAreaGridMapCallback(const grid_map_msgs::GridMap::ConstPtr &in_message)
{
  if (!processing_)
  {
    processing_ = true;
    wayarea_gridmap_ = in_message;
    processing_ = false;
  }
}


bool
RosRoiObjectFilterApp::CheckPointInGrid(const grid_map::GridMap &in_grid_map,
                                        const cv::Mat &in_grid_image,
                                        const geometry_msgs::Point &in_point)
{
  // calculate out_grid_map position
  grid_map::Position map_pos = in_grid_map.getPosition();
  double origin_x_offset = in_grid_map.getLength().x() / 2.0 - map_pos.x();
  double origin_y_offset = in_grid_map.getLength().y() / 2.0 - map_pos.y();
  // coordinate conversion for cv image
  double cv_x = (in_grid_map.getLength().y() - origin_y_offset - in_point.y) / in_grid_map.getResolution();
  double cv_y = (in_grid_map.getLength().x() - origin_x_offset - in_point.x) / in_grid_map.getResolution();

  // check coords are inside the gridmap
  if (cv_x < 0 || cv_x > in_grid_image.cols || cv_y < 0 || cv_y > in_grid_image.rows)
  {
    return false;
  }

  //_gridmap_no_road_value if road
  if (in_grid_image.at<uchar>(cv_y, cv_x) != gridmap_no_road_value_)
  {
    return true;
  }

  return false;
}


void
RosRoiObjectFilterApp::InitializeRosIo(ros::NodeHandle &in_private_handle)
{
  std::string exception_list;

  ros_namespace_ = ros::this_node::getNamespace();

  if (ros_namespace_.substr(0, 2) == "//")
  {
    ros_namespace_.erase(ros_namespace_.begin());
  }

  objects_src_topic_ = ros_namespace_ + "/objects";
  wayarea_gridmap_topic_= "/grid_map_wayarea";
  std::string roi_topic_str = ros_namespace_+ "/objects_filtered";

  ROS_INFO("[%s] objects_src_topic: %s", __APP_NAME__, objects_src_topic_.c_str());
  ROS_INFO("[%s] wayarea_gridmap_topic: %s", __APP_NAME__, wayarea_gridmap_topic_.c_str());

  in_private_handle.param<bool>("sync_topics", sync_topics_, false);
  ROS_INFO("[%s] sync_topics: %d", __APP_NAME__, sync_topics_);

  in_private_handle.param<std::string>("wayarea_gridmap_layer", gridmap_layer_, "wayarea");
  ROS_INFO("[%s] wayarea_gridmap_layer: %s", __APP_NAME__, gridmap_layer_.c_str());

  in_private_handle.param<int>("wayarea_no_road_value", gridmap_no_road_value_, GRID_MAX_VALUE);
  ROS_INFO("[%s] wayarea_no_road_value: %ds", __APP_NAME__, gridmap_no_road_value_);

  in_private_handle.param<std::string>("exception_list", exception_list, "[person, bicycle]");
  ROS_INFO("[%s] exception_list: %s", __APP_NAME__, exception_list.c_str());

  YAML::Node exception_list_yaml;
  try
  {
    exception_list_yaml = YAML::Load(exception_list);
    for (const auto &exception : exception_list_yaml)
    {
      exception_list_.push_back(exception.as<std::string>());
    }
  }
  catch (const std::exception &e)
  {
    ROS_INFO("[%s] Incorrect format for Exception List (exception_list) param, using default.", __APP_NAME__);
    exception_list_ = {"person", "bycicle"};
  }

  if (!sync_topics_)
  {
    detections_range_subscriber_ = in_private_handle.subscribe(objects_src_topic_,
                                                               1,
                                                               &RosRoiObjectFilterApp::DetectionsCallback, this);

    wayarea_gridmap_subscriber_ = in_private_handle.subscribe(wayarea_gridmap_topic_,
                                                              1,
                                                              &RosRoiObjectFilterApp::WayAreaGridMapCallback,
                                                              this);
  }
  else
  {
    detections_filter_subscriber_ = new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>(node_handle_,
                                                                                                        objects_src_topic_,
                                                                                                        1);
    gridmap_filter_subscriber_ = new message_filters::Subscriber<grid_map_msgs::GridMap>(node_handle_,
                                                                                         wayarea_gridmap_topic_, 1);
    detections_synchronizer_ =
      new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(10),
                                                     *detections_filter_subscriber_,
                                                     *gridmap_filter_subscriber_);
    detections_synchronizer_->registerCallback(boost::bind(&RosRoiObjectFilterApp::SyncedDetectionsCallback,
                                                           this,
                                                           _1, _2));
  }

  roi_objects_publisher_ = node_handle_.advertise<autoware_msgs::DetectedObjectArray>(roi_topic_str, 1);
  ROS_INFO("[%s] Publishing filtered objects in %s", __APP_NAME__, roi_topic_str.c_str());
}


void
RosRoiObjectFilterApp::Run()
{
  ros::NodeHandle private_node_handle("~");
  tf::TransformListener transform_listener;

  transform_listener_ = &transform_listener;

  InitializeRosIo(private_node_handle);

  ROS_INFO("[%s] Ready. Waiting for data...", __APP_NAME__);

  ros::spin();

  ROS_INFO("[%s] END", __APP_NAME__);
}

RosRoiObjectFilterApp::RosRoiObjectFilterApp()
{
  gridmap_ready_ = false;
  detections_ready_ = false;
  processing_ = false;
}
