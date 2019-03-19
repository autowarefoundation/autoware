/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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
 * range_vision_fusion_node.cpp
 *
 *  Created on: July, 05th, 2018
 */

#include "range_vision_fusion/range_vision_fusion.h"

cv::Point3f
ROSRangeVisionFusionApp::TransformPoint(const geometry_msgs::Point &in_point, const tf::StampedTransform &in_transform)
{
  tf::Vector3 tf_point(in_point.x, in_point.y, in_point.z);
  tf::Vector3 tf_point_t = in_transform * tf_point;
  return cv::Point3f(tf_point_t.x(), tf_point_t.y(), tf_point_t.z());
}

cv::Point2i
ROSRangeVisionFusionApp::ProjectPoint(const cv::Point3f &in_point)
{
  auto u = int(in_point.x * fx_ / in_point.z + cx_);
  auto v = int(in_point.y * fy_ / in_point.z + cy_);

  return cv::Point2i(u, v);
}

autoware_msgs::DetectedObject
ROSRangeVisionFusionApp::TransformObject(const autoware_msgs::DetectedObject &in_detection,
                                         const tf::StampedTransform &in_transform)
{
  autoware_msgs::DetectedObject t_obj = in_detection;

  tf::Vector3 in_pos(in_detection.pose.position.x,
                     in_detection.pose.position.y,
                     in_detection.pose.position.z);
  tf::Quaternion in_quat(in_detection.pose.orientation.x,
                         in_detection.pose.orientation.y,
                         in_detection.pose.orientation.w,
                         in_detection.pose.orientation.z);

  tf::Vector3 in_pos_t = in_transform * in_pos;
  tf::Quaternion in_quat_t = in_transform * in_quat;

  t_obj.pose.position.x = in_pos_t.x();
  t_obj.pose.position.y = in_pos_t.y();
  t_obj.pose.position.z = in_pos_t.z();

  t_obj.pose.orientation.x = in_quat_t.x();
  t_obj.pose.orientation.y = in_quat_t.y();
  t_obj.pose.orientation.z = in_quat_t.z();
  t_obj.pose.orientation.w = in_quat_t.w();

  return t_obj;
}

bool
ROSRangeVisionFusionApp::IsObjectInImage(const autoware_msgs::DetectedObject &in_detection)
{
  cv::Point3f image_space_point = TransformPoint(in_detection.pose.position, camera_lidar_tf_);

  cv::Point2i image_pixel = ProjectPoint(image_space_point);

  return (image_pixel.x >= 0)
         && (image_pixel.x < image_size_.width)
         && (image_pixel.y >= 0)
         && (image_pixel.y < image_size_.height)
         && (image_space_point.z > 0);
}

cv::Rect ROSRangeVisionFusionApp::ProjectDetectionToRect(const autoware_msgs::DetectedObject &in_detection)
{
  cv::Rect projected_box;

  Eigen::Vector3f pos;
  pos << in_detection.pose.position.x,
    in_detection.pose.position.y,
    in_detection.pose.position.z;

  Eigen::Quaternionf rot(in_detection.pose.orientation.w,
                         in_detection.pose.orientation.x,
                         in_detection.pose.orientation.y,
                         in_detection.pose.orientation.z);

  std::vector<double> dims = {
    in_detection.dimensions.x,
    in_detection.dimensions.y,
    in_detection.dimensions.z
  };

  jsk_recognition_utils::Cube cube(pos, rot, dims);

  Eigen::Affine3f range_vision_tf;
  tf::transformTFToEigen(camera_lidar_tf_, range_vision_tf);
  jsk_recognition_utils::Vertices vertices = cube.transformVertices(range_vision_tf);

  std::vector<cv::Point> polygon;
  for (auto &vertex : vertices)
  {
    cv::Point p = ProjectPoint(cv::Point3f(vertex.x(), vertex.y(), vertex.z()));
    polygon.push_back(p);
  }

  projected_box = cv::boundingRect(polygon);

  return projected_box;
}

void
ROSRangeVisionFusionApp::TransformRangeToVision(const autoware_msgs::DetectedObjectArray::ConstPtr &in_range_detections,
                                                autoware_msgs::DetectedObjectArray &out_in_cv_range_detections,
                                                autoware_msgs::DetectedObjectArray &out_out_cv_range_detections)
{
  out_in_cv_range_detections.header = in_range_detections->header;
  out_in_cv_range_detections.objects.clear();
  out_out_cv_range_detections.header = in_range_detections->header;
  out_out_cv_range_detections.objects.clear();
  for (size_t i = 0; i < in_range_detections->objects.size(); i++)
  {
    if (IsObjectInImage(in_range_detections->objects[i]))
    {
      out_in_cv_range_detections.objects.push_back(in_range_detections->objects[i]);
    } else
    {
      out_out_cv_range_detections.objects.push_back(in_range_detections->objects[i]);
    }
  }
}

void
ROSRangeVisionFusionApp::CalculateObjectFeatures(autoware_msgs::DetectedObject &in_out_object, bool in_estimate_pose)
{

  float min_x = std::numeric_limits<float>::max();
  float max_x = -std::numeric_limits<float>::max();
  float min_y = std::numeric_limits<float>::max();
  float max_y = -std::numeric_limits<float>::max();
  float min_z = std::numeric_limits<float>::max();
  float max_z = -std::numeric_limits<float>::max();
  float average_x = 0, average_y = 0, average_z = 0, length, width, height;
  pcl::PointXYZ centroid, min_point, max_point, average_point;

  std::vector<cv::Point2f> object_2d_points;

  pcl::PointCloud<pcl::PointXYZ> in_cloud;
  pcl::fromROSMsg(in_out_object.pointcloud, in_cloud);

  for (const auto &point : in_cloud.points)
  {
    average_x += point.x;
    average_y += point.y;
    average_z += point.z;
    centroid.x += point.x;
    centroid.y += point.y;
    centroid.z += point.z;

    if (point.x < min_x)
      min_x = point.x;
    if (point.y < min_y)
      min_y = point.y;
    if (point.z < min_z)
      min_z = point.z;
    if (point.x > max_x)
      max_x = point.x;
    if (point.y > max_y)
      max_y = point.y;
    if (point.z > max_z)
      max_z = point.z;

    cv::Point2f pt;
    pt.x = point.x;
    pt.y = point.y;
    object_2d_points.push_back(pt);
  }
  min_point.x = min_x;
  min_point.y = min_y;
  min_point.z = min_z;
  max_point.x = max_x;
  max_point.y = max_y;
  max_point.z = max_z;

  if (in_cloud.points.size() > 0)
  {
    centroid.x /= in_cloud.points.size();
    centroid.y /= in_cloud.points.size();
    centroid.z /= in_cloud.points.size();

    average_x /= in_cloud.points.size();
    average_y /= in_cloud.points.size();
    average_z /= in_cloud.points.size();
  }

  average_point.x = average_x;
  average_point.y = average_y;
  average_point.z = average_z;

  length = max_point.x - min_point.x;
  width = max_point.y - min_point.y;
  height = max_point.z - min_point.z;

  geometry_msgs::PolygonStamped convex_hull;
  std::vector<cv::Point2f> hull_points;
  if (object_2d_points.size() > 0)
    cv::convexHull(object_2d_points, hull_points);

  convex_hull.header = in_out_object.header;
  for (size_t i = 0; i < hull_points.size() + 1; i++)
  {
    geometry_msgs::Point32 point;
    point.x = hull_points[i % hull_points.size()].x;
    point.y = hull_points[i % hull_points.size()].y;
    point.z = min_point.z;
    convex_hull.polygon.points.push_back(point);
  }

  for (size_t i = 0; i < hull_points.size() + 1; i++)
  {
    geometry_msgs::Point32 point;
    point.x = hull_points[i % hull_points.size()].x;
    point.y = hull_points[i % hull_points.size()].y;
    point.z = max_point.z;
    convex_hull.polygon.points.push_back(point);
  }

  double rz = 0;
  if (in_estimate_pose)
  {
    cv::RotatedRect box = cv::minAreaRect(hull_points);
    rz = box.angle * 3.14 / 180;
    in_out_object.pose.position.x = box.center.x;
    in_out_object.pose.position.y = box.center.y;
    in_out_object.dimensions.x = box.size.width;
    in_out_object.dimensions.y = box.size.height;
  }

  in_out_object.convex_hull = convex_hull;

  in_out_object.pose.position.x = min_point.x + length / 2;
  in_out_object.pose.position.y = min_point.y + width / 2;
  in_out_object.pose.position.z = min_point.z + height / 2;

  in_out_object.dimensions.x = ((length < 0) ? -1 * length : length);
  in_out_object.dimensions.y = ((width < 0) ? -1 * width : width);
  in_out_object.dimensions.z = ((height < 0) ? -1 * height : height);

  tf::Quaternion quat = tf::createQuaternionFromRPY(0.0, 0.0, rz);
  tf::quaternionTFToMsg(quat, in_out_object.pose.orientation);
}

autoware_msgs::DetectedObject ROSRangeVisionFusionApp::MergeObjects(const autoware_msgs::DetectedObject &in_object_a,
                                                                    const autoware_msgs::DetectedObject &in_object_b)
{
  autoware_msgs::DetectedObject object_merged;
  object_merged = in_object_b;

  pcl::PointCloud<pcl::PointXYZ> cloud_a, cloud_b, cloud_merged;

  if (!in_object_a.pointcloud.data.empty())
    pcl::fromROSMsg(in_object_a.pointcloud, cloud_a);
  if (!in_object_b.pointcloud.data.empty())
    pcl::fromROSMsg(in_object_b.pointcloud, cloud_b);

  cloud_merged = cloud_a + cloud_b;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud_merged, cloud_msg);
  cloud_msg.header = object_merged.pointcloud.header;

  object_merged.pointcloud = cloud_msg;

  return object_merged;

}

double ROSRangeVisionFusionApp::GetDistanceToObject(const autoware_msgs::DetectedObject &in_object)
{
  return sqrt(in_object.dimensions.x * in_object.dimensions.x +
              in_object.dimensions.y * in_object.dimensions.y +
              in_object.dimensions.z * in_object.dimensions.z);
}

void ROSRangeVisionFusionApp::CheckMinimumDimensions(autoware_msgs::DetectedObject &in_out_object)
{
  if (in_out_object.label == "car")
  {
    if (in_out_object.dimensions.x < car_depth_)
      in_out_object.dimensions.x = car_depth_;
    if (in_out_object.dimensions.y < car_width_)
      in_out_object.dimensions.y = car_width_;
    if (in_out_object.dimensions.z < car_height_)
      in_out_object.dimensions.z = car_height_;
  }
  if (in_out_object.label == "person")
  {
    if (in_out_object.dimensions.x < person_depth_)
      in_out_object.dimensions.x = person_depth_;
    if (in_out_object.dimensions.y < person_width_)
      in_out_object.dimensions.y = person_width_;
    if (in_out_object.dimensions.z < person_height_)
      in_out_object.dimensions.z = person_height_;
  }

  if (in_out_object.label == "truck" || in_out_object.label == "bus")
  {
    if (in_out_object.dimensions.x < truck_depth_)
      in_out_object.dimensions.x = truck_depth_;
    if (in_out_object.dimensions.y < truck_width_)
      in_out_object.dimensions.y = truck_width_;
    if (in_out_object.dimensions.z < truck_height_)
      in_out_object.dimensions.z = truck_height_;
  }
}

autoware_msgs::DetectedObjectArray
ROSRangeVisionFusionApp::FuseRangeVisionDetections(
  const autoware_msgs::DetectedObjectArray::ConstPtr &in_vision_detections,
  const autoware_msgs::DetectedObjectArray::ConstPtr &in_range_detections)
{

  autoware_msgs::DetectedObjectArray range_in_cv;
  autoware_msgs::DetectedObjectArray range_out_cv;
  TransformRangeToVision(in_range_detections, range_in_cv, range_out_cv);

  autoware_msgs::DetectedObjectArray fused_objects;
  fused_objects.header = in_range_detections->header;

  std::vector<std::vector<size_t> > vision_range_assignments(in_vision_detections->objects.size());
  std::vector<bool> used_vision_detections(in_vision_detections->objects.size(), false);
  std::vector<long> vision_range_closest(in_vision_detections->objects.size());

  for (size_t i = 0; i < in_vision_detections->objects.size(); i++)
  {
    auto vision_object = in_vision_detections->objects[i];

    cv::Rect vision_rect(vision_object.x, vision_object.y,
                         vision_object.width, vision_object.height);
    int vision_rect_area = vision_rect.area();
    long closest_index = -1;
    double closest_distance = std::numeric_limits<double>::max();

    for (size_t j = 0; j < range_in_cv.objects.size(); j++)
    {
      double current_distance = GetDistanceToObject(range_in_cv.objects[j]);

      cv::Rect range_rect = ProjectDetectionToRect(range_in_cv.objects[j]);
      int range_rect_area = range_rect.area();

      cv::Rect overlap = range_rect & vision_rect;
      if ((overlap.area() > range_rect_area * overlap_threshold_)
          || (overlap.area() > vision_rect_area * overlap_threshold_)
        )
      {
        vision_range_assignments[i].push_back(j);
        range_in_cv.objects[j].score = vision_object.score;
        range_in_cv.objects[j].label = vision_object.label;
        range_in_cv.objects[j].color = vision_object.color;
        range_in_cv.objects[j].image_frame = vision_object.image_frame;
        range_in_cv.objects[j].x = vision_object.x;
        range_in_cv.objects[j].y = vision_object.y;
        range_in_cv.objects[j].width = vision_object.width;
        range_in_cv.objects[j].height = vision_object.height;
        range_in_cv.objects[j].angle = vision_object.angle;
        range_in_cv.objects[j].id = vision_object.id;
        CheckMinimumDimensions(range_in_cv.objects[j]);
        if (vision_object.pose.orientation.x > 0
            || vision_object.pose.orientation.y > 0
            || vision_object.pose.orientation.z > 0)
        {
          range_in_cv.objects[i].pose.orientation = vision_object.pose.orientation;
        }
        if (current_distance < closest_distance)
        {
          closest_index = j;
          closest_distance = current_distance;
        }
        used_vision_detections[i] = true;
      }//end if overlap
    }//end for range_in_cv
    vision_range_closest[i] = closest_index;
  }

  std::vector<bool> used_range_detections(range_in_cv.objects.size(), false);
  //only assign the closest
  for (size_t i = 0; i < vision_range_assignments.size(); i++)
  {
    if (!range_in_cv.objects.empty() && vision_range_closest[i] >= 0)
    {
      used_range_detections[i] = true;
      fused_objects.objects.push_back(range_in_cv.objects[vision_range_closest[i]]);
    }
  }
  for (size_t i = 0; i < used_vision_detections.size(); i++)
  {
    if (!used_vision_detections[i])
    {
      fused_objects.objects.push_back(in_vision_detections->objects[i]);
    }
  }
  //add also objects outside the image
  for (auto &object: range_out_cv.objects)
  {
    fused_objects.objects.push_back(object);
  }
  //enable merged for visualization
  for (auto &object : fused_objects.objects)
  {
    object.valid = true;
  }

  return fused_objects;
}

void
ROSRangeVisionFusionApp::SyncedDetectionsCallback(
  const autoware_msgs::DetectedObjectArray::ConstPtr &in_vision_detections,
  const autoware_msgs::DetectedObjectArray::ConstPtr &in_range_detections)
{
  autoware_msgs::DetectedObjectArray fusion_objects;
  fusion_objects.objects.clear();

  if (empty_frames_ > 5)
  {
    ROS_INFO("[%s] Empty Detections. Make sure the vision and range detectors are running.", __APP_NAME__);
  }

  if (nullptr == in_vision_detections
      && nullptr == in_range_detections)
  {
    empty_frames_++;
    return;
  }

  if (nullptr == in_vision_detections
      && nullptr != in_range_detections
      && !in_range_detections->objects.empty())
  {
    publisher_fused_objects_.publish(in_range_detections);
    empty_frames_++;
    return;
  }
  if (nullptr == in_range_detections
      && nullptr != in_vision_detections
      && !in_vision_detections->objects.empty())
  {
    publisher_fused_objects_.publish(in_vision_detections);
    empty_frames_++;
    return;
  }

  if (!camera_lidar_tf_ok_)
  {
    camera_lidar_tf_ = FindTransform(image_frame_id_,
                                     in_range_detections->header.frame_id);
  }
  if (
    !camera_lidar_tf_ok_ ||
    !camera_info_ok_)
  {
    ROS_INFO("[%s] Missing Camera-LiDAR TF or CameraInfo", __APP_NAME__);
    return;
  }

  fusion_objects = FuseRangeVisionDetections(in_vision_detections, in_range_detections);

  publisher_fused_objects_.publish(fusion_objects);
  empty_frames_ = 0;

  vision_detections_ = nullptr;
  range_detections_ = nullptr;

}

void
ROSRangeVisionFusionApp::VisionDetectionsCallback(
  const autoware_msgs::DetectedObjectArray::ConstPtr &in_vision_detections)
{
  if (!processing_ && !in_vision_detections->objects.empty())
  {
    processing_ = true;
    vision_detections_ = in_vision_detections;
    SyncedDetectionsCallback(in_vision_detections, range_detections_);
    processing_ = false;
  }
}

void
ROSRangeVisionFusionApp::RangeDetectionsCallback(
  const autoware_msgs::DetectedObjectArray::ConstPtr &in_range_detections)
{
  if (!processing_ && !in_range_detections->objects.empty())
  {
    processing_ = true;
    range_detections_ = in_range_detections;
    SyncedDetectionsCallback(vision_detections_, in_range_detections);
    processing_ = false;
  }
}

void ROSRangeVisionFusionApp::ImageCallback(const sensor_msgs::Image::ConstPtr &in_image_msg)
{
  if (!camera_info_ok_)
    return;
  cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(in_image_msg, "bgr8");
  cv::Mat in_image = cv_image->image;

  cv::Mat undistorted_image;
  cv::undistort(in_image, image_, camera_instrinsics_, distortion_coefficients_);
};

void
ROSRangeVisionFusionApp::IntrinsicsCallback(const sensor_msgs::CameraInfo &in_message)
{
  image_size_.height = in_message.height;
  image_size_.width = in_message.width;

  camera_instrinsics_ = cv::Mat(3, 3, CV_64F);
  for (int row = 0; row < 3; row++)
  {
    for (int col = 0; col < 3; col++)
    {
      camera_instrinsics_.at<double>(row, col) = in_message.K[row * 3 + col];
    }
  }

  distortion_coefficients_ = cv::Mat(1, 5, CV_64F);
  for (int col = 0; col < 5; col++)
  {
    distortion_coefficients_.at<double>(col) = in_message.D[col];
  }

  fx_ = static_cast<float>(in_message.P[0]);
  fy_ = static_cast<float>(in_message.P[5]);
  cx_ = static_cast<float>(in_message.P[2]);
  cy_ = static_cast<float>(in_message.P[6]);

  intrinsics_subscriber_.shutdown();
  camera_info_ok_ = true;
  image_frame_id_ = in_message.header.frame_id;
  ROS_INFO("[%s] CameraIntrinsics obtained.", __APP_NAME__);
}

tf::StampedTransform
ROSRangeVisionFusionApp::FindTransform(const std::string &in_target_frame, const std::string &in_source_frame)
{
  tf::StampedTransform transform;

  ROS_INFO("%s - > %s", in_source_frame.c_str(), in_target_frame.c_str());
  camera_lidar_tf_ok_ = false;
  try
  {
    transform_listener_->lookupTransform(in_target_frame, in_source_frame, ros::Time(0), transform);
    camera_lidar_tf_ok_ = true;
    ROS_INFO("[%s] Camera-Lidar TF obtained", __APP_NAME__);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("[%s] %s", __APP_NAME__, ex.what());
  }

  return transform;
}

void
ROSRangeVisionFusionApp::InitializeROSIo(ros::NodeHandle &in_private_handle)
{
  //get params
  std::string camera_info_src, detected_objects_vision, min_car_dimensions, min_person_dimensions, min_truck_dimensions;
  std::string detected_objects_range, fused_topic_str = "/detection/fusion_tools/objects";
  std::string name_space_str = ros::this_node::getNamespace();
  bool sync_topics = false;

  ROS_INFO(
    "[%s] This node requires: Registered TF(Lidar-Camera), CameraInfo, Vision and Range Detections being published.",
    __APP_NAME__);
  in_private_handle.param<std::string>("detected_objects_range", detected_objects_range,
                                       "/detection/lidar_detector/objects");
  ROS_INFO("[%s] detected_objects_range: %s", __APP_NAME__, detected_objects_range.c_str());

  in_private_handle.param<std::string>("detected_objects_vision", detected_objects_vision,
                                       "/detection/image_detector/objects");
  ROS_INFO("[%s] detected_objects_vision: %s", __APP_NAME__, detected_objects_vision.c_str());

  in_private_handle.param<std::string>("camera_info_src", camera_info_src, "/camera_info");
  ROS_INFO("[%s] camera_info_src: %s", __APP_NAME__, camera_info_src.c_str());

  in_private_handle.param<double>("overlap_threshold", overlap_threshold_, 0.6);
  ROS_INFO("[%s] overlap_threshold: %f", __APP_NAME__, overlap_threshold_);

  in_private_handle.param<std::string>("min_car_dimensions", min_car_dimensions, "[3,2,2]");//w,h,d
  ROS_INFO("[%s] min_car_dimensions: %s", __APP_NAME__, min_car_dimensions.c_str());

  in_private_handle.param<std::string>("min_person_dimensions", min_person_dimensions, "[1,2,1]");
  ROS_INFO("[%s] min_person_dimensions: %s", __APP_NAME__, min_person_dimensions.c_str());

  in_private_handle.param<std::string>("min_truck_dimensions", min_truck_dimensions, "[4,2,2]");
  ROS_INFO("[%s] min_truck_dimensions: %s", __APP_NAME__, min_truck_dimensions.c_str());


  in_private_handle.param<bool>("sync_topics", sync_topics, false);
  ROS_INFO("[%s] sync_topics: %d", __APP_NAME__, sync_topics);

  YAML::Node car_dimensions = YAML::Load(min_car_dimensions);
  YAML::Node person_dimensions = YAML::Load(min_person_dimensions);
  YAML::Node truck_dimensions = YAML::Load(min_truck_dimensions);

  if (car_dimensions.size() == 3)
  {
    car_width_ = car_dimensions[0].as<double>();
    car_height_ = car_dimensions[1].as<double>();
    car_depth_ = car_dimensions[2].as<double>();
  }
  if (person_dimensions.size() == 3)
  {
    person_width_ = person_dimensions[0].as<double>();
    person_height_ = person_dimensions[1].as<double>();
    person_depth_ = person_dimensions[2].as<double>();
  }
  if (truck_dimensions.size() == 3)
  {
    truck_width_ = truck_dimensions[0].as<double>();
    truck_height_ = truck_dimensions[1].as<double>();
    truck_depth_ = truck_dimensions[2].as<double>();
  }

  if (name_space_str != "/")
  {
    if (name_space_str.substr(0, 2) == "//")
    {
      name_space_str.erase(name_space_str.begin());
    }
    camera_info_src = name_space_str + camera_info_src;
  }

  //generate subscribers and sychronizers
  ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, camera_info_src.c_str());
  intrinsics_subscriber_ = in_private_handle.subscribe(camera_info_src,
                                                       1,
                                                       &ROSRangeVisionFusionApp::IntrinsicsCallback, this);

  ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, detected_objects_vision.c_str());
  ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, detected_objects_range.c_str());
  if (!sync_topics)
  {
    detections_range_subscriber_ = in_private_handle.subscribe(detected_objects_vision,
                                                               1,
                                                               &ROSRangeVisionFusionApp::VisionDetectionsCallback,
                                                               this);

    detections_vision_subscriber_ = in_private_handle.subscribe(detected_objects_range,
                                                                1,
                                                                &ROSRangeVisionFusionApp::RangeDetectionsCallback,
                                                                this);
  }
  else
  {
    vision_filter_subscriber_ = new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>(node_handle_,
                                                                                                    detected_objects_vision,
                                                                                                    1);
    range_filter_subscriber_ = new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>(node_handle_,
                                                                                                   detected_objects_range,
                                                                                                   1);
    detections_synchronizer_ =
      new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(10),
                                                     *vision_filter_subscriber_,
                                                     *range_filter_subscriber_);
    detections_synchronizer_->registerCallback(
      boost::bind(&ROSRangeVisionFusionApp::SyncedDetectionsCallback, this, _1, _2));
  }

  publisher_fused_objects_ = node_handle_.advertise<autoware_msgs::DetectedObjectArray>(fused_topic_str, 1);

  ROS_INFO("[%s] Publishing fused objects in %s", __APP_NAME__, fused_topic_str.c_str());

}


void
ROSRangeVisionFusionApp::Run()
{
  ros::NodeHandle private_node_handle("~");
  tf::TransformListener transform_listener;

  transform_listener_ = &transform_listener;

  InitializeROSIo(private_node_handle);

  ROS_INFO("[%s] Ready. Waiting for data...", __APP_NAME__);

  ros::spin();

  ROS_INFO("[%s] END", __APP_NAME__);
}

ROSRangeVisionFusionApp::ROSRangeVisionFusionApp()
{
  camera_lidar_tf_ok_ = false;
  camera_info_ok_ = false;
  processing_ = false;
  image_frame_id_ = "";
  overlap_threshold_ = 0.5;
  empty_frames_ = 0;
}