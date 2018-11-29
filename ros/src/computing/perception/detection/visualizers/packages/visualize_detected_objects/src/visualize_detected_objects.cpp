/*
 *  Copyright (c) 2018, Nagoya University
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

#include "visualize_detected_objects.h"

VisualizeDetectedObjects::VisualizeDetectedObjects() : arrow_height_(0.5), label_height_(1.0)
{
  ros::NodeHandle private_nh_("~");

  ros_namespace_ = ros::this_node::getNamespace();

  if (ros_namespace_.substr(0, 2) == "//")
  {
    ros_namespace_.erase(ros_namespace_.begin());
  }

  std::string object_src_topic = ros_namespace_ + "/objects";
  std::string labels_out_topic = ros_namespace_ + "/objects_labels";
  std::string arrows_out_topic = ros_namespace_ + "/objects_arrows";
  std::string hulls_out_topic = ros_namespace_ + "/objects_hulls";
  std::string boxes_out_topic = ros_namespace_ + "/objects_boxes";
  std::string centroids_out_topic = ros_namespace_ + "/objects_centroids";

  private_nh_.param<double>("object_speed_threshold", object_speed_threshold_, 0.1);
  ROS_INFO("[%s] object_speed_threshold: %.2f", __APP_NAME__, object_speed_threshold_);

  private_nh_.param<double>("arrow_speed_threshold", arrow_speed_threshold_, 0.25);
  ROS_INFO("[%s] arrow_speed_threshold: %.2f", __APP_NAME__, arrow_speed_threshold_);

  private_nh_.param<double>("marker_display_duration", marker_display_duration_, 0.2);
  ROS_INFO("[%s] marker_display_duration: %.2f", __APP_NAME__, marker_display_duration_);

  subscriber_detected_objects_ =
    node_handle_.subscribe(object_src_topic, 1,
                           &VisualizeDetectedObjects::DetectedObjectsCallback, this);
  ROS_INFO("[%s] object_src_topic: %s", __APP_NAME__, object_src_topic.c_str());


  publisher_label_markers_ = node_handle_.advertise<visualization_msgs::MarkerArray>(
    labels_out_topic, 1);
  ROS_INFO("[%s] labels_out_topic: %s", __APP_NAME__, labels_out_topic.c_str());

  publisher_arrow_markers_ = node_handle_.advertise<visualization_msgs::MarkerArray>(
    arrows_out_topic, 1);
  ROS_INFO("[%s] arrows_out_topic: %s", __APP_NAME__, arrows_out_topic.c_str());

  publisher_polygon_hulls_ = node_handle_.advertise<jsk_recognition_msgs::PolygonArray>(
    hulls_out_topic, 1);
  ROS_INFO("[%s] hulls_out_topic: %s", __APP_NAME__, hulls_out_topic.c_str());

  publisher_bounding_boxes_ = node_handle_.advertise<jsk_recognition_msgs::BoundingBoxArray>(
    boxes_out_topic, 1);
  ROS_INFO("[%s] boxes_out_topic: %s", __APP_NAME__, boxes_out_topic.c_str());

  publisher_centroid_markers_ = node_handle_.advertise<visualization_msgs::MarkerArray>(
    centroids_out_topic, 1);
  ROS_INFO("[%s] centroids_out_topic: %s", __APP_NAME__, centroids_out_topic.c_str());

}

void VisualizeDetectedObjects::DetectedObjectsCallback(const autoware_msgs::DetectedObjectArray &in_objects)
{
  visualization_msgs::MarkerArray label_markers, arrow_markers, centroid_markers;
  jsk_recognition_msgs::PolygonArray polygon_hulls;
  jsk_recognition_msgs::BoundingBoxArray bounding_boxes;

  label_markers = ObjectsToLabels(in_objects);
  arrow_markers = ObjectsToArrows(in_objects);
  polygon_hulls = ObjectsToHulls(in_objects);
  bounding_boxes = ObjectsToBoxes(in_objects);
  centroid_markers = ObjectsToCentroids(in_objects);

  publisher_label_markers_.publish(label_markers);
  publisher_arrow_markers_.publish(arrow_markers);
  publisher_polygon_hulls_.publish(polygon_hulls);
  publisher_bounding_boxes_.publish(bounding_boxes);
  publisher_centroid_markers_.publish(centroid_markers);

}

visualization_msgs::MarkerArray
VisualizeDetectedObjects::ObjectsToCentroids(const autoware_msgs::DetectedObjectArray &in_objects)
{
  visualization_msgs::MarkerArray centroid_markers;
  int marker_id = 0;
  for (auto const &object: in_objects.objects)
  {
    if (IsObjectValid(object))
    {
      visualization_msgs::Marker centroid_marker;
      centroid_marker.lifetime = ros::Duration(marker_display_duration_);

      centroid_marker.header = in_objects.header;
      centroid_marker.type = visualization_msgs::Marker::SPHERE;
      centroid_marker.action = visualization_msgs::Marker::ADD;
      centroid_marker.pose = object.pose;
      centroid_marker.ns = ros_namespace_ + "/centroid_markers";

      centroid_marker.scale.x = 1.0;
      centroid_marker.scale.y = 1.0;
      centroid_marker.scale.z = 1.0;

      if (object.color.a == 0)
      {
        centroid_marker.color.a = 1.f;
        centroid_marker.color.g = 1.f;
      }
      else
      {
        centroid_marker.color = object.color;
      }
      centroid_marker.id = marker_id++;
      centroid_markers.markers.push_back(centroid_marker);
    }
  }
  return centroid_markers;
}//ObjectsToCentroids

jsk_recognition_msgs::BoundingBoxArray
VisualizeDetectedObjects::ObjectsToBoxes(const autoware_msgs::DetectedObjectArray &in_objects)
{
  jsk_recognition_msgs::BoundingBoxArray object_boxes;
  int marker_id = 0;

  object_boxes.header = in_objects.header;

  for (auto const &object: in_objects.objects)
  {
    if (IsObjectValid(object) &&
        (object.dimensions.x + object.dimensions.y + object.dimensions.z) < object_max_linear_size_)
    {
      jsk_recognition_msgs::BoundingBox box;
      box.header = in_objects.header;
      box.dimensions = object.dimensions;
      box.pose.position = object.pose.position;
      if (object.pose_reliable)
        box.pose.orientation = object.pose.orientation;
      object_boxes.boxes.push_back(box);
    }
  }
  return object_boxes;
}//ObjectsToBoxes

jsk_recognition_msgs::PolygonArray
VisualizeDetectedObjects::ObjectsToHulls(const autoware_msgs::DetectedObjectArray &in_objects)
{
  jsk_recognition_msgs::PolygonArray polygon_hulls;

  polygon_hulls.header = in_objects.header;

  for (auto const &object: in_objects.objects)
  {
    if (IsObjectValid(object))
      polygon_hulls.polygons.push_back(object.convex_hull);
  }
  return polygon_hulls;
}

visualization_msgs::MarkerArray
VisualizeDetectedObjects::ObjectsToArrows(const autoware_msgs::DetectedObjectArray &in_objects)
{
  visualization_msgs::MarkerArray arrow_markers;
  int marker_id = 0;
  for (auto const &object: in_objects.objects)
  {
    if (IsObjectValid(object) && object.pose_reliable)
    {
      double velocity = object.velocity.linear.x;

      if (abs(velocity) >= arrow_speed_threshold_)
      {
        visualization_msgs::Marker arrow_marker;
        arrow_marker.lifetime = ros::Duration(marker_display_duration_);

        tf::Quaternion q(object.pose.orientation.x,
                         object.pose.orientation.y,
                         object.pose.orientation.z,
                         object.pose.orientation.w);
        double roll, pitch, yaw;

        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // in the case motion model fit opposite direction
        if (velocity < -0.1)
        {
          yaw += M_PI;
          // normalize angle
          while (yaw > M_PI)
            yaw -= 2. * M_PI;
          while (yaw < -M_PI)
            yaw += 2. * M_PI;
        }

        tf::Matrix3x3 obs_mat;
        tf::Quaternion q_tf;

        obs_mat.setEulerYPR(yaw, 0, 0);  // yaw, pitch, roll
        obs_mat.getRotation(q_tf);

        arrow_marker.header = in_objects.header;
        arrow_marker.ns = ros_namespace_ + "/arrow_markers";
        arrow_marker.action = visualization_msgs::Marker::ADD;
        arrow_marker.type = visualization_msgs::Marker::ARROW;

        // green
        if (object.color.a == 0)
        {
          arrow_marker.color.g = 1.f;
          arrow_marker.color.a = 1.f;
        }
        else
        {
          arrow_marker.color = object.color;
        }
        arrow_marker.id = marker_id++;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        arrow_marker.pose.position.x = object.pose.position.x;
        arrow_marker.pose.position.y = object.pose.position.y;
        arrow_marker.pose.position.z = arrow_height_;

        arrow_marker.pose.orientation.x = q_tf.getX();
        arrow_marker.pose.orientation.y = q_tf.getY();
        arrow_marker.pose.orientation.z = q_tf.getZ();
        arrow_marker.pose.orientation.w = q_tf.getW();

        // Set the scale of the arrow -- 1x1x1 here means 1m on a side
        arrow_marker.scale.x = 3;
        arrow_marker.scale.y = 0.1;
        arrow_marker.scale.z = 0.1;

        arrow_markers.markers.push_back(arrow_marker);
      }//velocity threshold
    }//valid object
  }//end for
  return arrow_markers;
}//ObjectsToArrows

bool VisualizeDetectedObjects::IsObjectValid(const autoware_msgs::DetectedObject &in_object)
{
  if (!in_object.valid ||
    std::isnan(in_object.pose.orientation.x) ||
    std::isnan(in_object.pose.orientation.y) ||
    std::isnan(in_object.pose.orientation.z) ||
    std::isnan(in_object.pose.orientation.w) ||
    std::isnan(in_object.pose.position.x) ||
    std::isnan(in_object.pose.position.y) ||
    std::isnan(in_object.pose.position.z) ||
    (in_object.pose.position.x == 0.) ||
    (in_object.pose.position.y == 0.) ||
    (in_object.dimensions.x <= 0.) ||
    (in_object.dimensions.y <= 0.) ||
    (in_object.dimensions.z <= 0.)
    )
  {
    return false;
  }
  return true;
}//end IsObjectValid

visualization_msgs::MarkerArray
VisualizeDetectedObjects::ObjectsToLabels(const autoware_msgs::DetectedObjectArray &in_objects)
{
  visualization_msgs::MarkerArray label_markers;
  for (auto const &object: in_objects.objects)
  {
    if (IsObjectValid(object))
    {
      visualization_msgs::Marker label_marker;

      label_marker.lifetime = ros::Duration(marker_display_duration_);
      label_marker.header = in_objects.header;
      label_marker.ns = ros_namespace_ + "/label_markers";
      label_marker.action = visualization_msgs::Marker::ADD;
      label_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      label_marker.scale.x = 1.5;
      label_marker.scale.y = 1.5;
      label_marker.scale.z = 1.5;

      label_marker.color.r = 1.f;
      label_marker.color.g = 1.f;
      label_marker.color.b = 1.f;
      label_marker.color.a = 1.f;

      label_marker.id = object.id;

      if(!object.label.empty() && object.label != "unknown")
        label_marker.text = object.label + " "; //Object Class if available

      std::stringstream distance_stream;
      distance_stream << std::fixed << std::setprecision(1)
                      << sqrt((object.pose.position.x * object.pose.position.x) +
                                (object.pose.position.y * object.pose.position.y));
      std::string distance_str = distance_stream.str() + " m";
      label_marker.text += distance_str;

      if (object.velocity_reliable)
      {
        double velocity = object.velocity.linear.x;
        if (velocity < -0.1)
        {
          velocity *= -1;
        }

        if (abs(velocity) < object_speed_threshold_)
        {
          velocity = 0.0;
        }

        tf::Quaternion q(object.pose.orientation.x, object.pose.orientation.y,
                         object.pose.orientation.z, object.pose.orientation.w);

        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // convert m/s to km/h
        std::stringstream kmh_velocity_stream;
        kmh_velocity_stream << std::fixed << std::setprecision(1) << (velocity * 3.6);
        std::string text = "\n<" + std::to_string(object.id) + "> " + kmh_velocity_stream.str() + " km/h";
        label_marker.text += text;
      }

      label_marker.pose.position.x = object.pose.position.x;
      label_marker.pose.position.y = object.pose.position.y;
      label_marker.pose.position.z = label_height_;
      label_marker.scale.z = 1.0;
      if (!label_marker.text.empty())
        label_markers.markers.push_back(label_marker);
    }
  }  // end in_objects.objects loop

  return label_markers;
}//ObjectsToLabels
