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
  std::string markers_out_topic = ros_namespace_ + "/objects_markers";

  private_nh_.param<double>("object_speed_threshold", object_speed_threshold_, 0.1);
  ROS_INFO("[%s] object_speed_threshold: %.2f", __APP_NAME__, object_speed_threshold_);

  private_nh_.param<double>("arrow_speed_threshold", arrow_speed_threshold_, 0.25);
  ROS_INFO("[%s] arrow_speed_threshold: %.2f", __APP_NAME__, arrow_speed_threshold_);

  private_nh_.param<double>("marker_display_duration", marker_display_duration_, 0.2);
  ROS_INFO("[%s] marker_display_duration: %.2f", __APP_NAME__, marker_display_duration_);

  std::vector<double> color;
  private_nh_.param<std::vector<double>>("label_color", color, {255.,255.,255.,1.0});
  label_color_ = ParseColor(color);
  ROS_INFO("[%s] label_color: %s", __APP_NAME__, ColorToString(label_color_).c_str());

  private_nh_.param<std::vector<double>>("arrow_color", color, {0.,255.,0.,0.8});
  arrow_color_ = ParseColor(color);
  ROS_INFO("[%s] arrow_color: %s", __APP_NAME__, ColorToString(arrow_color_).c_str());

  private_nh_.param<std::vector<double>>("hull_color", color, {51.,204.,51.,0.8});
  hull_color_ = ParseColor(color);
  ROS_INFO("[%s] hull_color: %s", __APP_NAME__, ColorToString(hull_color_).c_str());

  private_nh_.param<std::vector<double>>("box_color", color, {51.,128.,204.,0.8});
  box_color_ = ParseColor(color);
  ROS_INFO("[%s] box_color: %s", __APP_NAME__, ColorToString(box_color_).c_str());

  private_nh_.param<std::vector<double>>("centroid_color", color, {77.,121.,255.,0.8});
  centroid_color_ = ParseColor(color);
  ROS_INFO("[%s] centroid_color: %s", __APP_NAME__, ColorToString(centroid_color_).c_str());

  subscriber_detected_objects_ =
    node_handle_.subscribe(object_src_topic, 1,
                           &VisualizeDetectedObjects::DetectedObjectsCallback, this);
  ROS_INFO("[%s] object_src_topic: %s", __APP_NAME__, object_src_topic.c_str());

  publisher_markers_ = node_handle_.advertise<visualization_msgs::MarkerArray>(
    markers_out_topic, 1);
  ROS_INFO("[%s] markers_out_topic: %s", __APP_NAME__, markers_out_topic.c_str());

}

std::string VisualizeDetectedObjects::ColorToString(const std_msgs::ColorRGBA &in_color)
{
  std::stringstream stream;

  stream << "{R:" << std::fixed << std::setprecision(1) << in_color.r*255 << ", ";
  stream << "G:" << std::fixed << std::setprecision(1) << in_color.g*255 << ", ";
  stream << "B:" << std::fixed << std::setprecision(1) << in_color.b*255 << ", ";
  stream << "A:" << std::fixed << std::setprecision(1) << in_color.a << "}";
  return stream.str();
}

float VisualizeDetectedObjects::CheckColor(double value)
{
  float final_value;
  if (value > 255.)
    final_value = 1.f;
  else if (value < 0)
    final_value = 0.f;
  else
    final_value = value/255.f;
  return final_value;
}

float VisualizeDetectedObjects::CheckAlpha(double value)
{
  float final_value;
  if (value > 1.)
    final_value = 1.f;
  else if (value < 0.1)
    final_value = 0.1f;
  else
    final_value = value;
  return final_value;
}

std_msgs::ColorRGBA VisualizeDetectedObjects::ParseColor(const std::vector<double> &in_color)
{
  std_msgs::ColorRGBA color;
  float r,g,b,a;
  if (in_color.size() == 4) //r,g,b,a
  {
    color.r = CheckColor(in_color[0]);
    color.g = CheckColor(in_color[1]);
    color.b = CheckColor(in_color[2]);
    color.a = CheckAlpha(in_color[3]);
  }
  return color;
}

void VisualizeDetectedObjects::DetectedObjectsCallback(const autoware_msgs::DetectedObjectArray &in_objects)
{
  visualization_msgs::MarkerArray label_markers, arrow_markers, centroid_markers;
  visualization_msgs::MarkerArray polygon_hulls;
  visualization_msgs::MarkerArray bounding_boxes;

  visualization_msgs::MarkerArray visualization_markers;

  marker_id_ = 0;

  label_markers = ObjectsToLabels(in_objects);
  arrow_markers = ObjectsToArrows(in_objects);
  polygon_hulls = ObjectsToHulls(in_objects);
  bounding_boxes = ObjectsToBoxes(in_objects);
  centroid_markers = ObjectsToCentroids(in_objects);

  visualization_markers.markers.insert(visualization_markers.markers.end(),
                                       label_markers.markers.begin(), label_markers.markers.end());
  visualization_markers.markers.insert(visualization_markers.markers.end(),
                                       arrow_markers.markers.begin(), arrow_markers.markers.end());
  visualization_markers.markers.insert(visualization_markers.markers.end(),
                                       polygon_hulls.markers.begin(), polygon_hulls.markers.end());
  visualization_markers.markers.insert(visualization_markers.markers.end(),
                                       bounding_boxes.markers.begin(), bounding_boxes.markers.end());
  visualization_markers.markers.insert(visualization_markers.markers.end(),
                                       centroid_markers.markers.begin(), centroid_markers.markers.end());

  publisher_markers_.publish(visualization_markers);

}

visualization_msgs::MarkerArray
VisualizeDetectedObjects::ObjectsToCentroids(const autoware_msgs::DetectedObjectArray &in_objects)
{
  visualization_msgs::MarkerArray centroid_markers;
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

      centroid_marker.scale.x = 0.5;
      centroid_marker.scale.y = 0.5;
      centroid_marker.scale.z = 0.5;

      if (object.color.a == 0)
      {
        centroid_marker.color = centroid_color_;
      }
      else
      {
        centroid_marker.color = object.color;
      }
      centroid_marker.id = marker_id_++;
      centroid_markers.markers.push_back(centroid_marker);
    }
  }
  return centroid_markers;
}//ObjectsToCentroids

visualization_msgs::MarkerArray
VisualizeDetectedObjects::ObjectsToBoxes(const autoware_msgs::DetectedObjectArray &in_objects)
{
  visualization_msgs::MarkerArray object_boxes;

  for (auto const &object: in_objects.objects)
  {
    if (IsObjectValid(object) &&
       object.label != "unknown" &&
        (object.dimensions.x + object.dimensions.y + object.dimensions.z) < object_max_linear_size_)
    {
      visualization_msgs::Marker box;

      box.lifetime = ros::Duration(marker_display_duration_);
      box.header = in_objects.header;
      box.type = visualization_msgs::Marker::CUBE;
      box.action = visualization_msgs::Marker::ADD;
      box.ns = ros_namespace_ + "/box_markers";
      box.id = marker_id_++;
      box.scale = object.dimensions;
      box.pose.position = object.pose.position;

      if (object.pose_reliable)
        box.pose.orientation = object.pose.orientation;

      if (object.color.a == 0)
      {
        box.color = box_color_;
      }
      else
      {
        box.color = object.color;
      }

      object_boxes.markers.push_back(box);
    }
  }
  return object_boxes;
}//ObjectsToBoxes

visualization_msgs::MarkerArray
VisualizeDetectedObjects::ObjectsToHulls(const autoware_msgs::DetectedObjectArray &in_objects)
{
  visualization_msgs::MarkerArray polygon_hulls;

  for (auto const &object: in_objects.objects)
  {
    if (IsObjectValid(object) && !object.convex_hull.polygon.points.empty() && object.label == "unknown")
    {
      visualization_msgs::Marker hull;
      hull.lifetime = ros::Duration(marker_display_duration_);
      hull.header = in_objects.header;
      hull.type = visualization_msgs::Marker::LINE_STRIP;
      hull.action = visualization_msgs::Marker::ADD;
      hull.ns = ros_namespace_ + "/hull_markers";
      hull.id = marker_id_++;
      hull.scale.x = 0.2;

      for(auto const &point: object.convex_hull.polygon.points)
      {
        geometry_msgs::Point tmp_point;
        tmp_point.x = point.x;
        tmp_point.y = point.y;
        tmp_point.z = point.z;
        hull.points.push_back(tmp_point);
      }

      if (object.color.a == 0)
      {
        hull.color = hull_color_;
      }
      else
      {
        hull.color = object.color;
      }

      polygon_hulls.markers.push_back(hull);
    }
  }
  return polygon_hulls;
}

visualization_msgs::MarkerArray
VisualizeDetectedObjects::ObjectsToArrows(const autoware_msgs::DetectedObjectArray &in_objects)
{
  visualization_msgs::MarkerArray arrow_markers;
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
          arrow_marker.color = arrow_color_;
        }
        else
        {
          arrow_marker.color = object.color;
        }
        arrow_marker.id = marker_id_++;

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

      label_marker.color = label_color_;

      label_marker.id = marker_id_++;

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