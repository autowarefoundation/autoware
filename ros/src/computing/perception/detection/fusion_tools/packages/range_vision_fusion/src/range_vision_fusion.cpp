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
 ********************
 *  v1.0: amc-nu (abrahammonrroy@yahoo.com)
 *
 * range_vision_fusion_node.cpp
 *
 *  Created on: July, 05th, 2018
 */

#include "range_vision_fusion/range_vision_fusion.h"

cv::Point3f
RosRangeVisionFusionApp::TransformPoint(const geometry_msgs::Point &in_point, const tf::StampedTransform &in_transform)
{
    tf::Vector3 tf_point(in_point.x, in_point.y, in_point.z);
    tf::Vector3 tf_point_t = in_transform * tf_point;
    return cv::Point3f(tf_point_t.x(), tf_point_t.y(), tf_point_t.z());
}

cv::Point2i
RosRangeVisionFusionApp::ProjectPoint(const cv::Point3f &in_point)
{
    auto u = int(in_point.x * fx_ / in_point.z + cx_);
    auto v = int(in_point.y * fy_ / in_point.z + cy_);

    return cv::Point2i(u, v);
}

autoware_msgs::DetectedObject
RosRangeVisionFusionApp::TransformObject(const autoware_msgs::DetectedObject &in_detection,
                                                                       const tf::StampedTransform& in_transform)
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
RosRangeVisionFusionApp::IsObjectInImage(const autoware_msgs::DetectedObject &in_detection)
{
    cv::Point3f image_space_point = TransformPoint(in_detection.pose.position, camera_lidar_tf_);

    cv::Point2i image_pixel = ProjectPoint(image_space_point);

    return (image_pixel.x >= 0)
           && (image_pixel.x < image_size_.width)
           && (image_pixel.y >= 0)
           && (image_pixel.y < image_size_.height)
           && (image_space_point.z > 0);
}

cv::Rect RosRangeVisionFusionApp::ProjectDetectionToRect(const autoware_msgs::DetectedObject &in_detection)
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

    std::vector<double> dims = {in_detection.dimensions.x,
                                  in_detection.dimensions.y,
                                  in_detection.dimensions.z};

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
RosRangeVisionFusionApp::TransformRangeToVision(const autoware_msgs::DetectedObjectArray::ConstPtr &in_range_detections,
                                                      autoware_msgs::DetectedObjectArray &out_in_cv_range_detections,
                                                      autoware_msgs::DetectedObjectArray &out_out_cv_range_detections)
{
    out_in_cv_range_detections.header = in_range_detections->header;
    out_in_cv_range_detections.objects.clear();
    out_out_cv_range_detections.header = in_range_detections->header;
    out_out_cv_range_detections.objects.clear();
    for (size_t i= 0; i < in_range_detections->objects.size(); i++)
    {
        if(IsObjectInImage(in_range_detections->objects[i]))
        {
            out_in_cv_range_detections.objects.push_back(in_range_detections->objects[i]);
        }
        else
        {
            out_out_cv_range_detections.objects.push_back(in_range_detections->objects[i]);
        }
    }
}

void
RosRangeVisionFusionApp::CalculateObjectFeatures(autoware_msgs::DetectedObject &in_out_object, bool in_estimate_pose)
{

    float min_x=std::numeric_limits<float>::max();float max_x=-std::numeric_limits<float>::max();
    float min_y=std::numeric_limits<float>::max();float max_y=-std::numeric_limits<float>::max();
    float min_z=std::numeric_limits<float>::max();float max_z=-std::numeric_limits<float>::max();
    float average_x = 0, average_y = 0, average_z = 0, length, width, height;
    pcl::PointXYZ centroid, min_point, max_point, average_point;

    std::vector<cv::Point2f> object_2d_points;

    pcl::PointCloud<pcl::PointXYZ> in_cloud;
    pcl::fromROSMsg(in_out_object.pointcloud, in_cloud);

    for (const auto &point : in_cloud.points)
    {
        average_x+=point.x;		average_y+=point.y;		average_z+=point.z;
        centroid.x += point.x; centroid.y += point.y;	centroid.z += point.z;

        if(point.x<min_x)	min_x = point.x;
        if(point.y<min_y)	min_y = point.y;
        if(point.z<min_z)	min_z = point.z;
        if(point.x>max_x)	max_x = point.x;
        if(point.y>max_y)	max_y = point.y;
        if(point.z>max_z)	max_z = point.z;

        cv::Point2f pt;
        pt.x = point.x;
        pt.y = point.y;
        object_2d_points.push_back(pt);
    }
    min_point.x = min_x;	min_point.y = min_y;	min_point.z = min_z;
    max_point.x = max_x;	max_point.y = max_y;	max_point.z = max_z;

    if (in_cloud.points.size() > 0)
    {
        centroid.x /= in_cloud.points.size();
        centroid.y /= in_cloud.points.size();
        centroid.z /= in_cloud.points.size();

        average_x /= in_cloud.points.size();
        average_y /= in_cloud.points.size();
        average_z /= in_cloud.points.size();
    }

    average_point.x = average_x; average_point.y = average_y;	average_point.z = average_z;

    length = max_point.x - min_point.x;
    width = max_point.y - min_point.y;
    height = max_point.z - min_point.z;

    geometry_msgs::PolygonStamped  convex_hull;
    std::vector<cv::Point2f> hull_points;
    if (object_2d_points.size() > 0)
        cv::convexHull(object_2d_points, hull_points);

    convex_hull.header = in_out_object.header;
    for (size_t i = 0; i < hull_points.size() + 1 ; i++)
    {
        geometry_msgs::Point32 point;
        point.x = hull_points[i%hull_points.size()].x;
        point.y = hull_points[i%hull_points.size()].y;
        point.z = min_point.z;
        convex_hull.polygon.points.push_back(point);
    }

    for (size_t i = 0; i < hull_points.size() + 1 ; i++)
    {
        geometry_msgs::Point32 point;
        point.x = hull_points[i%hull_points.size()].x;
        point.y = hull_points[i%hull_points.size()].y;
        point.z = max_point.z;
        convex_hull.polygon.points.push_back(point);
    }

    double rz = 0;
    if (in_estimate_pose)
    {
        cv::RotatedRect box = cv::minAreaRect(hull_points);
        rz = box.angle*3.14/180;
        in_out_object.pose.position.x = box.center.x;
        in_out_object.pose.position.y = box.center.y;
        in_out_object.dimensions.x = box.size.width;
        in_out_object.dimensions.y = box.size.height;
    }

    in_out_object.convex_hull = convex_hull;

    in_out_object.pose.position.x = min_point.x + length/2;
    in_out_object.pose.position.y = min_point.y + width/2;
    in_out_object.pose.position.z = min_point.z + height/2;

    in_out_object.dimensions.x = ((length<0)?-1*length:length);
    in_out_object.dimensions.y = ((width<0)?-1*width:width);
    in_out_object.dimensions.z = ((height<0)?-1*height:height);

    tf::Quaternion quat = tf::createQuaternionFromRPY(0.0, 0.0, rz);
    tf::quaternionTFToMsg(quat, in_out_object.pose.orientation);
}

autoware_msgs::DetectedObject RosRangeVisionFusionApp::MergeObjects(const autoware_msgs::DetectedObject &in_object_a,
                                           const autoware_msgs::DetectedObject & in_object_b)
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

double RosRangeVisionFusionApp::GetDistanceToObject(const autoware_msgs::DetectedObject &in_object)
{
    return sqrt(in_object.dimensions.x*in_object.dimensions.x +
                in_object.dimensions.y*in_object.dimensions.y +
                in_object.dimensions.z*in_object.dimensions.z);
}

autoware_msgs::DetectedObjectArray
RosRangeVisionFusionApp::FuseRangeVisionDetections(const autoware_msgs::DetectedObjectArray::ConstPtr &in_vision_detections,
                                                   const autoware_msgs::DetectedObjectArray::ConstPtr &in_range_detections)
{

    autoware_msgs::DetectedObjectArray range_in_cv;
    autoware_msgs::DetectedObjectArray range_out_cv;
    TransformRangeToVision(in_range_detections, range_in_cv, range_out_cv);

    autoware_msgs::DetectedObjectArray fused_objects;
    fused_objects.header = in_range_detections->header;

    std::vector< std::vector<size_t> > vision_range_assignments (in_vision_detections->objects.size());
    std::vector< long > vision_range_closest (in_vision_detections->objects.size());

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
            if ( (overlap.area() > range_rect_area*overlap_threshold_)
                 || (overlap.area() > vision_rect_area*overlap_threshold_)
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
                if(current_distance < closest_distance)
                {
                    closest_index = j;
                    closest_distance = current_distance;
                }
            }
        }
        vision_range_closest[i] = closest_index;
    }

    std::vector<bool> used_range_detections(range_in_cv.objects.size(), false);
    //only assign the closest
    for(size_t i = 0; i < vision_range_assignments.size(); i++)
    {
        if(!range_in_cv.objects.empty() && vision_range_closest[i] >= 0)
        {
            used_range_detections[i] = true;
            fused_objects.objects.push_back(range_in_cv.objects[vision_range_closest[i]]);
        }
    }

    /*
    for(size_t i = 0; i < vision_range_assignments.size(); i++)
    {
        autoware_msgs::DetectedObject merged_object = range_in_cv.objects[0];

        for(const auto& range_detection_idx: vision_range_assignments[i])
        {
            if(merged_object.label == range_in_cv.objects[range_detection_idx].label)
            {
                used_range_detections[range_detection_idx] = true;

                merged_object = MergeObjects(merged_object, range_in_cv.objects[range_detection_idx]);
            }
        }
        if(!vision_range_assignments[i].empty())
        {
            CalculateObjectFeatures(merged_object, true);
            fused_objects.objects.push_back(merged_object);
        }
    }*/

    //add objects outside image
    for(size_t i=0; i < range_out_cv.objects.size(); i++)
    {
        fused_objects.objects.push_back(range_out_cv.objects[i]);
    }

    return fused_objects;
}

void
RosRangeVisionFusionApp::SyncedDetectionsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr &in_vision_detections,
                                                       const autoware_msgs::DetectedObjectArray::ConstPtr &in_range_detections)
{
    autoware_msgs::DetectedObjectArray fusion_objects;
    jsk_recognition_msgs::BoundingBoxArray fused_boxes;
    visualization_msgs::MarkerArray fused_objects_labels;

    if (nullptr == in_vision_detections ||
        nullptr == in_range_detections)
    {
        ROS_INFO("[%s] Empty Detections, check that vision and range detectors are running and publishing.", __APP_NAME__);
        if (empty_frames_ > 5)
        {
            publisher_fused_objects_.publish(fusion_objects);
            publisher_fused_boxes_.publish(fused_boxes);
            publisher_fused_text_.publish(fused_objects_labels);
            empty_frames_++;
        }
        return;
    }

    if (!camera_lidar_tf_ok_)
    {
        camera_lidar_tf_ = FindTransform(image_frame_id_,
                                         in_range_detections->header.frame_id);
    }
    if(
        !camera_lidar_tf_ok_ ||
        !camera_info_ok_)
    {
        ROS_INFO("[%s] Missing Camera-LiDAR TF or CameraInfo", __APP_NAME__);
        return;
    }

    fusion_objects = FuseRangeVisionDetections(in_vision_detections, in_range_detections);
    fused_boxes = ObjectsToBoxes(fusion_objects);
    fused_objects_labels = ObjectsToMarkers(fusion_objects);

    publisher_fused_objects_.publish(fusion_objects);
    publisher_fused_boxes_.publish(fused_boxes);
    publisher_fused_text_.publish(fused_objects_labels);
    empty_frames_ = 0;

    vision_detections_ = nullptr;
    range_detections_ = nullptr;

}

visualization_msgs::MarkerArray
RosRangeVisionFusionApp::ObjectsToMarkers(const autoware_msgs::DetectedObjectArray &in_objects)
{
    visualization_msgs::MarkerArray final_markers;

    for(const autoware_msgs::DetectedObject& object : in_objects.objects)
    {
        if (object.label != "unknown")
        {
            visualization_msgs::Marker marker;
            marker.header = in_objects.header;
            marker.ns = "range_vision_fusion";
            marker.id = object.id;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.scale.z = 1.0;
            marker.text = object.label;
            marker.pose.position = object.pose.position;
            marker.pose.position.z += 1.5;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.color.a = 1.0;
            marker.scale.x = 1.5;
            marker.scale.y = 1.5;
            marker.scale.z = 1.5;

            marker.lifetime = ros::Duration(0.1);
            final_markers.markers.push_back(marker);
        }
    }
    return final_markers;
}

jsk_recognition_msgs::BoundingBoxArray
RosRangeVisionFusionApp::ObjectsToBoxes(const autoware_msgs::DetectedObjectArray &in_objects)
{
    jsk_recognition_msgs::BoundingBoxArray final_boxes;
    final_boxes.header = in_objects.header;

    for(const autoware_msgs::DetectedObject& object : in_objects.objects)
    {
        jsk_recognition_msgs::BoundingBox box;

        box.header = in_objects.header;
        box.label = object.id;
        box.dimensions = object.dimensions;
        box.pose = object.pose;
        box.value = object.score;

        final_boxes.boxes.push_back(box);
    }
    return final_boxes;
}

void
RosRangeVisionFusionApp::VisionDetectionsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr &in_vision_detections)
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
RosRangeVisionFusionApp::RangeDetectionsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr &in_range_detections)
{
    if (!processing_ && !in_range_detections->objects.empty())
    {
        processing_ = true;
        range_detections_ = in_range_detections;
        SyncedDetectionsCallback(vision_detections_, in_range_detections);
        processing_ = false;
    }
}

void RosRangeVisionFusionApp::ImageCallback(const sensor_msgs::Image::ConstPtr &in_image_msg)
{
    if(!camera_info_ok_)
        return;
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(in_image_msg, "bgr8");
    cv::Mat in_image = cv_image->image;

    cv::Mat undistorted_image;
    cv::undistort(in_image, image_, camera_instrinsics_, distortion_coefficients_);
};

void
RosRangeVisionFusionApp::IntrinsicsCallback(const sensor_msgs::CameraInfo &in_message)
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
RosRangeVisionFusionApp::FindTransform(const std::string &in_target_frame, const std::string &in_source_frame)
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
RosRangeVisionFusionApp::InitializeRosIo(ros::NodeHandle &in_private_handle)
{
    //get params
    std::string camera_info_src, detected_objects_vision;
    std::string detected_objects_range, fused_topic_str = "/detection/combined_objects", fused_boxes_str = "/detection/combined_objects_boxes";
    std::string fused_text_str = "detection/combined_objects_labels";
    std::string name_space_str = ros::this_node::getNamespace();
    bool sync_topics = false;

    ROS_INFO("[%s] This node requires: Registered TF(Lidar-Camera), CameraInfo, Vision and Range Detections being published.", __APP_NAME__);
    in_private_handle.param<std::string>("detected_objects_range", detected_objects_range, "/detection/lidar_objects");
    ROS_INFO("[%s] detected_objects_range: %s", __APP_NAME__, detected_objects_range.c_str());

    in_private_handle.param<std::string>("detected_objects_vision", detected_objects_vision, "/detection/vision_objects");
    ROS_INFO("[%s] detected_objects_vision: %s", __APP_NAME__, detected_objects_vision.c_str());

    in_private_handle.param<std::string>("camera_info_src", camera_info_src, "/camera_info");
    ROS_INFO("[%s] camera_info_src: %s", __APP_NAME__, camera_info_src.c_str());

    in_private_handle.param<double>("overlap_threshold", overlap_threshold_, 0.5);
    ROS_INFO("[%s] overlap_threshold: %f", __APP_NAME__, overlap_threshold_);


    in_private_handle.param<bool>("sync_topics", sync_topics, false);
    ROS_INFO("[%s] sync_topics: %d", __APP_NAME__, sync_topics);

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
                                                         &RosRangeVisionFusionApp::IntrinsicsCallback, this);

    /*image_subscriber_ = in_private_handle.subscribe("/image_raw",
                                                    1,
                                                    &RosRangeVisionFusionApp::ImageCallback, this);*/

    ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, detected_objects_vision.c_str());
    ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, detected_objects_range.c_str());
    if (!sync_topics)
    {
        detections_range_subscriber_ = in_private_handle.subscribe(detected_objects_vision,
                                                                   1,
                                                                   &RosRangeVisionFusionApp::VisionDetectionsCallback, this);

        detections_vision_subscriber_ = in_private_handle.subscribe(detected_objects_range,
                                                                    1,
                                                                    &RosRangeVisionFusionApp::RangeDetectionsCallback, this);
    }
    else
    {
        vision_filter_subscriber_ = new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>(node_handle_,
                                                                                                        detected_objects_vision, 1);
        range_filter_subscriber_ = new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>(node_handle_,
                                                                                                        detected_objects_range, 1);
        detections_synchronizer_ =
                new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(10),
                                                               *vision_filter_subscriber_,
                                                               *range_filter_subscriber_);
        detections_synchronizer_->registerCallback(boost::bind(&RosRangeVisionFusionApp::SyncedDetectionsCallback, this, _1, _2));
    }

    publisher_fused_objects_ = node_handle_.advertise<autoware_msgs::DetectedObjectArray>(fused_topic_str, 1);
    publisher_fused_boxes_ = node_handle_.advertise<jsk_recognition_msgs::BoundingBoxArray>(fused_boxes_str, 1);
    publisher_fused_text_ = node_handle_.advertise<visualization_msgs::MarkerArray>(fused_text_str, 1);

    ROS_INFO("[%s] Publishing fused objects in %s", __APP_NAME__, fused_topic_str.c_str());
    ROS_INFO("[%s] Publishing fused boxes in %s", __APP_NAME__, fused_boxes_str.c_str());

}


void
RosRangeVisionFusionApp::Run()
{
    ros::NodeHandle private_node_handle("~");
    tf::TransformListener transform_listener;

    transform_listener_ = &transform_listener;

    InitializeRosIo(private_node_handle);

    ROS_INFO("[%s] Ready. Waiting for data...", __APP_NAME__);

    ros::spin();

    ROS_INFO("[%s] END", __APP_NAME__);
}

RosRangeVisionFusionApp::RosRangeVisionFusionApp()
{
    camera_lidar_tf_ok_ = false;
    camera_info_ok_ = false;
    processing_ = false;
    image_frame_id_ = "";
    overlap_threshold_ = 0.5;
    empty_frames_ = 0;
}