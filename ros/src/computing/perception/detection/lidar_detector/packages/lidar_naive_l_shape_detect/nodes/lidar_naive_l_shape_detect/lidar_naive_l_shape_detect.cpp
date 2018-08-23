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

//
#include <pcl_conversions/pcl_conversions.h>
//
#include <tf/transform_datatypes.h>
//

#include "lidar_naive_l_shape_detect.h"



LShapeFilter::LShapeFilter() {
  roi_m_ = 120;
  pic_scale_ = 1800 / roi_m_;
  random_points_ = 80;

  // l-shape fitting params
  slope_dist_thres_ = 2.0;
  num_points_thres_ = 10;

  sensor_height_ = 2.35;

  sub_object_array_ = node_handle_.subscribe("/detection/lidar_objects", 1, &LShapeFilter::callback, this);
  pub_object_array_ = node_handle_.advertise<autoware_msgs::DetectedObjectArray>("/detection/lidar_objects/l_shaped", 1);
}

void LShapeFilter::callback(const autoware_msgs::DetectedObjectArray& input) {
  autoware_msgs::DetectedObjectArray out_objects;
  autoware_msgs::DetectedObjectArray copy_objects;
  copy_objects = input;
  getLShapeBB(copy_objects, out_objects);
  out_objects.header = input.header;
  pub_object_array_.publish(out_objects);
}

void LShapeFilter::getPointsInPointcloudFrame(cv::Point2f rect_points[],
                                       std::vector<cv::Point2f> &pointcloud_frame_points,
                                       const cv::Point& offset_point) {
  // loop 4 rect points
  for (int point_i = 0; point_i < 4; point_i++)
  {
    cv::Point2f offset_point_float;
    offset_point_float = static_cast<cv::Point2f>(offset_point);

    cv::Point2f reverse_offset_point = rect_points[point_i] - offset_point_float;
    // reverse from image coordinate to eucledian coordinate
    float r_x = reverse_offset_point.x;
    float r_y = pic_scale_ * roi_m_ - reverse_offset_point.y;
    cv::Point2f eucledian_coordinate_pic_point(r_x, r_y);
    // reverse to roi_m_*roi_m_ scale
    cv::Point2f offset_pointcloud_point = eucledian_coordinate_pic_point/ pic_scale_;
    // reverse from (0 < x,y < roi_m_) to (roi_m_/2 < x,y < roi_m_/2)
    cv::Point2f offset_vec_(roi_m_ / 2, roi_m_ / 2);
    cv::Point2f pointcloud_point = offset_pointcloud_point - offset_vec_;
    pointcloud_frame_points[point_i] = pointcloud_point;
  }
}


void LShapeFilter::updateCpFromPoints(const std::vector<cv::Point2f>& pointcloud_frame_points,
                                       autoware_msgs::DetectedObject &object) {
  cv::Point2f p1 = pointcloud_frame_points[0];
  cv::Point2f p2 = pointcloud_frame_points[1];
  cv::Point2f p3 = pointcloud_frame_points[2];
  cv::Point2f p4 = pointcloud_frame_points[3];

  double s1 =
      ((p4.x - p2.x) * (p1.y - p2.y) - (p4.y - p2.y) * (p1.x - p2.x)) / 2;
  double s2 =
      ((p4.x - p2.x) * (p2.y - p3.y) - (p4.y - p2.y) * (p2.x - p3.x)) / 2;
  double cx = p1.x + (p3.x - p1.x) * s1 / (s1 + s2);
  double cy = p1.y + (p3.y - p1.y) * s1 / (s1 + s2);

  // std::cout << "cp from euclidean cluster " << cluster.bounding_box.pose.position.x << " "<<cluster.bounding_box.pose.position.y<<std::endl;
  // std::cout << "cp from l shape "<<cx << " "<< cy << std::endl;
  object.pose.position.x = cx;
  object.pose.position.y = cy;
  object.pose.position.z = -sensor_height_ / 2;
}

void LShapeFilter::toRightAngleBBox(std::vector<cv::Point2f> &pointcloud_frame_points)
{
  cv::Point2f p1 = pointcloud_frame_points[0];
  cv::Point2f p2 = pointcloud_frame_points[1];
  cv::Point2f p3 = pointcloud_frame_points[2];
  // cv::Point2f p4 = pointcloud_frame_points[3];

  double vec1x = p2.x - p1.x;
  double vec1y = p2.y - p1.y;
  double vec2x = p3.x - p2.x;
  double vec2y = p3.y - p2.y;

  // from the equation of inner product
  double cos_theta =
      (vec1x * vec2x + vec1y * vec2y) / (sqrt(vec1x * vec1x + vec2x * vec2x) +
                                         sqrt(vec1y * vec1y + vec2y * vec2y));
  double theta = acos(cos_theta);
  double diff_theta = theta - M_PI / 2;

  if (abs(diff_theta) > 0.1) {
    double m1 = vec1y / vec1x;
    double b1 = p3.y - m1 * p3.x;
    double m2 = -1.0 / m1;
    double b2 = p2.y - (m2 * p2.x);

    double x = (b2 - b1) / (m1 - m2);
    double y = (b2 * m1 - b1 * m2) / (m1 - m2);

    double delta_x = x - p2.x;
    double delta_y = y - p2.y;

    pointcloud_frame_points[2].x = x;
    pointcloud_frame_points[2].y = y;
    pointcloud_frame_points[3].x = pointcloud_frame_points[0].x + delta_x;
    pointcloud_frame_points[3].y = pointcloud_frame_points[0].y + delta_y;
  }
}

void LShapeFilter::updateDimentionAndEstimatedAngle(
    const std::vector<cv::Point2f>& pointcloud_frame_points, autoware_msgs::DetectedObject &object) {

  cv::Point2f p1 = pointcloud_frame_points[0];
  cv::Point2f p2 = pointcloud_frame_points[1];
  cv::Point2f p3 = pointcloud_frame_points[2];

  double dist1 = sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
  double dist2 = sqrt((p3.x - p2.x) * (p3.x - p2.x) + (p3.y - p2.y) * (p3.y - p2.y));
  double bb_yaw;
  // dist1 is length
  if (dist1 > dist2) {
    bb_yaw = atan2(p1.y - p2.y, p1.x - p2.x);
    object.dimensions.x = dist1;
    object.dimensions.y = dist2;
    object.dimensions.z = 2;
  } else {
    bb_yaw = atan2(p3.y - p2.y, p3.x - p2.x);
    object.dimensions.x = dist2;
    object.dimensions.y = dist1;
    object.dimensions.z = 2;
  }
  // convert yaw to quartenion
  tf::Matrix3x3 obs_mat;
  obs_mat.setEulerYPR(bb_yaw, 0, 0);

  tf::Quaternion q_tf;
  obs_mat.getRotation(q_tf);
  object.pose.orientation.x = q_tf.getX();
  object.pose.orientation.y = q_tf.getY();
  object.pose.orientation.z = q_tf.getZ();
  object.pose.orientation.w = q_tf.getW();
}

void LShapeFilter::getLShapeBB(
    autoware_msgs::DetectedObjectArray&  in_object_array,
    autoware_msgs::DetectedObjectArray& out_object_array) {


  out_object_array.header = in_object_array.header;

  for (size_t i_object = 0; i_object < in_object_array.objects.size();
       i_object++) {

    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Convert from ros msg to PCL::PointCloud data type
    pcl::fromROSMsg(in_object_array.objects[i_object].pointcloud, cloud);

    // calculating offset so that projecting pointcloud into cv::mat
    cv::Mat m(pic_scale_ * roi_m_, pic_scale_ * roi_m_, CV_8UC1, cv::Scalar(0));
    float init_px = cloud[0].x + roi_m_ / 2;
    float init_py = cloud[0].y + roi_m_ / 2;
    int init_x = floor(init_px * pic_scale_);
    int init_y = floor(init_py * pic_scale_);
    int init_pic_x = init_x;
    int init_pic_y = pic_scale_ * roi_m_ - init_y;
    int offset_init_x = roi_m_ * pic_scale_ / 2 - init_pic_x;
    int offset_init_y = roi_m_ * pic_scale_ / 2 - init_pic_y;

    int num_points = cloud.size();
    std::vector<cv::Point> point_vec(num_points);
    std::vector<cv::Point2f> pointcloud_frame_points(4);


    float min_mx = 0;
    float min_my = 0;
    float max_mx = 0;
    float max_my = 0;
    float min_m = 999;
    float max_m = -999;
    float max_z = -99;

    // for center of gravity
    for (int i_point = 0; i_point < num_points; i_point++) {
      float p_x = cloud[i_point].x;
      float p_y = cloud[i_point].y;
      float p_z = cloud[i_point].z;
      // cast (roi_m_/2 < x,y < roi_m_/2) into (0 < x,y < roi_m_)
      float roi_x = p_x + roi_m_ / 2;
      float roi_y = p_y + roi_m_ / 2;
      // cast (roi_m_)mx(roi_m_)m into 900x900 scale
      int x = floor(roi_x * pic_scale_);
      int y = floor(roi_y * pic_scale_);
      // cast into image coordinate
      int pic_x = x;
      int pic_y = pic_scale_ * roi_m_ - y;
      // offset so that the object would be locate at the center
      int offset_x = pic_x + offset_init_x;
      int offset_y = pic_y + offset_init_y;

      //Make sure points are inside the image size
      if(offset_x > (pic_scale_ * roi_m_) ||
         offset_x < 0                     ||
         offset_y < 0                     ||
         offset_y > (pic_scale_ * roi_m_)){
        // std::cout << offset_x <<" "<<offset_y <<" are not in the image coordinate" << std::endl;
        continue;
      }
      // cast the pointcloud into cv::mat
      m.at<uchar>(offset_y, offset_x) = 255;
      point_vec[i_point] = cv::Point(offset_x, offset_y);
      // calculate min and max slope for x1, x3(edge points)
      float delta_m = p_y / p_x;
      if (delta_m < min_m) {
        min_m = delta_m;
        min_mx = p_x;
        min_my = p_y;
      }

      if (delta_m > max_m) {
        max_m = delta_m;
        max_mx = p_x;
        max_my = p_y;
      }

      // get maxZ
      if (p_z > max_z)
      {
        max_z = p_z;
      }

    }
    // L shape fitting parameters
    float x_dist = max_mx - min_mx;
    float y_dist = max_my - min_my;
    float slope_dist = sqrt(x_dist * x_dist + y_dist * y_dist);
    float slope = (max_my - min_my) / (max_mx - min_mx);

    std::mt19937 mt{ std::random_device{}() };
    mt.seed(in_object_array.header.stamp.toSec());
    std::uniform_int_distribution<int> random_points(0, num_points - 1);

    // start l shape fitting for car like object
    if (slope_dist > slope_dist_thres_ && num_points > num_points_thres_) {
      float max_dist = 0;
      float max_dx = 0;
      float max_dy = 0;

      // 80 random points, get max distance
      for (int i = 0; i < random_points_; i++) {
        int p_ind = random_points(mt);
        float x_i = cloud[p_ind].x;
        float y_i = cloud[p_ind].y;

        // from equation of distance between line and point
        float dist = std::abs(slope * x_i - 1 * y_i + max_my - slope * max_mx) /
                     std::sqrt(slope * slope + 1);
        if (dist > max_dist) {
          max_dist = dist;
          max_dx = x_i;
          max_dy = y_i;
        }
      }


      // vector adding
      float max_m_vec_x = max_mx - max_dx;
      float max_m_vec_y = max_my - max_dy;
      float min_m_vec_x = min_mx - max_dx;
      float min_m_vec_y = min_my - max_dy;
      float last_x = max_dx + max_m_vec_x + min_m_vec_x;
      float last_y = max_dy + max_m_vec_y + min_m_vec_y;

      pointcloud_frame_points[0] = cv::Point2f(min_mx, min_my);
      pointcloud_frame_points[1] = cv::Point2f(max_dx, max_dy);
      pointcloud_frame_points[2] = cv::Point2f(max_mx, max_my);
      pointcloud_frame_points[3] = cv::Point2f(last_x, last_y);


    } else {
      // MAR fitting
      cv::RotatedRect rect_info = cv::minAreaRect(point_vec);
      cv::Point2f rect_points[4];
      rect_info.points(rect_points);
      // covert points back to lidar coordinate
      cv::Point offset_init_p(offset_init_x, offset_init_y);
      getPointsInPointcloudFrame(rect_points, pointcloud_frame_points, offset_init_p);
    }

    updateCpFromPoints(pointcloud_frame_points, in_object_array.objects[i_object]);
    // update pcPoints to make it right angle bbox
    toRightAngleBBox(pointcloud_frame_points);

    updateDimentionAndEstimatedAngle(pointcloud_frame_points,
                                     in_object_array.objects[i_object]);

    out_object_array.objects.push_back(in_object_array.objects[i_object]);
  }
}
