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

#include <pcl_conversions/pcl_conversions.h>
#include <random>

#include <tf/transform_datatypes.h>

#include "lidar_naive_l_shape_detect.h"

LShapeFilter::LShapeFilter()
{
  // l-shape fitting params
  ros::NodeHandle private_nh_("~");
  private_nh_.param<int>("random_ponts", random_points_, 80);
  private_nh_.param<float>("slope_dist_thres", slope_dist_thres_, 2.0);
  private_nh_.param<int>("num_points_thres", num_points_thres_, 10);
  private_nh_.param<float>("sensor_height", sensor_height_, 2.35);

  // Assuming pointcloud x and y range within roi_m_: 0 < x, y < roi_m_
  // Short for region of interest in meters
  private_nh_.param<float>("roi_m_", roi_m_, 120);
  // Scale roi_m*roi_m_ to pic_scale_ times: will end up cv::Mat<roi_m_*pic_scale_, roi_m_*pic_scale_>
  // in order to make fitting algorithm work
  private_nh_.param<float>("pic_scale_", pic_scale_, 15);

  sub_object_array_ = node_handle_.subscribe("/detection/lidar_objects", 1, &LShapeFilter::callback, this);
  pub_object_array_ =
      node_handle_.advertise<autoware_msgs::DetectedObjectArray>("/detection/lidar_objects/l_shaped", 1);
}

void LShapeFilter::callback(const autoware_msgs::DetectedObjectArray& input)
{
  autoware_msgs::DetectedObjectArray out_objects;
  getLShapeBB(input, out_objects);
  out_objects.header = input.header;
  pub_object_array_.publish(out_objects);
}

void LShapeFilter::getPointsInPcFrame(cv::Point2f rect_points[], std::vector<cv::Point2f>& pointcloud_points,
                                      const cv::Point& offset_point)
{
  // loop 4 rect points
  for (int point_i = 0; point_i < 4; point_i++)
  {
    float pic_x = rect_points[point_i].x;
    float pic_y = rect_points[point_i].y;
    cv::Point2f pic_point(pic_x, pic_y);

    cv::Point2f offset_point_float;
    offset_point_float = static_cast<cv::Point2f>(offset_point);

    // reverse offset
    cv::Point2f reverse_offset_point = pic_point - offset_point_float;
    // reverse from image coordinate to eucledian coordinate
    float r_x = reverse_offset_point.x;
    float r_y = pic_scale_ * roi_m_ - reverse_offset_point.y;
    cv::Point2f eucledian_coordinate_pic_point(r_x, r_y);
    // reverse to roi_m_*roi_m_ scale
    cv::Point2f offset_pointcloud_point = eucledian_coordinate_pic_point - reverse_offset_point;
    // reverse from (0 < x,y < roi_m_) to (roi_m_/2 < x,y < roi_m_/2)
    cv::Point2f offset_vec_(roi_m_ / 2, roi_m_ / 2);
    cv::Point2f pointcloud_point = offset_pointcloud_point - offset_vec_;
    pointcloud_points[point_i] = pointcloud_point;
  }
}

void LShapeFilter::updateCpFromPoints(const std::vector<cv::Point2f>& pointcloud_points,
                                      autoware_msgs::DetectedObject& output)
{
  cv::Point2f p1 = pointcloud_points[0];
  cv::Point2f p2 = pointcloud_points[1];
  cv::Point2f p3 = pointcloud_points[2];
  cv::Point2f p4 = pointcloud_points[3];

  double s1 = ((p4.x - p2.x) * (p1.y - p2.y) - (p4.y - p2.y) * (p1.x - p2.x)) / 2;
  double s2 = ((p4.x - p2.x) * (p2.y - p3.y) - (p4.y - p2.y) * (p2.x - p3.x)) / 2;
  double cx = p1.x + (p3.x - p1.x) * s1 / (s1 + s2);
  double cy = p1.y + (p3.y - p1.y) * s1 / (s1 + s2);

  output.pose.position.x = cx;
  output.pose.position.y = cy;
  output.pose.position.z = -sensor_height_ / 2;
}

void LShapeFilter::toRightAngleBBox(std::vector<cv::Point2f>& pointcloud_points)
{
  cv::Point2f p1 = pointcloud_points[0];
  cv::Point2f p2 = pointcloud_points[1];
  cv::Point2f p3 = pointcloud_points[2];

  double vec1x = p2.x - p1.x;
  double vec1y = p2.y - p1.y;
  double vec2x = p3.x - p2.x;
  double vec2y = p3.y - p2.y;

  // from the equation of inner product
  double cos_theta =
      (vec1x * vec2x + vec1y * vec2y) / (sqrt(vec1x * vec1x + vec2x * vec2x) + sqrt(vec1y * vec1y + vec2y * vec2y));
  double theta = acos(cos_theta);
  double diff_theta = theta - M_PI / 2;

  if (abs(diff_theta) > 0.1)
  {
    double m1 = vec1y / vec1x;
    double b1 = p3.y - m1 * p3.x;
    double m2 = -1.0 / m1;
    double b2 = p2.y - (m2 * p2.x);

    double x = (b2 - b1) / (m1 - m2);
    double y = (b2 * m1 - b1 * m2) / (m1 - m2);

    double delta_x = x - p2.x;
    double delta_y = y - p2.y;

    pointcloud_points[2].x = x;
    pointcloud_points[2].y = y;
    pointcloud_points[3].x = pointcloud_points[0].x + delta_x;
    pointcloud_points[3].y = pointcloud_points[0].y + delta_y;
  }
}

void LShapeFilter::updateDimentionAndEstimatedAngle(const std::vector<cv::Point2f>& pointcloud_points,
                                                    autoware_msgs::DetectedObject& object)
{
  // p1-p2 and p2-p3 is line segment, p1-p3 is diagonal
  cv::Point2f p1 = pointcloud_points[0];
  cv::Point2f p2 = pointcloud_points[1];
  cv::Point2f p3 = pointcloud_points[2];

  double dist1 = sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
  double dist2 = sqrt((p3.x - p2.x) * (p3.x - p2.x) + (p3.y - p2.y) * (p3.y - p2.y));
  double bb_yaw;
  // dist1 is length, dist2 is width
  if (dist1 > dist2)
  {
    bb_yaw = atan2(p1.y - p2.y, p1.x - p2.x);
    object.dimensions.x = dist1;
    object.dimensions.y = dist2;
    object.dimensions.z = 2;
  }
  // dist1 is width, dist2 is length
  else
  {
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

void LShapeFilter::getLShapeBB(const autoware_msgs::DetectedObjectArray& in_object_array,
                               autoware_msgs::DetectedObjectArray& out_object_array)
{
  out_object_array.header = in_object_array.header;

  for (const auto& in_object : in_object_array.objects)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Convert from ros msg to PCL::pic_scalePointCloud data type
    pcl::fromROSMsg(in_object.pointcloud, cloud);

    // calculating offset so that projecting pointcloud into cv::mat
    cv::Mat m(pic_scale_ * roi_m_, pic_scale_ * roi_m_, CV_8UC1, cv::Scalar(0));
    cv::Point2f tmp_pointcloud_point(cloud[0].x, cloud[0].y);
    cv::Point2f tmp_pointcloud_offset(roi_m_ / 2, roi_m_ / 2);
    cv::Point2f tmp_offset_pointcloud_point = tmp_pointcloud_point + tmp_pointcloud_offset;
    cv::Point tmp_pic_point = tmp_offset_pointcloud_point * pic_scale_;

    int tmp_init_pic_x = tmp_pic_point.x;
    int tmp_init_pic_y = pic_scale_ * roi_m_ - tmp_pic_point.y;

    cv::Point tmp_init_pic_point(tmp_init_pic_x, tmp_init_pic_y);
    cv::Point tmp_init_offset_vec(roi_m_ * pic_scale_ / 2, roi_m_ * pic_scale_ / 2);
    cv::Point offset_init_pic_point = tmp_init_offset_vec - tmp_init_pic_point;

    int num_points = cloud.size();
    std::vector<cv::Point> point_vec(num_points);
    std::vector<cv::Point2f> pointcloud_points(4);

    // init variables
    float min_mx = 0;
    float min_my = 0;
    float max_mx = 0;
    float max_my = 0;
    float min_m = std::numeric_limits<float>::max();
    float max_m = std::numeric_limits<float>::min();
    float max_z = std::numeric_limits<float>::min();

    for (int i_point = 0; i_point < num_points; i_point++)
    {
      float p_x = cloud[i_point].x;
      float p_y = cloud[i_point].y;
      float p_z = cloud[i_point].z;

      // cast (roi_m_/2 < x,y < roi_m_/2) into (0 < x,y < roi_m_)
      cv::Point2f pointcloud_point(p_x, p_y);
      cv::Point2f pointcloud_offset_vec(roi_m_ / 2, roi_m_ / 2);
      cv::Point2f offset_pointcloud_point = pointcloud_point + pointcloud_offset_vec;
      // cast (roi_m_)m*(roi_m_)m into  pic_scale_
      cv::Point scaled_point = offset_pointcloud_point * pic_scale_;
      // cast into image coordinate
      int pic_x = scaled_point.x;
      int pic_y = pic_scale_ * roi_m_ - scaled_point.y;
      // offset so that the object would be locate at the center
      cv::Point pic_point(pic_x, pic_y);
      cv::Point offset_point = pic_point + offset_init_pic_point;

      // Make sure points are inside the image size
      if (offset_point.x > (pic_scale_ * roi_m_) || offset_point.x < 0 || offset_point.y < 0 ||
          offset_point.y > (pic_scale_ * roi_m_))
      {
        continue;
      }
      // cast the pointcloud into cv::mat
      m.at<uchar>(offset_point.y, offset_point.x) = 255;
      point_vec[i_point] = offset_point;
      // calculate min and max slope for x1, x3(edge points)
      float delta_m = p_y / p_x;
      if (delta_m < min_m)
      {
        min_m = delta_m;
        min_mx = p_x;
        min_my = p_y;
      }

      if (delta_m > max_m)
      {
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

    if (max_m == std::numeric_limits<float>::min() || min_m == std::numeric_limits<float>::max() ||
        max_z == std::numeric_limits<float>::min())
    {
      continue;
    }
    // L shape fitting parameters
    float x_dist = max_mx - min_mx;
    float y_dist = max_my - min_my;
    float slope_dist = sqrt(x_dist * x_dist + y_dist * y_dist);
    float slope = (max_my - min_my) / (max_mx - min_mx);

    // random variable
    std::mt19937_64 mt;
    mt.seed(in_object_array.header.stamp.toSec());
    // mt.seed(0);
    std::uniform_int_distribution<> rand_points(0, num_points - 1);

    // start l shape fitting for car like object
    if (slope_dist > slope_dist_thres_ && num_points > num_points_thres_)
    {
      float max_dist = 0;
      float max_dx = 0;
      float max_dy = 0;

      // get max distance from random sampling points
      for (int i = 0; i < random_points_; i++)
      {
        int p_ind = rand_points(mt);
        assert(p_ind >= 0 && p_ind < (cloud.size() - 1));
        float x_i = cloud[p_ind].x;
        float y_i = cloud[p_ind].y;

        // from equation of distance between line and point
        float dist = abs(slope * x_i - 1 * y_i + max_my - slope * max_mx) / sqrt(slope * slope + 1);
        if (dist > max_dist)
        {
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

      pointcloud_points[0] = cv::Point2f(min_mx, min_my);
      pointcloud_points[1] = cv::Point2f(max_dx, max_dy);
      pointcloud_points[2] = cv::Point2f(max_mx, max_my);
      pointcloud_points[3] = cv::Point2f(last_x, last_y);
    }
    else
    {
      // MinAreaRect fitting
      cv::RotatedRect rect_info = cv::minAreaRect(point_vec);
      cv::Point2f rect_points[4];
      rect_info.points(rect_points);
      // covert points back to lidar coordinate
      getPointsInPcFrame(rect_points, pointcloud_points, offset_init_pic_point);
    }

    autoware_msgs::DetectedObject output_object;
    output_object = in_object;

    // update output_object pose
    updateCpFromPoints(pointcloud_points, output_object);

    // update pointcloud_points to make it right angle bbox
    toRightAngleBBox(pointcloud_points);

    // update output_object dimensions
    updateDimentionAndEstimatedAngle(pointcloud_points, output_object);

    out_object_array.objects.push_back(output_object);
  }
}
