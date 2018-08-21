#include <pcl_conversions/pcl_conversions.h>
#include <random>

#include <tf/transform_datatypes.h>

#include "lidar_naive_l_shape_detect.h"

LShapeFilter::LShapeFilter()
{
  // l-shape fitting params
  ros::NodeHandle private_nh_("~");
  private_nh_.param<int>("ram_points", ram_points_, 80);
  private_nh_.param<float>("slope_dist_thres", slope_dist_thres_, 2.0);
  private_nh_.param<int>("num_points_thres", num_points_thres_, 10);
  private_nh_.param<float>("sensor_height", sensor_height_, 2.35);

  // Assuming pointcloud x and y range within roi_m_: 0 < x, y < roi_m_
  private_nh_.param<float>("roi_m_", roi_m_, 120);
  // Scale roi_m*roi_m_ to pic_scale_ times: will end up cv::Mat<roi_m_*pic_scale_, roi_m_*pic_scale_>
  // in order to make MinAreaRect algo would work well
  private_nh_.param<float>("pic_scale_", pic_scale_, 15);

  sub_object_array_ = node_handle_.subscribe("/detection/lidar_objects", 1, &LShapeFilter::callback, this);
  pub_object_array_ =
      node_handle_.advertise<autoware_msgs::DetectedObjectArray>("/detection/lidar_objects/l_shaped", 1);
}

void LShapeFilter::callback(const autoware_msgs::DetectedObjectArray& input)
{
  autoware_msgs::DetectedObjectArray out_objects;
  autoware_msgs::DetectedObjectArray copy_objects;
  copy_objects = input;
  getLShapeBB(copy_objects, out_objects);
  out_objects.header = input.header;
  pub_object_array_.publish(out_objects);
}

void LShapeFilter::getPointsInPcFrame(cv::Point2f rect_points[], std::vector<cv::Point2f>& pc_points, int offset_x,
                                      int offset_y)
{
  // loop 4 rect points
  for (int point_i = 0; point_i < 4; point_i++)
  {
    float pic_x = rect_points[point_i].x;
    float pic_y = rect_points[point_i].y;
    // reverse offset
    float r_offset_x = pic_x - offset_x;
    float r_offset_y = pic_y - offset_y;
    // reverse from image coordinate to eucledian coordinate
    float r_x = r_offset_x;
    float r_y = pic_scale_ * roi_m_ - r_offset_y;
    // reverse to roi_m_*roi_m_ scale
    float rm_x = r_x / pic_scale_;
    float rm_y = r_y / pic_scale_;
    // reverse from (0 < x,y < roi_m_) to (roi_m_/2 < x,y < roi_m_/2)
    float pc_x = rm_x - roi_m_ / 2;
    float pc_y = rm_y - roi_m_ / 2;
    cv::Point2f point(pc_x, pc_y);
    pc_points[point_i] = point;
  }
}

void LShapeFilter::updateCpFromPoints(const std::vector<cv::Point2f>& pc_points, autoware_msgs::DetectedObject& object)
{
  cv::Point2f p1 = pc_points[0];
  cv::Point2f p2 = pc_points[1];
  cv::Point2f p3 = pc_points[2];
  cv::Point2f p4 = pc_points[3];

  double s1 = ((p4.x - p2.x) * (p1.y - p2.y) - (p4.y - p2.y) * (p1.x - p2.x)) / 2;
  double s2 = ((p4.x - p2.x) * (p2.y - p3.y) - (p4.y - p2.y) * (p2.x - p3.x)) / 2;
  double cx = p1.x + (p3.x - p1.x) * s1 / (s1 + s2);
  double cy = p1.y + (p3.y - p1.y) * s1 / (s1 + s2);

  object.pose.position.x = cx;
  object.pose.position.y = cy;
  object.pose.position.z = -sensor_height_ / 2;
}

void LShapeFilter::toRightAngleBBox(std::vector<cv::Point2f>& pc_points)
{
  cv::Point2f p1 = pc_points[0];
  cv::Point2f p2 = pc_points[1];
  cv::Point2f p3 = pc_points[2];

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

    pc_points[2].x = x;
    pc_points[2].y = y;
    pc_points[3].x = pc_points[0].x + delta_x;
    pc_points[3].y = pc_points[0].y + delta_y;
  }
}

void LShapeFilter::updateDimentionAndEstimatedAngle(const std::vector<cv::Point2f>& pc_points,
                                                    autoware_msgs::DetectedObject& object)
{
  // p1-p2 and p2-p3 is line segment, p1-p3 is diagonal
  cv::Point2f p1 = pc_points[0];
  cv::Point2f p2 = pc_points[1];
  cv::Point2f p3 = pc_points[2];

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

void LShapeFilter::getLShapeBB(autoware_msgs::DetectedObjectArray& in_object_array,
                               autoware_msgs::DetectedObjectArray& out_object_array)
{
  out_object_array.header = in_object_array.header;

  for (size_t i_object = 0; i_object < in_object_array.objects.size(); i_object++)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Convert from ros msg to PCL::pic_scalePointCloud data type
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
    std::vector<cv::Point2f> pc_points(4);

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
      float roi_x = p_x + roi_m_ / 2;
      float roi_y = p_y + roi_m_ / 2;
      // cast (roi_m_)m*(roi_m_)m into  pic_scale_
      int x = floor(roi_x * pic_scale_);
      int y = floor(roi_y * pic_scale_);
      // cast into image coordinate
      int pic_x = x;
      int pic_y = pic_scale_ * roi_m_ - y;
      // offset so that the object would be locate at the center
      int offset_x = pic_x + offset_init_x;
      int offset_y = pic_y + offset_init_y;

      // Make sure points are inside the image size
      if (offset_x > (pic_scale_ * roi_m_) || offset_x < 0 || offset_y < 0 || offset_y > (pic_scale_ * roi_m_))
      {
        continue;
      }
      // cast the pointcloud into cv::mat
      m.at<uchar>(offset_y, offset_x) = 255;
      point_vec[i_point] = cv::Point(offset_x, offset_y);
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

      // num_random_points, get max distance
      for (int i = 0; i < ram_points_; i++)
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

      pc_points[0] = cv::Point2f(min_mx, min_my);
      pc_points[1] = cv::Point2f(max_dx, max_dy);
      pc_points[2] = cv::Point2f(max_mx, max_my);
      pc_points[3] = cv::Point2f(last_x, last_y);
    }
    else
    {
      // MAR fitting
      cv::RotatedRect rect_info = cv::minAreaRect(point_vec);
      cv::Point2f rect_points[4];
      rect_info.points(rect_points);
      // covert points back to lidar coordinate
      getPointsInPcFrame(rect_points, pc_points, offset_init_x, offset_init_y);
    }
    updateCpFromPoints(pc_points, in_object_array.objects[i_object]);

    // update pc_points to make it right angle bbox
    toRightAngleBBox(pc_points);

    updateDimentionAndEstimatedAngle(pc_points, in_object_array.objects[i_object]);

    out_object_array.objects.push_back(in_object_array.objects[i_object]);
  }
}
