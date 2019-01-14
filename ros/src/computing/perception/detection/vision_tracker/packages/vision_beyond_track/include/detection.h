/*
 *  Copyright (c) 2018, Tokyo University
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
 *  v1.0: Yuki Tsuji (yukitsuji020832@gmail.com)
 *
 *  Created on: Aug 8th, 2018
 */

#ifndef BEYOND_DETECTION_H
#define BEYOND_DETECTION_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "clipper.hpp"

using namespace ClipperLib;

#define deg2rad(a) ((a)/180.0 * M_PI)

namespace beyondtrack
{
  typedef struct ObjectCuboid_
  {
    double avg_l, avg_h, avg_w;
    double sz_ub_l, sz_ub_h, sz_ub_w;
    double sz_lb_l, sz_lb_h, sz_lb_w;

    ObjectCuboid_(double avg_car_sz[3], double sz_ub[3], double sz_lb[3])
    {
      avg_l = avg_car_sz[0];
      avg_h = avg_car_sz[1];
      avg_w = avg_car_sz[2];
      sz_ub_l = sz_ub[0];
      sz_ub_h = sz_ub[1];
      sz_ub_w = sz_ub[2];
      sz_lb_l = sz_lb[0];
      sz_lb_h = sz_lb[1];
      sz_lb_w = sz_lb[2];
    }

  } ObjectCuboid;

  class Detection
  {

  public:
    // average car parameters in meters [l, h, w];
    static ObjectCuboid params_car_cuboid_;
    // static ObjectCuboid params_PersonCuboid;

    int object_id_;
    double bbox_[4];
    double yaw_;
    std::string class_type_;
    cv::Mat sigma_3D;
    cv::Mat B1Q;
    cv::Mat origin_;
    cv::Mat bvolume_;
    cv::Mat bvolume_proj_;

    double cd_area_;
    cv::Mat cd_2d_;
    std::vector<cv::Point> pd_2d_convhull_;
    Paths pd_2d_convhull_clip_;
    Paths cd_2d_convhull_clip_;

    std::vector<cv::Point> pd_3d_convhull_;
    std::vector<cv::Point> cd_3d_convhull_;
    Paths pd_3d_convhull_clip_;
    Paths cd_3d_convhull_clip_;

    Detection(int x, int y, int width, int height, std::string class_type)
    {
      object_id_ = -1;
      bbox_[0] = (double) x;
      bbox_[1] = (double) y;
      bbox_[2] = (double) (x + width);
      bbox_[3] = (double) (y + height);
      //std::cout << "bbox_: " << bbox_[0] << '\t' << bbox_[1] << '\t' << bbox_[2] << '\t' << bbox_[3] << '\n';
      yaw_ = deg2rad(-90);
      sigma_3D = (cv::Mat_<double>(4, 4) <<
                                         1.3, 0, 0, 0, 0, 1.1, 0, 0, 0, 0, 1.1, 0, 0, 0, 0, deg2rad(0));
      class_type_ = class_type;
    }

    Detection(std::vector<double> det, std::string class_type)
    {
      object_id_ = -1;
      bbox_[0] = det[0];
      bbox_[1] = det[1];
      bbox_[2] = det[2];
      bbox_[3] = det[3];
      yaw_ = deg2rad(-90);
      sigma_3D = (cv::Mat_<double>(4, 4) <<
                                         1.3, 0, 0, 0, 0, 1.1, 0, 0, 0, 0, 1.1, 0, 0, 0, 0, deg2rad(0));
      class_type_ = class_type;
    }

    void propagate_prev_det(cv::Mat k, cv::Mat motion)
    {
      // std::cout << "### Propagate previous detection　###\n";
      B1Q = origin_;
      cv::Mat bvolume = bvolume_;
      B1Q = B1Q + motion.colRange(0, 3).t();
      bvolume += repeat(motion.colRange(0, 3), bvolume.size().height, 1);
      bvolume -= repeat(B1Q.t(), bvolume.size().height, 1);
      bvolume = get_boundingbox_volume(bvolume, motion.at<double>(0, 3)); //translated_cuboid);
      origin_ = B1Q;
      bvolume_ = bvolume;
      bvolume_proj_ = bvolume_ * k.t();
      bvolume_proj_.colRange(0, 3) /= repeat(bvolume_proj_.col(2), 1, 3);

      // Pre-calculate the convexhull -- 3d --
      cv::Mat pd_3d_xz = cv::Mat(bvolume_.size().height, 2, CV_64FC1);
      bvolume_.col(0).copyTo(pd_3d_xz.col(0));
      bvolume_.col(2).copyTo(pd_3d_xz.col(1));
      pd_3d_xz.convertTo(pd_3d_xz, CV_32FC1);

      pd_3d_convhull_ = get_convhull(pd_3d_xz * 100); // due to cast

      Paths pd_3d_convhull_clip(1);
      for (const auto &e: pd_3d_convhull_)
      {
        pd_3d_convhull_clip[0] << IntPoint(e.x, e.y);
      }
      pd_3d_convhull_clip_ = pd_3d_convhull_clip;

      // Pre-calculate the convexhull -- 2d --
      cv::Mat pd_2d = bvolume_proj_.colRange(0, 2);
      pd_2d.convertTo(pd_2d, CV_32FC1);

      pd_2d_convhull_ = get_convhull(pd_2d);

      Paths pd_2d_convhull_clip(1);
      for (const auto &e: pd_2d_convhull_)
      {
        pd_2d_convhull_clip[0] << IntPoint(e.x, e.y);
      }
      pd_2d_convhull_clip_ = pd_2d_convhull_clip;
    }

    void propagate_cur_det(cv::Mat cuboid, double h, cv::Mat k, cv::Mat inv_k, cv::Mat n)
    {
      // std::cout << "Propagate current detection\n";
      cv::Mat b1Q = (cv::Mat_<double>(3, 1) << bbox_[0] + (bbox_[2] - bbox_[0]) / 2, bbox_[3], 1.0);
      B1Q = (h * inv_k * b1Q) / (n * inv_k * b1Q);
      // apply offset which is a function of yaw and get car's origin.
      double offset_z = calc_offset_base_yaw();
      B1Q.at<double>(0, 2) += offset_z;
      cv::Mat bvolume = get_boundingbox_volume(cuboid, yaw_); //translated_cuboid);
      origin_ = B1Q;
      cv::Mat offset = (cv::Mat_<double>(1, 3) << 0, params_car_cuboid_.avg_w / 2., 0);
      bvolume_ = bvolume - repeat(offset, bvolume.size().height, 1);
      bvolume_proj_ = bvolume_ * k.t();
      bvolume_proj_.colRange(0, 3) /= repeat(bvolume_proj_.col(2), 1, 3);

      // Pre-calculate the convexhull -- 3d --
      cv::Mat bvolume_xy = cv::Mat(bvolume_.size().height, 2, CV_64FC1);
      bvolume_.col(0).copyTo(bvolume_xy.col(0));
      bvolume_.col(2).copyTo(bvolume_xy.col(1));
      bvolume_xy.convertTo(bvolume_xy, CV_32FC1);

      cd_3d_convhull_ = get_convhull(bvolume_xy * 100); // due to cast

      Paths cd_3d_convhull_clip(1);
      for (const auto &e: cd_3d_convhull_)
      {
        cd_3d_convhull_clip[0] << IntPoint(e.x, e.y);
      }
      cd_3d_convhull_clip_ = cd_3d_convhull_clip;

      // Pre-calculate the convexhull -- 2d --
      cd_2d_ = (cv::Mat_<double>(4, 2) <<
                                       bbox_[0], bbox_[1], bbox_[2], bbox_[1], bbox_[2], bbox_[3], bbox_[0], bbox_[3]);

      Paths cd_2d_convhull_clip(1);
      for (int i = 0; i < 4; ++i)
      {
        cd_2d_convhull_clip[0] << IntPoint((int) cd_2d_.at<double>(i, 0), (int) cd_2d_.at<double>(i, 1));
      }
      cd_2d_convhull_clip_ = cd_2d_convhull_clip;

      cd_area_ = (bbox_[3] - bbox_[1]) * (bbox_[2] - bbox_[0]);
    }

  private:

    double calc_offset_base_yaw()
    {
      double offset;
      double l = params_car_cuboid_.avg_l / 2.;
      double w = params_car_cuboid_.avg_w / 2.;
      cv::Mat car_bottom_plane = (cv::Mat_<double>(4, 2) << -l, -w, l, -w, l, w, -l, w);
      car_bottom_plane = car_bottom_plane.t();

      cv::Mat rot = (cv::Mat_<double>(2, 2) << cos(yaw_), -sin(yaw_), sin(yaw_), cos(yaw_));
      cv::Mat car_bottom_plane_rot = (rot * car_bottom_plane).t();
      cv::Mat sum_mat;
      cv::reduce(car_bottom_plane_rot, sum_mat, 1, CV_REDUCE_SUM);
      double min_val, max_val;
      CvPoint min_loc = cvPoint(0, 0);
      CvPoint max_loc = cvPoint(0, 0);
      CvMat cvmat = sum_mat;
      cvMinMaxLoc(&cvmat, &min_val, &max_val, &min_loc, &max_loc);
      offset =
        abs(car_bottom_plane_rot.at<double>(max_loc.y, 1) - car_bottom_plane_rot.at<double>(min_loc.y, 1)) /
        2.;
      return offset;
    }

    cv::Mat get_boundingbox_volume(cv::Mat cuboid, double ry)
    {
      double ry_n = sigma_3D.at<double>(3, 3);
      cv::Mat centered_pts = cuboid; //cuboid - repeat(B1Q.t(), 8, 1);
      // ry_n = 0.5;
      cv::Mat rot_pts_plus_yaw = centered_pts * rot_mat_y(ry_n).t();
      cv::Mat rot_pts_minus_yaw = centered_pts * rot_mat_y(-ry_n).t();
      cv::Mat pts;
      vconcat(centered_pts, rot_pts_plus_yaw, pts);
      vconcat(pts, rot_pts_minus_yaw, pts);

      cv::Mat scale_mat = sigma_3D(cv::Range(0, 3), cv::Range(0, 3));
      cv::Mat bvolume = pts * scale_mat;
      // cv::Mat m_bvolume = -bvolume;
      // cv::cvThreshold(bvolume, bvolume);

      bvolume = bvolume * rot_mat_y(ry).t();
      bvolume = bvolume + repeat(B1Q.t(), bvolume.size().height, 1);
      return bvolume;
    }

    std::vector<cv::Point> get_convhull(cv::Mat mat_2d)
    {
      int size_pd = mat_2d.size().height;
      std::vector<cv::Point> mat_2d_convhull(1);
      std::vector<cv::Point> mat_2d_vec;
      for (int i = 0; i < size_pd; ++i)
      {
        cv::Mat a = mat_2d.row(i);
        cv::Point b = (cv::Point) a;
        mat_2d_vec.push_back(b);
      }

      cv::convexHull(mat_2d_vec, mat_2d_convhull, false);
      return mat_2d_convhull;
    }

    cv::Mat rot_mat_x(double r)
    {
      return (cv::Mat_<double>(3, 3) << 1.0, 0.0, 0.0,
        0.0, cos(r), -sin(r),
        0.0, sin(r), cos(r));
    }

    cv::Mat rot_mat_y(double r)
    {
      return (cv::Mat_<double>(3, 3) << cos(r), 0.0, sin(r),
        0.0, 1.0, 0.0,
        -sin(r), 0.0, cos(r));
    }

    cv::Mat rot_mat_z(double r)
    {
      return (cv::Mat_<double>(3, 3) << cos(r), -sin(r), 0.0,
        sin(r), cos(r), 0.0,
        0.0, 0.0, 1.0);
    }
  };
}

#endif
