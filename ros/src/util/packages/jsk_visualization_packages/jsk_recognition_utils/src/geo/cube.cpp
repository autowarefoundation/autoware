// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#define BOOST_PARAMETER_MAX_ARITY 7

#include "jsk_recognition_utils/geo/cube.h"
#include "jsk_recognition_utils/geo_util.h"

namespace jsk_recognition_utils
{
    Cube::Cube(const Eigen::Vector3f& pos, const Eigen::Quaternionf& rot):
    pos_(pos), rot_(rot)
  {
    dimensions_.resize(3);
  }

  Cube::Cube(const Eigen::Vector3f& pos, const Eigen::Quaternionf& rot,
             const std::vector<double>& dimensions):
    pos_(pos), rot_(rot), dimensions_(dimensions)
  {
    
  }
  Cube::Cube(const Eigen::Vector3f& pos, const Eigen::Quaternionf& rot,
             const Eigen::Vector3f& dimensions):
    pos_(pos), rot_(rot)
  {
    dimensions_.resize(3);
    dimensions_[0] = dimensions[0];
    dimensions_[1] = dimensions[1];
    dimensions_[2] = dimensions[2];
  }

  Cube::Cube(const Eigen::Vector3f& pos,
             const Line& line_a, const Line& line_b, const Line& line_c)
  {
    double distance_a_b = line_a.distance(line_b);
    double distance_a_c = line_a.distance(line_c);
    double distance_b_c = line_b.distance(line_c);
    Line::Ptr axis;
    dimensions_.resize(3);
    Eigen::Vector3f ex, ey, ez;
    if (distance_a_b >= distance_a_c &&
        distance_a_b >= distance_b_c) {
      axis = line_a.midLine(line_b);
      line_a.parallelLineNormal(line_c, ex);
      line_c.parallelLineNormal(line_b, ey);
      
    }
    else if (distance_a_c >= distance_a_b &&
             distance_a_c >= distance_b_c) {
      axis = line_a.midLine(line_c);
      line_a.parallelLineNormal(line_b, ex);
      line_b.parallelLineNormal(line_c, ey);
    }
    else {
      // else if (distance_b_c >= distance_a_b &&
      //          distance_b_c >= distance_a_c) {
      axis = line_b.midLine(line_c);
      line_b.parallelLineNormal(line_a, ex);
      line_a.parallelLineNormal(line_c, ey);
    }
    dimensions_[0] = ex.norm();
    dimensions_[1] = ey.norm();
    axis->getDirection(ez);
    ez.normalize();
    ex.normalize();
    ey.normalize();
    if (ex.cross(ey).dot(ez) < 0) {
      ez = - ez;
    }
    rot_ = rotFrom3Axis(ex, ey, ez);
    axis->foot(pos, pos_);       // project
  }
  
  Cube::~Cube()
  {

  }

  std::vector<Segment::Ptr> Cube::edges()
  {
    std::vector<Segment::Ptr> ret;
    Eigen::Vector3f A = pos_
      + rot_ * ((+ dimensions_[0] * Eigen::Vector3f::UnitX()) +
                (- dimensions_[1] * Eigen::Vector3f::UnitY()) +
                (+ dimensions_[2] * Eigen::Vector3f::UnitZ())) / 2.0;
    Eigen::Vector3f B = pos_
      + rot_ * ((+ dimensions_[0] * Eigen::Vector3f::UnitX()) +
                (+ dimensions_[1] * Eigen::Vector3f::UnitY()) +
                (+ dimensions_[2] * Eigen::Vector3f::UnitZ())) / 2.0;
    Eigen::Vector3f C = pos_
      + rot_ * ((- dimensions_[0] * Eigen::Vector3f::UnitX()) +
                (+ dimensions_[1] * Eigen::Vector3f::UnitY()) +
                (+ dimensions_[2] * Eigen::Vector3f::UnitZ())) / 2.0;
    Eigen::Vector3f D = pos_
      + rot_ * ((- dimensions_[0] * Eigen::Vector3f::UnitX()) +
                (- dimensions_[1] * Eigen::Vector3f::UnitY()) +
                (+ dimensions_[2] * Eigen::Vector3f::UnitZ())) / 2.0;
    Eigen::Vector3f E = pos_
      + rot_ * ((+ dimensions_[0] * Eigen::Vector3f::UnitX()) +
                (- dimensions_[1] * Eigen::Vector3f::UnitY()) +
                (- dimensions_[2] * Eigen::Vector3f::UnitZ())) / 2.0;
    Eigen::Vector3f F = pos_
      + rot_ * ((+ dimensions_[0] * Eigen::Vector3f::UnitX()) +
                (+ dimensions_[1] * Eigen::Vector3f::UnitY()) +
                (- dimensions_[2] * Eigen::Vector3f::UnitZ())) / 2.0;
    Eigen::Vector3f G = pos_
      + rot_ * ((- dimensions_[0] * Eigen::Vector3f::UnitX()) +
                (+ dimensions_[1] * Eigen::Vector3f::UnitY()) +
                (- dimensions_[2] * Eigen::Vector3f::UnitZ())) / 2.0;
    Eigen::Vector3f H = pos_
      + rot_ * ((- dimensions_[0] * Eigen::Vector3f::UnitX()) +
                (- dimensions_[1] * Eigen::Vector3f::UnitY()) +
                (- dimensions_[2] * Eigen::Vector3f::UnitZ())) / 2.0;
    
    ret.push_back(Segment::Ptr(new Segment(A, B)));
    ret.push_back(Segment::Ptr(new Segment(B, C)));
    ret.push_back(Segment::Ptr(new Segment(C, D)));
    ret.push_back(Segment::Ptr(new Segment(D, A)));
    ret.push_back(Segment::Ptr(new Segment(E, F)));
    ret.push_back(Segment::Ptr(new Segment(F, G)));
    ret.push_back(Segment::Ptr(new Segment(G, H)));
    ret.push_back(Segment::Ptr(new Segment(H, E)));
    ret.push_back(Segment::Ptr(new Segment(A, E)));
    ret.push_back(Segment::Ptr(new Segment(B, F)));
    ret.push_back(Segment::Ptr(new Segment(C, G)));
    ret.push_back(Segment::Ptr(new Segment(D, H)));
    return ret;
  } 
  
  ConvexPolygon::Ptr Cube::intersectConvexPolygon(Plane& plane)
  {
    std::vector<Segment::Ptr> candidate_edges = edges();
    Vertices intersects;
    for (size_t i = 0; i < candidate_edges.size(); i++) {
      Segment::Ptr edge = candidate_edges[i];
      Eigen::Vector3f p;
      if (edge->intersect(plane, p)) {
        intersects.push_back(p);
      }
    }
    //JSK_ROS_INFO("%lu intersects", intersects.size());
    // Compute convex hull
    pcl::ConvexHull<pcl::PointXYZ> chull;
    pcl::PointCloud<pcl::PointXYZ>::Ptr chull_input
      = verticesToPointCloud<pcl::PointXYZ>(intersects);
    pcl::PointCloud<pcl::PointXYZ>::Ptr chull_cloud
      (new pcl::PointCloud<pcl::PointXYZ>);
    chull.setDimension(2);
    chull.setInputCloud(chull_input);
    {
      boost::mutex::scoped_lock lock(global_chull_mutex);
      chull.reconstruct(*chull_cloud);
    }
    
    return ConvexPolygon::Ptr(
      new ConvexPolygon(pointCloudToVertices(*chull_cloud)));
  }
  
  jsk_recognition_msgs::BoundingBox Cube::toROSMsg()
  {
    jsk_recognition_msgs::BoundingBox ret;
    ret.pose.position.x = pos_[0];
    ret.pose.position.y = pos_[1];
    ret.pose.position.z = pos_[2];
    ret.pose.orientation.x = rot_.x();
    ret.pose.orientation.y = rot_.y();
    ret.pose.orientation.z = rot_.z();
    ret.pose.orientation.w = rot_.w();
    ret.dimensions.x = dimensions_[0];
    ret.dimensions.y = dimensions_[1];
    ret.dimensions.z = dimensions_[2];
    return ret;
  }

  Vertices Cube::vertices()
  {
    Vertices vs;
    vs.push_back(buildVertex(0.5, 0.5, 0.5));
    vs.push_back(buildVertex(-0.5, 0.5, 0.5));
    vs.push_back(buildVertex(-0.5, -0.5, 0.5));
    vs.push_back(buildVertex(0.5, -0.5, 0.5));
    vs.push_back(buildVertex(0.5, 0.5, -0.5));
    vs.push_back(buildVertex(-0.5, 0.5, -0.5));
    vs.push_back(buildVertex(-0.5, -0.5, -0.5));
    vs.push_back(buildVertex(0.5, -0.5, -0.5));
    return vs;
  }
    
  
  Polygon::Ptr Cube::buildFace(const Eigen::Vector3f v0,
                               const Eigen::Vector3f v1,
                               const Eigen::Vector3f v2,
                               const Eigen::Vector3f v3)
  {
    Vertices vs;
    vs.push_back(v0);
    vs.push_back(v1);
    vs.push_back(v2);
    vs.push_back(v3);
    Polygon::Ptr(new Polygon(vs));
  }
  
  std::vector<Polygon::Ptr> Cube::faces()
  {
    std::vector<Polygon::Ptr> fs(6);
    Vertices vs = vertices();
    Eigen::Vector3f A = vs[0];
    Eigen::Vector3f B = vs[1];
    Eigen::Vector3f C = vs[2];
    Eigen::Vector3f D = vs[3];
    Eigen::Vector3f E = vs[4];
    Eigen::Vector3f F = vs[5];
    Eigen::Vector3f G = vs[6];
    Eigen::Vector3f H = vs[7];
    Vertices vs0, vs1, vs2, vs3, vs4, vs5, vs6;
    vs0.push_back(A); vs0.push_back(E); vs0.push_back(F); vs0.push_back(B);
    vs1.push_back(B); vs1.push_back(F); vs1.push_back(G); vs1.push_back(C);
    vs2.push_back(C); vs2.push_back(G); vs2.push_back(H); vs2.push_back(D);
    vs3.push_back(D); vs3.push_back(H); vs3.push_back(E); vs3.push_back(A);
    vs4.push_back(A); vs4.push_back(B); vs4.push_back(C); vs4.push_back(D);
    vs5.push_back(E); vs5.push_back(H); vs5.push_back(G); vs5.push_back(F);
    fs[0].reset(new Polygon(vs0));
    fs[1].reset(new Polygon(vs1));
    fs[2].reset(new Polygon(vs2));
    fs[3].reset(new Polygon(vs3));
    fs[4].reset(new Polygon(vs4));
    fs[5].reset(new Polygon(vs5));
    return fs;
  }

  Eigen::Vector3f Cube::buildVertex(double i, double j, double k)
  {
    Eigen::Vector3f local = (Eigen::Vector3f::UnitX() * i * dimensions_[0] +
                             Eigen::Vector3f::UnitY() * j * dimensions_[1] +
                             Eigen::Vector3f::UnitZ() * k * dimensions_[2]);
    return Eigen::Translation3f(pos_) * rot_ * local;
  }
  
  Eigen::Vector3f Cube::nearestPoint(const Eigen::Vector3f& p,
                                     double& distance)
  {
    std::vector<Polygon::Ptr> current_faces = faces();
    double min_distance = DBL_MAX;
    Eigen::Vector3f min_point;
    for (size_t i = 0; i < current_faces.size(); i++) {
      Polygon::Ptr f = current_faces[i];
      double d;
      Eigen::Vector3f q = f->nearestPoint(p, d);
      if (min_distance > d) {
        min_distance = d;
        min_point = q;
      }
    }
    distance = min_distance;
    return min_point;
  }
}
