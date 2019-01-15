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

#include "jsk_recognition_utils/geo/polygon.h"
#include "jsk_recognition_utils/geo_util.h"
#include <jsk_topic_tools/log_utils.h>
#include "jsk_recognition_utils/pcl/ear_clipping_patched.h"
#include "jsk_recognition_utils/sensor_model_utils.h"

namespace jsk_recognition_utils
{
  Polygon Polygon::createPolygonWithSkip(const Vertices& vertices)
  {
    const double thr = 0.01;
    Polygon not_skipped_polygon(vertices);
    Vertices skipped_vertices;
    for (size_t i = 0; i < vertices.size(); i++) {
      size_t next_i = not_skipped_polygon.nextIndex(i);
      Eigen::Vector3f v0 = vertices[i];
      Eigen::Vector3f v1 = vertices[next_i];
      if ((v1 - v0).norm() > thr) {
        skipped_vertices.push_back(vertices[i]);
      }
    }
    return Polygon(skipped_vertices);
  }

  Eigen::Vector3f Polygon::centroid()
  {
    Eigen::Vector3f c(0, 0, 0);
    if (vertices_.size() == 0) {
      return c;
    }
    else {
      for (size_t i = 0; i < vertices_.size(); i++) {
        c = c + vertices_[i];
      }
      return c / vertices_.size();
    }
  }

  std::vector<Plane::Ptr> convertToPlanes(
    std::vector<pcl::ModelCoefficients::Ptr> coefficients)
  {
    std::vector<Plane::Ptr> ret;
    for (size_t i = 0; i < coefficients.size(); i++) {
      ret.push_back(Plane::Ptr (new Plane(coefficients[i]->values)));
    }
    return ret;
  }
  
  
  Polygon::Polygon(const Vertices& vertices):
    Plane((vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]).normalized(), vertices[0]),
    vertices_(vertices)
  {
    
  }

  Polygon::Polygon(const Vertices& vertices,
                   const std::vector<float>& coefficients):
    Plane(coefficients), vertices_(vertices)
  {
    
  }
  
  Polygon::~Polygon()
  {

  }

  size_t Polygon::getFarestPointIndex(const Eigen::Vector3f& O)
  {
    double max_distance = - DBL_MAX;
    size_t max_index = 0;
    for (size_t i = 0; i < vertices_.size(); i++) {
      Eigen::Vector3f v = vertices_[i];
      double d = (O - v).norm();
      if (max_distance < d) {
        max_distance = d;
        max_index = i;
      }
    }
    return max_index;
  }

  PointIndexPair Polygon::getNeighborIndex(size_t index)
  {
    return boost::make_tuple<size_t, size_t>(
      previousIndex(index), nextIndex(index));
  }

  double Polygon::area()
  {
    if (isTriangle()) {
      return (vertices_[1] - vertices_[0]).cross(vertices_[2] - vertices_[0]).norm() / 2.0;
    }
    else {
      std::vector<Polygon::Ptr> triangles = decomposeToTriangles();
      double sum = 0;
      for (size_t i = 0; i < triangles.size(); i++) {
        sum += triangles[i]->area();
      }
      return sum;
    }
  }
  
  Eigen::Vector3f Polygon::directionAtPoint(size_t i)
  {
    Eigen::Vector3f O = vertices_[i];
    Eigen::Vector3f A = vertices_[previousIndex(i)];
    Eigen::Vector3f B = vertices_[nextIndex(i)];
    Eigen::Vector3f OA = A - O;
    Eigen::Vector3f OB = B - O;
    Eigen::Vector3f n = (OA.normalized()).cross(OB.normalized());
    if (n.norm() == 0) {
      // ROS_ERROR("normal is 0");
      // ROS_ERROR("O: [%f, %f, %f]", O[0], O[1], O[2]);
      // ROS_ERROR("A: [%f, %f, %f]", A[0], A[1], A[2]);
      // ROS_ERROR("B: [%f, %f, %f]", B[0], B[1], B[2]);
      // ROS_ERROR("OA: [%f, %f, %f]", OA[0], OA[1], OA[2]);
      // ROS_ERROR("OB: [%f, %f, %f]", OB[0], OB[1], OB[2]);
      //exit(1);
    }
    return n.normalized();
  }
  
  bool Polygon::isTriangle() {
    return vertices_.size() == 3;
  }

  void Polygon::getLocalMinMax(double& min_x, double& min_y,
                               double& max_x, double& max_y)
  {
    min_x = DBL_MAX;
    min_y = DBL_MAX;
    max_x = - DBL_MAX;
    max_y = - DBL_MAX;
    
    Eigen::Affine3f inv_coords = coordinates().inverse();
    for (size_t i = 0; i < vertices_.size(); i++) {
      // Convert vertices into local coordinates
      Eigen::Vector3f local_point = inv_coords * vertices_[i];
      min_x = ::fmin(local_point[0], min_x);
      min_y = ::fmin(local_point[1], min_y);
      max_x = ::fmax(local_point[0], max_x);
      max_y = ::fmax(local_point[1], max_y);
    }
  }
  
  Eigen::Vector3f Polygon::randomSampleLocalPoint(boost::mt19937& random_generator)
  {
    // Compute min/max point
    double min_x, min_y, max_x, max_y;
    getLocalMinMax(min_x, min_y, max_x, max_y);
    std::vector<Polygon::Ptr> triangles = decomposeToTriangles();
    while (true) {
      double x = randomUniform(min_x, max_x, random_generator);
      double y = randomUniform(min_y, max_y, random_generator);
      Eigen::Vector3f local_v = Eigen::Vector3f(x, y, 0);
      Eigen::Vector3f v = coordinates() * local_v;
      // ROS_INFO_THROTTLE(1.0, "x: %f -- %f", min_x, max_x);
      // ROS_INFO_THROTTLE(1.0, "y: %f -- %f", min_y, max_y);
      // ROS_INFO_THROTTLE(1.0, "sampled point: [%f, %f]", x, y);
      // for (size_t i = 0; i < vertices_.size(); i++) {
      //   Eigen::Vector3f v = coordinates().inverse() * vertices_[i];
      //   ROS_INFO("v: [%f, %f, %f]", v[0], v[1], v[2]);
      // }
      if (isInside(v)) {
        return local_v;
      }
      else {
        // ROS_INFO_THROTTLE(1.0, "outside");
      }
    }
  }

  std::vector<Segment::Ptr> Polygon::edges() const
  {
    std::vector<Segment::Ptr> ret;
    ret.reserve(vertices_.size());
    for (size_t i = 0; i < vertices_.size() - 1; i++) {
      // edge between i and i+1
      ret.push_back(Segment::Ptr(new Segment(vertices_[i], vertices_[i+1])));
    }
    // edge between [-1] and [0]
    ret.push_back(Segment::Ptr(new Segment(vertices_[vertices_.size() - 1], vertices_[0])));
    return ret;
  }
  
  double Polygon::distance(const Eigen::Vector3f& point)
  {
    Eigen::Vector3f nearest_point;
    return Polygon::distance(point, nearest_point);
  }

  double Polygon::distance(const Eigen::Vector3f& point,
                           Eigen::Vector3f& nearest_point)
  {
    double distance;
    nearest_point = Polygon::nearestPoint(point, distance);
    return distance;
  }

  Eigen::Vector3f Polygon::nearestPoint(const Eigen::Vector3f& p,
                                        double& distance)
  {
    Eigen::Vector3f projected_p;
    Plane::project(p, projected_p);
    if (isInside(projected_p)) {
      distance = (p - projected_p).norm();
      return projected_p;
    }
    else {
      std::vector<Segment::Ptr> boundary_edges = edges();
      double min_distnace = DBL_MAX;
      Eigen::Vector3f nearest_point;
      // brute-force searching
      for (size_t i = 0; i < boundary_edges.size(); i++) {
        Segment::Ptr edge = boundary_edges[i];
        Eigen::Vector3f foot;
        double d = edge->distance(p, foot);
        if (min_distnace > d) {
          nearest_point = foot;
          min_distnace = d;
        }
      }
      distance = min_distnace;
      return nearest_point;
    }
  }
  
  size_t Polygon::getNumVertices() {
    return vertices_.size();
  }
  
  Eigen::Vector3f Polygon::getVertex(size_t i) {
    return vertices_[i];
  }
  
  Polygon::PtrPair Polygon::separatePolygon(size_t index)
  {
    PointIndexPair neighbor_index = getNeighborIndex(index);
    Vertices triangle_vertices;
    triangle_vertices.push_back(vertices_[index]);
    triangle_vertices.push_back(vertices_[neighbor_index.get<1>()]);
    triangle_vertices.push_back(vertices_[neighbor_index.get<0>()]);
    Polygon::Ptr triangle(new Polygon(triangle_vertices));
    Vertices rest_vertices;
    // do not add the points on the line
    for (size_t i = neighbor_index.get<1>(); i != index;) {
      // check the points on the line
      if (i == neighbor_index.get<1>()) {
        rest_vertices.push_back(vertices_[i]);
      }
      else {
        if (directionAtPoint(i).norm() != 0.0) {
          rest_vertices.push_back(vertices_[i]);
        }
        else {
          ROS_ERROR("removed: %lu", i);
        }
      }
      i = nextIndex(i);
    }
    Polygon::Ptr rest(new Polygon(rest_vertices));
    return boost::make_tuple<Polygon::Ptr, Polygon::Ptr>(
      triangle, rest);
  }
  
  bool Polygon::isPossibleToRemoveTriangleAtIndex(
    size_t index,
    const Eigen::Vector3f& direction)
  {
    Polygon::PtrPair candidate = separatePolygon(index);
    Polygon::Ptr triangle_candidate = candidate.get<0>();
    Polygon::Ptr rest_candidate = candidate.get<1>();
    // first check direction
    Eigen::Vector3f the_direction = directionAtPoint(index);
    //ROS_INFO("direction: [%f, %f, %f]", the_direction[0], the_direction[1], the_direction[2]);
    if (the_direction.norm() == 0.0) {
      ROS_ERROR("malformed polygon");
      exit(1);
    }
    if (direction.dot(the_direction) < 0) {
#ifdef DEBUG_GEO_UTIL
      ROS_INFO("triangle is not same direction");
      ROS_INFO("direction: [%f, %f, %f]", direction[0], direction[1], direction[2]);
      ROS_INFO("the_direction: [%f, %f, %f]",
               the_direction[0],
               the_direction[1],
               the_direction[2]);
      for (size_t i = 0; i < vertices_.size(); i++) {
        Eigen::Vector3f v = directionAtPoint(i);
        ROS_INFO("the_direction[%lu]: [%f, %f, %f]",
                 i, v[0], v[1], v[2]);
      // other direction
      }
#endif
      return false;
    }
    else {
      //return true;
      // second, check the triangle includes the rest of points or not
      for (size_t i = 0; i < rest_candidate->vertices_.size(); i++) {
        if (i == 0 || i == rest_candidate->vertices_.size() - 1) {
          continue;       // do not check the first and the last point
        }
        else {
          Eigen::Vector3f P = rest_candidate->getVertex(i);
          Eigen::Vector3f A = triangle_candidate->getVertex(0);
          Eigen::Vector3f B = triangle_candidate->getVertex(1);
          Eigen::Vector3f C = triangle_candidate->getVertex(2);
          Eigen::Vector3f CA = A - C;
          Eigen::Vector3f BC = C - B;
          Eigen::Vector3f AB = B - A;
          Eigen::Vector3f AP = P - A;
          Eigen::Vector3f BP = P - B;
          Eigen::Vector3f CP = P - C;
          Eigen::Vector3f Across = CA.normalized().cross(AP.normalized()).normalized();
          Eigen::Vector3f Bcross = AB.normalized().cross(BP.normalized()).normalized();
          Eigen::Vector3f Ccross = BC.normalized().cross(CP.normalized()).normalized();
#ifdef DEBUG_GEO_UTIL
          ROS_INFO("P: [%f, %f, %f]", P[0], P[1], P[2]);
          ROS_INFO("A: [%f, %f, %f]", A[0], A[1], A[2]);
          ROS_INFO("B: [%f, %f, %f]", B[0], B[1], B[2]);
          ROS_INFO("C: [%f, %f, %f]", C[0], C[1], C[2]);
          ROS_INFO("Across: [%f, %f, %f]", Across[0], Across[1], Across[2]);
          ROS_INFO("Bcross: [%f, %f, %f]", Bcross[0], Bcross[1], Bcross[2]);
          ROS_INFO("Ccross: [%f, %f, %f]", Ccross[0], Ccross[1], Ccross[2]);
          ROS_INFO("Across-Bcross: %f", Across.dot(Bcross));
          ROS_INFO("Bcross-Ccross: %f", Bcross.dot(Ccross));
          ROS_INFO("Ccross-Across: %f", Ccross.dot(Across));
#endif
          if (((Across.dot(Bcross) > 0 &&
                Bcross.dot(Ccross) > 0 &&
                Ccross.dot(Across) > 0) ||
               (Across.dot(Bcross) < 0 &&
                Bcross.dot(Ccross) < 0 &&
                Ccross.dot(Across) < 0))) {
            // ROS_ERROR("%lu -- %lu is inside", index, i);
            return false;
          }
          // ConvexPolygon convex_triangle(triangle_candidate->vertices_);
          // if (convex_triangle.isInside(v)) {
          //   //ROS_INFO("vertices is inside of the polygon");
          //   return false;
          // }
        }
      }
      return true;
    }
  }

  void Polygon::transformBy(const Eigen::Affine3d& transform)
  {
    Eigen::Affine3f float_affine;
    convertEigenAffine3(transform, float_affine);
    transformBy(float_affine);
  }

  void Polygon::transformBy(const Eigen::Affine3f& transform)
  {
    // 1. clear cached_triangles_
    // 2. transform vertices_
    // 3. update normal_ and d_
    // 3. update plane_coordinates_
    cached_triangles_.clear();
    for (size_t i = 0; i < vertices_.size(); i++) {
      vertices_[i] = transform * vertices_[i];
    }
    // compute normal and d
    normal_ = (vertices_[1] - vertices_[0]).cross(vertices_[2] - vertices_[0]).normalized();
    d_ = - normal_.dot(vertices_[0]) / normal_.norm();
    initializeCoordinates();
  }
  
  bool Polygon::maskImage(const jsk_recognition_utils::CameraDepthSensor& model,
                          cv::Mat& image) const
  {
    std::vector<cv::Point> projected_vertices
      = project3DPointstoPixel(model.getPinholeCameraModel(), vertices_);
    bool all_outside = true;
    // check some of vertices is inside of FoV
    for (size_t i = 0; i < projected_vertices.size(); i++) {
      if (model.isInside(projected_vertices[i])) {
        all_outside = false;
      }
    }
    image = model.image(CV_8UC1);
    // check all the v is positive
    for (size_t i = 0; i < vertices_.size(); i++) {
      if (vertices_[i][2] < 0) {
        return false;
      }
    }
    const cv::Point* element_points[1] = {&projected_vertices[0]};
    int number_of_points = (int)projected_vertices.size();
    // Is it should be cv::fillPoly?
    cv::fillPoly(image, 
                 element_points,
                 &number_of_points,
                 1,
                 cv::Scalar(255));
    return !all_outside;
  }

  void Polygon::drawLineToImage(const jsk_recognition_utils::CameraDepthSensor& model,
                                cv::Mat& image,
                                const cv::Scalar& color,
                                const int line_width) const
  {
    std::vector<cv::Point> projected_vertices
      = project3DPointstoPixel(model.getPinholeCameraModel(), vertices_);
    
    for (size_t i = 0; i < projected_vertices.size() - 1; i++) {
      cv::Point from = projected_vertices[i];
      cv::Point to = projected_vertices[i+1];
      if (model.isInside(from) || model.isInside(to)) {
        cv::line(image, from, to, color, line_width);
      }
    }
    cv::Point from = projected_vertices[projected_vertices.size() - 1];
    cv::Point to = projected_vertices[0];
    if (model.isInside(from) || model.isInside(to)) {
      cv::line(image, from, to, color, line_width);
    }
  }

  bool Polygon::isConvex()
  {
#ifdef DEBUG_GEO_UTIL
    for (size_t i = 0; i < getNumVertices(); i++) {
      Eigen::Vector3f n = directionAtPoint(i);
      ROS_INFO("n[%lu] [%f, %f, %f]", i, n[0], n[1], n[2]);
    }
#endif
    Eigen::Vector3f n0 = directionAtPoint(0);
    for (size_t i = 1; i < getNumVertices(); i++) {
      Eigen::Vector3f n = directionAtPoint(i);
      if (n0.dot(n) < 0) {
        return false;
      }
    }
    return true;
  }
  
  std::vector<Polygon::Ptr> Polygon::decomposeToTriangles()
  {
    if (cached_triangles_.size() != 0) {
      return cached_triangles_;
    }
    std::vector<Polygon::Ptr> ret;

    // if this polygon is triangle, return immediately
    if (isTriangle()) {
      ret.push_back(Polygon::Ptr( new Polygon(*this)));
      return ret;
    }

    pcl::EarClippingPatched clip;
    // convert
    pcl::PolygonMesh::Ptr input_mesh (new pcl::PolygonMesh);
    pcl::PCLPointCloud2 mesh_cloud;
    pcl::PointCloud<pcl::PointXYZ> mesh_pcl_cloud;
    boundariesToPointCloud<pcl::PointXYZ>(mesh_pcl_cloud);
    std::vector<pcl::Vertices> mesh_vertices(1);
    for (size_t i = 0; i < vertices_.size(); i++) {
      mesh_vertices[0].vertices.push_back(i);
    }
    //mesh_vertices[0].vertices.push_back(0); // close
    mesh_pcl_cloud.height = 1;
    mesh_pcl_cloud.width = mesh_pcl_cloud.points.size();
    pcl::toPCLPointCloud2<pcl::PointXYZ>(mesh_pcl_cloud, mesh_cloud);

    input_mesh->polygons = mesh_vertices;
    input_mesh->cloud = mesh_cloud;
    clip.setInputMesh(input_mesh);
    pcl::PolygonMesh output;
    clip.process(output);
    assert(output.polygons.size() != 0);
    // convert to Polygon instances
    for (size_t i = 0; i < output.polygons.size(); i++) {
      pcl::Vertices output_polygon_vertices = output.polygons[i];
      Vertices vs(output_polygon_vertices.vertices.size());
      for (size_t j = 0; j < output_polygon_vertices.vertices.size(); j++) {
        pcl::PointXYZ p
          = mesh_pcl_cloud.points[output_polygon_vertices.vertices[j]];
        Eigen::Vector3f v;
        pointFromXYZToVector<pcl::PointXYZ, Eigen::Vector3f>(p, v);
        vs[j] = v;
      }
      ret.push_back(Polygon::Ptr(new Polygon(vs, toCoefficients())));
    }
    cached_triangles_ = ret;
    return ret;
  }

  Eigen::Vector3f Polygon::getNormalFromVertices()
  {
    if (vertices_.size() >= 3) {
      return (vertices_[1] - vertices_[0]).cross(vertices_[2] - vertices_[0]).normalized();
    }
    else {
      ROS_ERROR("the number of vertices is not enough");
      return Eigen::Vector3f(0, 0, 0);
    }
  }

  size_t Polygon::previousIndex(size_t i)
  {
    if (i == 0) {
      return vertices_.size() - 1;
    }
    else {
      return i - 1;
    }
  }
  
  size_t Polygon::nextIndex(size_t i)
  {
    if (i == vertices_.size() - 1) {
      return 0;
    }
    else {
      return i + 1;
    }
  }

  Polygon Polygon::fromROSMsg(const geometry_msgs::Polygon& polygon)
  {
    Vertices vertices;
    for (size_t i = 0; i < polygon.points.size(); i++) {
      Eigen::Vector3f v;
      pointFromXYZToVector<geometry_msgs::Point32, Eigen::Vector3f>(
        polygon.points[i], v);
      vertices.push_back(v);
    }
    return Polygon(vertices);
  }

  Polygon::Ptr Polygon::fromROSMsgPtr(const geometry_msgs::Polygon& polygon)
  {
    Vertices vertices;
    for (size_t i = 0; i < polygon.points.size(); i++) {
      Eigen::Vector3f v;
      pointFromXYZToVector<geometry_msgs::Point32, Eigen::Vector3f>(
        polygon.points[i], v);
      vertices.push_back(v);
    }
    return Polygon::Ptr(new Polygon(vertices));
  }

  std::vector<Polygon::Ptr> Polygon::fromROSMsg(const jsk_recognition_msgs::PolygonArray& msg,
                                                const Eigen::Affine3f& offset)
  {
    std::vector<Polygon::Ptr> ret;
    for (size_t i = 0; i < msg.polygons.size(); i++) {
      Polygon::Ptr polygon = Polygon::fromROSMsgPtr(msg.polygons[i].polygon);
      polygon->transformBy(offset);
      ret.push_back(polygon);
    }
    return ret;
  }
  
  bool Polygon::isInside(const Eigen::Vector3f& p)
  {
    if (isTriangle()) {
      Eigen::Vector3f A = vertices_[0];
      Eigen::Vector3f B = vertices_[1];
      Eigen::Vector3f C = vertices_[2];
      // Eigen::Vector3f cross0 = (A - C).cross(p - A);
      // Eigen::Vector3f cross1 = (B - A).cross(p - B);
      // Eigen::Vector3f cross2 = (C - B).cross(p - C);
      
      Eigen::Vector3f cross0 = (B - A).cross(p - A);
      Eigen::Vector3f cross1 = (C - B).cross(p - B);
      Eigen::Vector3f cross2 = (A - C).cross(p - C);
      if (cross0.dot(cross1) >= 0 &&
          cross1.dot(cross2) >= 0) {
        return true;
      }
      else {
        return false;
      }
    }
    else {
      std::vector<Polygon::Ptr> triangles = decomposeToTriangles();
      for (size_t i = 0; i < triangles.size(); i++) {
        if (triangles[i]->isInside(p)) {
          return true;
        }
      }
      return false;
    }
  }


}
