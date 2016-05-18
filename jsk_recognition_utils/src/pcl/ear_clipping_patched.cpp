// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
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

#include "jsk_recognition_utils/pcl/ear_clipping_patched.h"
#include <pcl/surface/ear_clipping.h>
#include <pcl/conversions.h>
#include <pcl/pcl_config.h>

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::EarClippingPatched::initCompute ()
{
  points_.reset (new pcl::PointCloud<pcl::PointXYZ>);

  if (!MeshProcessing::initCompute ())
    return (false);
  fromPCLPointCloud2 (input_mesh_->cloud, *points_);

  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::EarClippingPatched::performProcessing (PolygonMesh& output)
{
  output.polygons.clear ();
  output.cloud = input_mesh_->cloud;
  for (int i = 0; i < static_cast<int> (input_mesh_->polygons.size ()); ++i)
    triangulate (input_mesh_->polygons[i], output);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::EarClippingPatched::triangulate (const Vertices& vertices, PolygonMesh& output)
{
  const int n_vertices = static_cast<const int> (vertices.vertices.size ());

  if (n_vertices < 3)
    return;
  else if (n_vertices == 3)
  {
    output.polygons.push_back( vertices );
    return;
  }

  std::vector<uint32_t> remaining_vertices (n_vertices);
  if (area (vertices.vertices) > 0) // clockwise?
    remaining_vertices = vertices.vertices;
  else
    for (int v = 0; v < n_vertices; v++)
      remaining_vertices[v] = vertices.vertices[n_vertices - 1 - v];

  // Avoid closed loops.
  if (remaining_vertices.front () == remaining_vertices.back ())
    remaining_vertices.erase (remaining_vertices.end () - 1);

  // null_iterations avoids infinite loops if the polygon is not simple.
  for (int u = static_cast<int> (remaining_vertices.size ()) - 1, null_iterations = 0;
      remaining_vertices.size () > 2 && null_iterations < static_cast<int >(remaining_vertices.size () * 2);
      ++null_iterations, u = (u+1) % static_cast<int> (remaining_vertices.size ()))
  {
    int v = (u + 1) % static_cast<int> (remaining_vertices.size ());
    int w = (u + 2) % static_cast<int> (remaining_vertices.size ());

    if (isEar (u, v, w, remaining_vertices))
    {
      Vertices triangle;
      triangle.vertices.resize (3);
      triangle.vertices[0] = remaining_vertices[u];
      triangle.vertices[1] = remaining_vertices[v];
      triangle.vertices[2] = remaining_vertices[w];
      output.polygons.push_back (triangle);
      remaining_vertices.erase (remaining_vertices.begin () + v);
      null_iterations = 0;
    }
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////
float
pcl::EarClippingPatched::area (const std::vector<uint32_t>& vertices)
{
    //if the polygon is projected onto the xy-plane, the area of the polygon is determined
    //by the trapeze formula of Gauss. However this fails, if the projection is one 'line'.
    //Therefore the following implementation determines the area of the flat polygon in 3D-space
    //using Stoke's law: http://code.activestate.com/recipes/578276-3d-polygon-area/

    int n = static_cast<int> (vertices.size ());
    float area = 0.0f;
    Eigen::Vector3f prev_p, cur_p;
    Eigen::Vector3f total (0,0,0);
    Eigen::Vector3f unit_normal;

    if (n > 3)
    {
        for (int prev = n - 1, cur = 0; cur < n; prev = cur++)
        {
            prev_p = points_->points[vertices[prev]].getVector3fMap();
            cur_p = points_->points[vertices[cur]].getVector3fMap();

            total += prev_p.cross( cur_p );
        }

        //unit_normal is unit normal vector of plane defined by the first three points
        prev_p = points_->points[vertices[1]].getVector3fMap() - points_->points[vertices[0]].getVector3fMap();
        cur_p = points_->points[vertices[2]].getVector3fMap() - points_->points[vertices[0]].getVector3fMap();
        unit_normal = (prev_p.cross(cur_p)).normalized();

        area = total.dot( unit_normal );
    }

    return area * 0.5f; 
}


/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::EarClippingPatched::isEar (int u, int v, int w, const std::vector<uint32_t>& vertices)
{
  Eigen::Vector3f p_u, p_v, p_w;
  p_u = points_->points[vertices[u]].getVector3fMap();
  p_v = points_->points[vertices[v]].getVector3fMap();
  p_w = points_->points[vertices[w]].getVector3fMap();

  const float eps = 1e-15f;
  Eigen::Vector3f p_uv, p_uw;
  p_uv = p_v - p_u;
  p_uw = p_w - p_u;

  // Avoid flat triangles.
  if ((p_uv.cross(p_uw)).norm() < eps)
    return (false);

  Eigen::Vector3f p;
  // Check if any other vertex is inside the triangle.
  for (int k = 0; k < static_cast<int> (vertices.size ()); k++)
  {
    if ((k == u) || (k == v) || (k == w))
      continue;
    p = points_->points[vertices[k]].getVector3fMap();

    if (isInsideTriangle (p_u, p_v, p_w, p))
      return (false);
  }
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::EarClippingPatched::isInsideTriangle (const Eigen::Vector3f& u,
                                    const Eigen::Vector3f& v,
                                    const Eigen::Vector3f& w,
                                    const Eigen::Vector3f& p)
{
  // see http://www.blackpawn.com/texts/pointinpoly/default.html
  // Barycentric Coordinates
  Eigen::Vector3f v0 = w - u;
  Eigen::Vector3f v1 = v - u;
  Eigen::Vector3f v2 = p - u;

  // Compute dot products
  float dot00 = v0.dot(v0);
  float dot01 = v0.dot(v1);
  float dot02 = v0.dot(v2);
  float dot11 = v1.dot(v1);
  float dot12 = v1.dot(v2);

  // Compute barycentric coordinates
  float invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
  float a = (dot11 * dot02 - dot01 * dot12) * invDenom;
  float b = (dot00 * dot12 - dot01 * dot02) * invDenom;

  // Check if point is in triangle
  return (a >= 0) && (b >= 0) && (a + b < 1);
}

