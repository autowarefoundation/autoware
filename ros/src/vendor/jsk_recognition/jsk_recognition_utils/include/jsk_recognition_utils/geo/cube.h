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

#ifndef JSK_RECOGNITION_UTILS_GEO_CUBE_H_
#define JSK_RECOGNITION_UTILS_GEO_CUBE_H_

#include "jsk_recognition_utils/geo/polygon.h"
#include "jsk_recognition_utils/geo/convex_polygon.h"
#include <jsk_recognition_msgs/BoundingBoxArray.h>

namespace jsk_recognition_utils
{
  class Cube
  {
  public:
    typedef boost::shared_ptr<Cube> Ptr;
    Cube(const Eigen::Vector3f& pos, const Eigen::Quaternionf& rot);
    Cube(const Eigen::Vector3f& pos, const Eigen::Quaternionf& rot,
         const std::vector<double>& dimensions);
    Cube(const Eigen::Vector3f& pos, const Eigen::Quaternionf& rot,
         const Eigen::Vector3f& dimensions);
    Cube(const Eigen::Vector3f& pos, // centroid
         const Line& line_a, const Line& line_b, const Line& line_c);
    Cube(const jsk_recognition_msgs::BoundingBox& box);
    virtual ~Cube();
    std::vector<Segment::Ptr> edges();
    ConvexPolygon::Ptr intersectConvexPolygon(Plane& plane);
    std::vector<double> getDimensions() const { return dimensions_; };
    void setDimensions(const std::vector<double>& new_dimensions) {
      dimensions_[0] = new_dimensions[0];
      dimensions_[1] = new_dimensions[1];
      dimensions_[2] = new_dimensions[2];
    }
    jsk_recognition_msgs::BoundingBox toROSMsg();

    /**
     * @brief
     * returns vertices as an array of Eigen::Vectro3f.
     * The order of the vertices is:
     * [1, 1, 1], [-1, 1, 1], [-1, -1, 1], [1, -1, 1],
     * [1, 1, -1], [-1, 1, -1], [-1, -1, -1], [1, -1, -1].
     */
    Vertices vertices();

    /**
     * @brief
     * returns vertices transformed by pose_offset.
     */
    Vertices transformVertices(const Eigen::Affine3f& pose_offset);
    
    /**
     * @brief
     * returns all the 6 faces as Polygon::Ptr.
     * TODO: is it should be ConvexPolygon?
     */
    std::vector<Polygon::Ptr> faces();

    /**
     * @brief
     * compute minimum distance from point p to cube surface.
     *
     * Distance computation depends on Polygon::nearestPoint and
     * this methods just searches a face which resutnrs the smallest
     * distance.
     */
    virtual Eigen::Vector3f nearestPoint(const Eigen::Vector3f& p,
                                         double& distance);
  protected:
    Eigen::Vector3f pos_;
    Eigen::Quaternionf rot_;
    std::vector<double> dimensions_;

    /**
     * @brief
     * A helper method to build polygon from 4 vertices.
     */
    virtual Polygon::Ptr buildFace(const Eigen::Vector3f v0,
                                   const Eigen::Vector3f v1,
                                   const Eigen::Vector3f v2,
                                   const Eigen::Vector3f v3);

    /**
     * @brief
     * A helper method to build vertex from x-y-z relatiev coordinates.
     */
    virtual Eigen::Vector3f buildVertex(double i, double j, double k);
    
  private:
    
  };
}

#endif
