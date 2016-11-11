// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef G2O_EIGEN_TYPES_H
#define G2O_EIGEN_TYPES_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace g2o {

  typedef Eigen::Matrix<int,2,1,Eigen::ColMajor>                                  Vector2I;
  typedef Eigen::Matrix<int,3,1,Eigen::ColMajor>                                  Vector3I;
  typedef Eigen::Matrix<int,4,1,Eigen::ColMajor>                                  Vector4I;
  typedef Eigen::Matrix<int,Eigen::Dynamic,1,Eigen::ColMajor>                     VectorXI; 

  typedef Eigen::Matrix<float,2,1,Eigen::ColMajor>                                Vector2F; 
  typedef Eigen::Matrix<float,3,1,Eigen::ColMajor>                                Vector3F; 
  typedef Eigen::Matrix<float,4,1,Eigen::ColMajor>                                Vector4F; 
  typedef Eigen::Matrix<float,Eigen::Dynamic,1,Eigen::ColMajor>                   VectorXF; 

  typedef Eigen::Matrix<double,2,1,Eigen::ColMajor>                               Vector2D;
  typedef Eigen::Matrix<double,3,1,Eigen::ColMajor>                               Vector3D;
  typedef Eigen::Matrix<double,4,1,Eigen::ColMajor>                               Vector4D;
  typedef Eigen::Matrix<double,Eigen::Dynamic,1,Eigen::ColMajor>                  VectorXD;

  typedef Eigen::Matrix<int,2,2,Eigen::ColMajor>                                  Matrix2I;
  typedef Eigen::Matrix<int,3,3,Eigen::ColMajor>                                  Matrix3I;
  typedef Eigen::Matrix<int,4,4,Eigen::ColMajor>                                  Matrix4I;
  typedef Eigen::Matrix<int,Eigen::Dynamic,Eigen::Dynamic,Eigen::ColMajor>        MatrixXI;

  typedef Eigen::Matrix<float,2,2,Eigen::ColMajor>                                Matrix2F;
  typedef Eigen::Matrix<float,3,3,Eigen::ColMajor>                                Matrix3F;
  typedef Eigen::Matrix<float,4,4,Eigen::ColMajor>                                Matrix4F;
  typedef Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic,Eigen::ColMajor>      MatrixXF;

  typedef Eigen::Matrix<double,2,2,Eigen::ColMajor>                               Matrix2D;
  typedef Eigen::Matrix<double,3,3,Eigen::ColMajor>                               Matrix3D;
  typedef Eigen::Matrix<double,4,4,Eigen::ColMajor>                               Matrix4D;
  typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::ColMajor>     MatrixXD;

  typedef Eigen::Transform<double,2,Eigen::Isometry,Eigen::ColMajor>              Isometry2D;
  typedef Eigen::Transform<double,3,Eigen::Isometry,Eigen::ColMajor>              Isometry3D;

  typedef Eigen::Transform<double,2,Eigen::Affine,Eigen::ColMajor>                Affine2D;
  typedef Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>                Affine3D;

} // end namespace g2o

#endif
