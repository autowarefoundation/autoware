// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
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

#include "g2o/types/types_seven_dof_expmap.h"

namespace g2o {

  VertexSim3Expmap::VertexSim3Expmap() : BaseVertex<7, Sim3>()
  {
    _marginalized=false;
    _fix_scale = false;
  }


  EdgeSim3::EdgeSim3() :
      BaseBinaryEdge<7, Sim3, VertexSim3Expmap, VertexSim3Expmap>()
  {
  }


  bool VertexSim3Expmap::read(std::istream& is)
  {
    Vector7d cam2world;
    for (int i=0; i<6; i++){
      is >> cam2world[i];
    }
    is >> cam2world[6];
//    if (! is) {
//      // if the scale is not specified we set it to 1;
//      std::cerr << "!s";
//      cam2world[6]=0.;
//    }

    for (int i=0; i<2; i++)
    {
      is >> _focal_length1[i];
    }
    for (int i=0; i<2; i++)
    {
      is >> _principle_point1[i];
    }

    setEstimate(Sim3(cam2world).inverse());
    return true;
  }

  bool VertexSim3Expmap::write(std::ostream& os) const
  {
    Sim3 cam2world(estimate().inverse());
    Vector7d lv=cam2world.log();
    for (int i=0; i<7; i++){
      os << lv[i] << " ";
    }
    for (int i=0; i<2; i++)
    {
      os << _focal_length1[i] << " ";
    }
    for (int i=0; i<2; i++)
    {
      os << _principle_point1[i] << " ";
    }
    return os.good();
  }

  bool EdgeSim3::read(std::istream& is)
  {
    Vector7d v7;
    for (int i=0; i<7; i++){
      is >> v7[i];
    }

    Sim3 cam2world(v7);
    setMeasurement(cam2world.inverse());

    for (int i=0; i<7; i++)
      for (int j=i; j<7; j++)
      {
        is >> information()(i,j);
        if (i!=j)
          information()(j,i)=information()(i,j);
      }
    return true;
  }

  bool EdgeSim3::write(std::ostream& os) const
  {
    Sim3 cam2world(measurement().inverse());
    Vector7d v7 = cam2world.log();
    for (int i=0; i<7; i++)
    {
      os  << v7[i] << " ";
    }
    for (int i=0; i<7; i++)
      for (int j=i; j<7; j++){
        os << " " <<  information()(i,j);
    }
    return os.good();
  }

  /**Sim3ProjectXYZ*/

  EdgeSim3ProjectXYZ::EdgeSim3ProjectXYZ() :
  BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSim3Expmap>()
  {
  }

  bool EdgeSim3ProjectXYZ::read(std::istream& is)
  {
    for (int i=0; i<2; i++)
    {
      is >> _measurement[i];
    }

    for (int i=0; i<2; i++)
      for (int j=i; j<2; j++) {
  is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
    return true;
  }

  bool EdgeSim3ProjectXYZ::write(std::ostream& os) const
  {
    for (int i=0; i<2; i++){
      os  << _measurement[i] << " ";
    }

    for (int i=0; i<2; i++)
      for (int j=i; j<2; j++){
  os << " " <<  information()(i,j);
    }
    return os.good();
  }

/**InverseSim3ProjectXYZ*/

  EdgeInverseSim3ProjectXYZ::EdgeInverseSim3ProjectXYZ() :
  BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSim3Expmap>()
  {
  }

  bool EdgeInverseSim3ProjectXYZ::read(std::istream& is)
  {
    for (int i=0; i<2; i++)
    {
      is >> _measurement[i];
    }

    for (int i=0; i<2; i++)
      for (int j=i; j<2; j++) {
  is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
    return true;
  }

  bool EdgeInverseSim3ProjectXYZ::write(std::ostream& os) const
  {
    for (int i=0; i<2; i++){
      os  << _measurement[i] << " ";
    }

    for (int i=0; i<2; i++)
      for (int j=i; j<2; j++){
  os << " " <<  information()(i,j);
    }
    return os.good();
  }


//  void EdgeSim3ProjectXYZ::linearizeOplus()
//  {
//    VertexSim3Expmap * vj = static_cast<VertexSim3Expmap *>(_vertices[1]);
//    Sim3 T = vj->estimate();

//    VertexPointXYZ* vi = static_cast<VertexPointXYZ*>(_vertices[0]);
//    Vector3d xyz = vi->estimate();
//    Vector3d xyz_trans = T.map(xyz);

//    double x = xyz_trans[0];
//    double y = xyz_trans[1];
//    double z = xyz_trans[2];
//    double z_2 = z*z;

//    Matrix<double,2,3> tmp;
//    tmp(0,0) = _focal_length(0);
//    tmp(0,1) = 0;
//    tmp(0,2) = -x/z*_focal_length(0);

//    tmp(1,0) = 0;
//    tmp(1,1) = _focal_length(1);
//    tmp(1,2) = -y/z*_focal_length(1);

//    _jacobianOplusXi =  -1./z * tmp * T.rotation().toRotationMatrix();

//    _jacobianOplusXj(0,0) =  x*y/z_2 * _focal_length(0);
//    _jacobianOplusXj(0,1) = -(1+(x*x/z_2)) *_focal_length(0);
//    _jacobianOplusXj(0,2) = y/z *_focal_length(0);
//    _jacobianOplusXj(0,3) = -1./z *_focal_length(0);
//    _jacobianOplusXj(0,4) = 0;
//    _jacobianOplusXj(0,5) = x/z_2 *_focal_length(0);
//    _jacobianOplusXj(0,6) = 0; // scale is ignored


//    _jacobianOplusXj(1,0) = (1+y*y/z_2) *_focal_length(1);
//    _jacobianOplusXj(1,1) = -x*y/z_2 *_focal_length(1);
//    _jacobianOplusXj(1,2) = -x/z *_focal_length(1);
//    _jacobianOplusXj(1,3) = 0;
//    _jacobianOplusXj(1,4) = -1./z *_focal_length(1);
//    _jacobianOplusXj(1,5) = y/z_2 *_focal_length(1);
//    _jacobianOplusXj(1,6) = 0; // scale is ignored
//  }

} // end namespace
