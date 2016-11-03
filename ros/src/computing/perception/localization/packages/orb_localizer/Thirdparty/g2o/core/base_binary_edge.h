// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, H. Strasdat, W. Burgard
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

#ifndef G2O_BASE_BINARY_EDGE_H
#define G2O_BASE_BINARY_EDGE_H

#include <iostream>
#include <limits>

#include "g2o/core/base_edge.h"
#include "g2o/core/robust_kernel.h"
#include "../config.h"

namespace g2o {

  using namespace Eigen;

  template <int D, typename E, typename VertexXi, typename VertexXj>
  class BaseBinaryEdge : public BaseEdge<D, E>
  {
    public:

      typedef VertexXi VertexXiType;
      typedef VertexXj VertexXjType;

      static const int Di = VertexXiType::Dimension;
      static const int Dj = VertexXjType::Dimension;

      static const int Dimension = BaseEdge<D, E>::Dimension;
      typedef typename BaseEdge<D,E>::Measurement Measurement;
      typedef typename Matrix<double, D, Di>::AlignedMapType JacobianXiOplusType;
      typedef typename Matrix<double, D, Dj>::AlignedMapType JacobianXjOplusType;
      typedef typename BaseEdge<D,E>::ErrorVector ErrorVector;
      typedef typename BaseEdge<D,E>::InformationType InformationType;

      typedef Eigen::Map<Matrix<double, Di, Dj>, Matrix<double, Di, Dj>::Flags & AlignedBit ? Aligned : Unaligned > HessianBlockType;
      typedef Eigen::Map<Matrix<double, Dj, Di>, Matrix<double, Dj, Di>::Flags & AlignedBit ? Aligned : Unaligned > HessianBlockTransposedType;

      BaseBinaryEdge() : BaseEdge<D,E>(),
      _hessianRowMajor(false),
      _hessian(0, VertexXiType::Dimension, VertexXjType::Dimension), // HACK we map to the null pointer for initializing the Maps
      _hessianTransposed(0, VertexXjType::Dimension, VertexXiType::Dimension),
      _jacobianOplusXi(0, D, Di), _jacobianOplusXj(0, D, Dj)
      {
        _vertices.resize(2);
      }

      virtual OptimizableGraph::Vertex* createFrom();
      virtual OptimizableGraph::Vertex* createTo();

      virtual void resize(size_t size);

      virtual bool allVerticesFixed() const;

      virtual void linearizeOplus(JacobianWorkspace& jacobianWorkspace);

      /**
       * Linearizes the oplus operator in the vertex, and stores
       * the result in temporary variables _jacobianOplusXi and _jacobianOplusXj
       */
      virtual void linearizeOplus();

      //! returns the result of the linearization in the manifold space for the node xi
      const JacobianXiOplusType& jacobianOplusXi() const { return _jacobianOplusXi;}
      //! returns the result of the linearization in the manifold space for the node xj
      const JacobianXjOplusType& jacobianOplusXj() const { return _jacobianOplusXj;}

      virtual void constructQuadraticForm() ;

      virtual void mapHessianMemory(double* d, int i, int j, bool rowMajor);

      using BaseEdge<D,E>::resize;
      using BaseEdge<D,E>::computeError;

    protected:
      using BaseEdge<D,E>::_measurement;
      using BaseEdge<D,E>::_information;
      using BaseEdge<D,E>::_error;
      using BaseEdge<D,E>::_vertices;
      using BaseEdge<D,E>::_dimension;

      bool _hessianRowMajor;
      HessianBlockType _hessian;
      HessianBlockTransposedType _hessianTransposed;
      JacobianXiOplusType _jacobianOplusXi;
      JacobianXjOplusType _jacobianOplusXj;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

#include "g2o/core/base_binary_edge.hpp"

} // end namespace g2o

#endif
