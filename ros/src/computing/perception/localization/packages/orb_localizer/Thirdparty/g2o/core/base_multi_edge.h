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

#ifndef G2O_BASE_MULTI_EDGE_H
#define G2O_BASE_MULTI_EDGE_H

#include <iostream>
#include <iomanip>
#include <limits>

#include <Eigen/StdVector>

#include "g2o/core/base_edge.h"
#include "g2o/core/robust_kernel.h"
#include "../config.h"

namespace g2o {

  using namespace Eigen;

  /**
   * \brief base class to represent an edge connecting an arbitrary number of nodes
   *
   * D - Dimension of the measurement
   * E - type to represent the measurement
   */
  template <int D, typename E>
  class BaseMultiEdge : public BaseEdge<D,E>
  {
    public:
      /**
       * \brief helper for mapping the Hessian memory of the upper triangular block
       */
      struct HessianHelper {
        Eigen::Map<MatrixXd> matrix;     ///< the mapped memory
        bool transposed;          ///< the block has to be transposed
        HessianHelper() : matrix(0, 0, 0), transposed(false) {}
      };

    public:
      static const int Dimension = BaseEdge<D,E>::Dimension;
      typedef typename BaseEdge<D,E>::Measurement Measurement;
      typedef MatrixXd::MapType JacobianType;
      typedef typename BaseEdge<D,E>::ErrorVector ErrorVector;
      typedef typename BaseEdge<D,E>::InformationType InformationType;
      typedef Eigen::Map<MatrixXd, MatrixXd::Flags & AlignedBit ? Aligned : Unaligned > HessianBlockType;

      BaseMultiEdge() : BaseEdge<D,E>()
      {
      }
      
      virtual void linearizeOplus(JacobianWorkspace& jacobianWorkspace);

      /**
       * Linearizes the oplus operator in the vertex, and stores
       * the result in temporary variable vector _jacobianOplus
       */
      virtual void linearizeOplus();
      
      virtual void resize(size_t size);

      virtual bool allVerticesFixed() const;

      virtual void constructQuadraticForm() ;

      virtual void mapHessianMemory(double* d, int i, int j, bool rowMajor);

      using BaseEdge<D,E>::computeError;

    protected:
      using BaseEdge<D,E>::_measurement;
      using BaseEdge<D,E>::_information;
      using BaseEdge<D,E>::_error;
      using BaseEdge<D,E>::_vertices;
      using BaseEdge<D,E>::_dimension;

      std::vector<HessianHelper> _hessian;
      std::vector<JacobianType, aligned_allocator<JacobianType> > _jacobianOplus; ///< jacobians of the edge (w.r.t. oplus)

      void computeQuadraticForm(const InformationType& omega, const ErrorVector& weightedError);

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

#include "g2o/core/base_multi_edge.hpp"

} // end namespace g2o

#endif
