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

#ifndef G2O_BASE_VERTEX_H
#define G2O_BASE_VERTEX_H

#include "../stuff/macros.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <Eigen/StdVector>
#include <stack>
#include "g2o/core/creators.h"
#include "g2o/core/optimizable_graph.h"

namespace g2o {

  using namespace Eigen;


/**
 * \brief Templatized BaseVertex
 *
 * Templatized BaseVertex
 * D  : minimal dimension of the vertex, e.g., 3 for rotation in 3D
 * T  : internal type to represent the estimate, e.g., Quaternion for rotation in 3D
 */
  template <int D, typename T>
  class BaseVertex : public OptimizableGraph::Vertex {
    public:
    typedef T EstimateType;
    typedef std::stack<EstimateType, 
                       std::vector<EstimateType,  Eigen::aligned_allocator<EstimateType> > >
    BackupStackType;

    static const int Dimension = D;           ///< dimension of the estimate (minimal) in the manifold space

    typedef Eigen::Map<Matrix<double, D, D>, Matrix<double,D,D>::Flags & AlignedBit ? Aligned : Unaligned >  HessianBlockType;

  public:
    BaseVertex();

    virtual const double& hessian(int i, int j) const { assert(i<D && j<D); return _hessian(i,j);}
    virtual double& hessian(int i, int j)  { assert(i<D && j<D); return _hessian(i,j);}
    virtual double hessianDeterminant() const {return _hessian.determinant();}
    virtual double* hessianData() { return const_cast<double*>(_hessian.data());}

    virtual void mapHessianMemory(double* d);

    virtual int copyB(double* b_) const {
      memcpy(b_, _b.data(), Dimension * sizeof(double));
      return Dimension; 
    }

    virtual const double& b(int i) const { assert(i < D); return _b(i);}
    virtual double& b(int i) { assert(i < D); return _b(i);}
    virtual double* bData() { return _b.data();}

    virtual void clearQuadraticForm();

    //! updates the current vertex with the direct solution x += H_ii\b_ii
    //! @returns the determinant of the inverted hessian
    virtual double solveDirect(double lambda=0);

    //! return right hand side b of the constructed linear system
    Matrix<double, D, 1>& b() { return _b;}
    const Matrix<double, D, 1>& b() const { return _b;}
    //! return the hessian block associated with the vertex
    HessianBlockType& A() { return _hessian;}
    const HessianBlockType& A() const { return _hessian;}

    virtual void push() { _backup.push(_estimate);}
    virtual void pop() { assert(!_backup.empty()); _estimate = _backup.top(); _backup.pop(); updateCache();}
    virtual void discardTop() { assert(!_backup.empty()); _backup.pop();}
    virtual int stackSize() const {return _backup.size();}

    //! return the current estimate of the vertex
    const EstimateType& estimate() const { return _estimate;}
    //! set the estimate for the vertex also calls updateCache()
    void setEstimate(const EstimateType& et) { _estimate = et; updateCache();}

  protected:
    HessianBlockType _hessian;
    Matrix<double, D, 1> _b;
    EstimateType _estimate;
    BackupStackType _backup;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#include "g2o/core/base_vertex.hpp"

} // end namespace g2o


#endif
