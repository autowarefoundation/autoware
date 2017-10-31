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

template <int D, typename T>
BaseVertex<D, T>::BaseVertex() :
  OptimizableGraph::Vertex(),
  _hessian(0, D, D)
{
  _dimension = D;
}

template <int D, typename T>
double BaseVertex<D, T>::solveDirect(double lambda) {
  Matrix <double, D, D> tempA=_hessian + Matrix <double, D, D>::Identity()*lambda;
  double det=tempA.determinant();
  if (g2o_isnan(det) || det < std::numeric_limits<double>::epsilon())
    return det;
  Matrix <double, D, 1> dx=tempA.llt().solve(_b);
  oplus(&dx[0]);
  return det;
}

template <int D, typename T>
void BaseVertex<D, T>::clearQuadraticForm() {
  _b.setZero();
}

template <int D, typename T>
void BaseVertex<D, T>::mapHessianMemory(double* d)
{
  new (&_hessian) HessianBlockType(d, D, D);
}
