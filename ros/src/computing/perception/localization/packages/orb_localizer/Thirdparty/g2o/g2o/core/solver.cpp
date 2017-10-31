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

#include "g2o/core/solver.h"

#include <cstring>
#include <algorithm>

namespace g2o {

Solver::Solver() :
  _optimizer(0), _x(0), _b(0), _xSize(0), _maxXSize(0),
  _isLevenberg(false), _additionalVectorSpace(0)
{
}

Solver::~Solver()
{
  delete[] _x;
  delete[] _b;
}

void Solver::resizeVector(size_t sx)
{
  size_t oldSize = _xSize;
  _xSize = sx;
  sx += _additionalVectorSpace; // allocate some additional space if requested
  if (_maxXSize < sx) {
    _maxXSize = 2*sx;
    delete[] _x;
    _x = new double[_maxXSize];
#ifndef NDEBUG
    memset(_x, 0, _maxXSize * sizeof(double));
#endif
    if (_b) { // backup the former b, might still be needed for online processing
      memcpy(_x, _b, oldSize * sizeof(double));
      delete[] _b;
      _b = new double[_maxXSize];
      std::swap(_b, _x);
    } else {
      _b = new double[_maxXSize];
#ifndef NDEBUG
      memset(_b, 0, _maxXSize * sizeof(double));
#endif
    }
  }
}

void Solver::setOptimizer(SparseOptimizer* optimizer)
{
  _optimizer = optimizer;
}

void Solver::setLevenberg(bool levenberg)
{
  _isLevenberg = levenberg;
}

void Solver::setAdditionalVectorSpace(size_t s)
{
  _additionalVectorSpace = s;
}

} // end namespace
