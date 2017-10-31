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

#include "g2o/core/marginal_covariance_cholesky.h"

#include <algorithm>
#include <cassert>
using namespace std;

namespace g2o {

struct MatrixElem
{
  int r, c;
  MatrixElem(int r_, int c_) : r(r_), c(c_) {}
  bool operator<(const MatrixElem& other) const
  {
    return c > other.c || (c == other.c && r > other.r);
  }
};

MarginalCovarianceCholesky::MarginalCovarianceCholesky() :
  _n(0), _Ap(0), _Ai(0), _Ax(0), _perm(0)
{
}

MarginalCovarianceCholesky::~MarginalCovarianceCholesky()
{
}

void MarginalCovarianceCholesky::setCholeskyFactor(int n, int* Lp, int* Li, double* Lx, int* permInv)
{
  _n = n;
  _Ap = Lp;
  _Ai = Li;
  _Ax = Lx;
  _perm = permInv;

  // pre-compute reciprocal values of the diagonal of L
  _diag.resize(n);
  for (int r = 0; r < n; ++r) {
    const int& sc = _Ap[r]; // L is lower triangular, thus the first elem in the column is the diagonal entry
    assert(r == _Ai[sc] && "Error in CCS storage of L");
    _diag[r] = 1.0 / _Ax[sc];
  }
}

double MarginalCovarianceCholesky::computeEntry(int r, int c)
{
  assert(r <= c);
  int idx = computeIndex(r, c);

  LookupMap::const_iterator foundIt = _map.find(idx);
  if (foundIt != _map.end()) {
    return foundIt->second;
  }

  // compute the summation over column r
  double s = 0.;
  const int& sc = _Ap[r];
  const int& ec = _Ap[r+1];
  for (int j = sc+1; j < ec; ++j) { // sum over row r while skipping the element on the diagonal
    const int& rr = _Ai[j];
    double val = rr < c ? computeEntry(rr, c) : computeEntry(c, rr);
    s += val * _Ax[j];
  }

  double result;
  if (r == c) {
    const double& diagElem = _diag[r];
    result = diagElem * (diagElem - s);
  } else {
    result = -s * _diag[r];
  }
  _map[idx] = result;
  return result;
}

void MarginalCovarianceCholesky::computeCovariance(double** covBlocks, const std::vector<int>& blockIndices)
{
  _map.clear();
  int base = 0;
  vector<MatrixElem> elemsToCompute;
  for (size_t i = 0; i < blockIndices.size(); ++i) {
    int nbase = blockIndices[i];
    int vdim = nbase - base;
    for (int rr = 0; rr < vdim; ++rr)
      for (int cc = rr; cc < vdim; ++cc) {
        int r = _perm ? _perm[rr + base] : rr + base; // apply permutation
        int c = _perm ? _perm[cc + base] : cc + base;
        if (r > c) // make sure it's still upper triangular after applying the permutation
          swap(r, c);
        elemsToCompute.push_back(MatrixElem(r, c));
      }
    base = nbase;
  }

  // sort the elems to reduce the recursive calls
  sort(elemsToCompute.begin(), elemsToCompute.end());

  // compute the inverse elements we need
  for (size_t i = 0; i < elemsToCompute.size(); ++i) {
    const MatrixElem& me = elemsToCompute[i];
    computeEntry(me.r, me.c);
  }

  // set the marginal covariance for the vertices, by writing to the blocks memory
  base = 0;
  for (size_t i = 0; i < blockIndices.size(); ++i) {
    int nbase = blockIndices[i];
    int vdim = nbase - base;
    double* cov = covBlocks[i];
    for (int rr = 0; rr < vdim; ++rr)
      for (int cc = rr; cc < vdim; ++cc) {
        int r = _perm ? _perm[rr + base] : rr + base; // apply permutation
        int c = _perm ? _perm[cc + base] : cc + base;
        if (r > c) // upper triangle
          swap(r, c);
        int idx = computeIndex(r, c);
        LookupMap::const_iterator foundIt = _map.find(idx);
        assert(foundIt != _map.end());
        cov[rr*vdim + cc] = foundIt->second;
        if (rr != cc)
          cov[cc*vdim + rr] = foundIt->second;
      }
    base = nbase;
  }
}


void MarginalCovarianceCholesky::computeCovariance(SparseBlockMatrix<MatrixXd>& spinv, const std::vector<int>& rowBlockIndices, const std::vector< std::pair<int, int> >& blockIndices)
{
  // allocate the sparse
  spinv = SparseBlockMatrix<MatrixXd>(&rowBlockIndices[0], 
              &rowBlockIndices[0], 
              rowBlockIndices.size(),
              rowBlockIndices.size(), true);
  _map.clear();
  vector<MatrixElem> elemsToCompute;
  for (size_t i = 0; i < blockIndices.size(); ++i) {
    int blockRow=blockIndices[i].first;    
    int blockCol=blockIndices[i].second;
    assert(blockRow>=0);
    assert(blockRow < (int)rowBlockIndices.size());
    assert(blockCol>=0);
    assert(blockCol < (int)rowBlockIndices.size());

    int rowBase=spinv.rowBaseOfBlock(blockRow);
    int colBase=spinv.colBaseOfBlock(blockCol);
    
    MatrixXd *block=spinv.block(blockRow, blockCol, true);
    assert(block);
    for (int iRow=0; iRow<block->rows(); ++iRow)
      for (int iCol=0; iCol<block->cols(); ++iCol){
  int rr=rowBase+iRow;
  int cc=colBase+iCol;
        int r = _perm ? _perm[rr] : rr; // apply permutation
        int c = _perm ? _perm[cc] : cc;
        if (r > c)
          swap(r, c);
        elemsToCompute.push_back(MatrixElem(r, c));
      }
  }

  // sort the elems to reduce the number of recursive calls
  sort(elemsToCompute.begin(), elemsToCompute.end());

  // compute the inverse elements we need
  for (size_t i = 0; i < elemsToCompute.size(); ++i) {
    const MatrixElem& me = elemsToCompute[i];
    computeEntry(me.r, me.c);
  }

  // set the marginal covariance 
  for (size_t i = 0; i < blockIndices.size(); ++i) {
    int blockRow=blockIndices[i].first;    
    int blockCol=blockIndices[i].second;
    int rowBase=spinv.rowBaseOfBlock(blockRow);
    int colBase=spinv.colBaseOfBlock(blockCol);
    
    MatrixXd *block=spinv.block(blockRow, blockCol);
    assert(block);
    for (int iRow=0; iRow<block->rows(); ++iRow)
      for (int iCol=0; iCol<block->cols(); ++iCol){
  int rr=rowBase+iRow;
  int cc=colBase+iCol;
        int r = _perm ? _perm[rr] : rr; // apply permutation
        int c = _perm ? _perm[cc] : cc;
        if (r > c)
          swap(r, c);
        int idx = computeIndex(r, c);
        LookupMap::const_iterator foundIt = _map.find(idx);
        assert(foundIt != _map.end());
  (*block)(iRow, iCol) = foundIt->second;
      }
  }
}

} // end namespace
