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

#include "g2o/core/sparse_block_matrix.h"

#include <iostream>

using namespace std;
using namespace g2o;
using namespace Eigen;

typedef SparseBlockMatrix< MatrixXd >
SparseBlockMatrixX;

std::ostream& operator << (std::ostream& os, const SparseBlockMatrixX::SparseMatrixBlock& m) {
  for (int i=0; i<m.rows(); ++i){
    for (int j=0; j<m.cols(); ++j)
      cerr << m(i,j) << " ";
    cerr << endl;
  }
  return os;
}

int main (int argc, char** argv){
  int rcol[] = {3,6,8,12};
  int ccol[] = {2,4,13};
  cerr << "creation" << endl;
  SparseBlockMatrixX* M=new SparseBlockMatrixX(rcol, ccol, 4,3);

  cerr << "block access" << endl;

  SparseBlockMatrixX::SparseMatrixBlock* b=M->block(0,0, true);
  cerr << b->rows() << " " << b->cols() << endl;
  for (int i=0; i<b->rows(); ++i)
    for (int j=0; j<b->cols(); ++j){
      (*b)(i,j)=i*b->cols()+j;
    }


  cerr << "block access 2" << endl;
  b=M->block(0,2, true);
  cerr << b->rows() << " " << b->cols() << endl;
  for (int i=0; i<b->rows(); ++i)
    for (int j=0; j<b->cols(); ++j){
      (*b)(i,j)=i*b->cols()+j;
    }

  b=M->block(3,2, true);
  cerr << b->rows() << " " << b->cols() << endl;
  for (int i=0; i<b->rows(); ++i)
    for (int j=0; j<b->cols(); ++j){
      (*b)(i,j)=i*b->cols()+j;
    }

  cerr << *M << endl;

  cerr << "SUM" << endl;

  SparseBlockMatrixX* Ms=0;
  M->add(Ms);
  M->add(Ms);
  cerr << *Ms;
  
  SparseBlockMatrixX* Mt=0;
  M->transpose(Mt);
  cerr << *Mt << endl;

  SparseBlockMatrixX* Mp=0;
  M->multiply(Mp, Mt);
  cerr << *Mp << endl;
  
  int iperm[]={3,2,1,0};
  SparseBlockMatrixX* PMp=0;

  Mp->symmPermutation(PMp,iperm, false);
  cerr << *PMp << endl;

  PMp->clear(true);
  Mp->block(3,0)->fill(0.);
  Mp->symmPermutation(PMp,iperm, true);
  cerr << *PMp << endl;
  
  
  
}
