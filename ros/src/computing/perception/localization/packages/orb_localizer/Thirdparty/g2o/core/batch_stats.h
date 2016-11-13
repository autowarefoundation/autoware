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

#ifndef G2O_BATCH_STATS_H_
#define G2O_BATCH_STATS_H_

#include <iostream>
#include <vector>


namespace g2o {

  /**
   * \brief statistics about the optimization
   */
  struct  G2OBatchStatistics {
    G2OBatchStatistics();
    int iteration;                    ///< which iteration
    int numVertices;                  ///< how many vertices are involved
    int numEdges;                     ///< how many edges
    double chi2;                      ///< total chi2

    /** timings **/
    // nonlinear part
    double timeResiduals;             ///< residuals
    double timeLinearize;             ///< jacobians
    double timeQuadraticForm;         ///< construct the quadratic form in the graph
    int levenbergIterations;          ///< number of iterations performed by LM
    // block_solver (constructs Ax=b, plus maybe schur)
    double timeSchurComplement;      ///< compute schur complement (0 if not done)

    // linear solver (computes Ax=b);
    double timeSymbolicDecomposition; ///< symbolic decomposition (0 if not done)
    double timeNumericDecomposition;  ///< numeric decomposition  (0 if not done)
    double timeLinearSolution;        ///< total time for solving Ax=b (including detup for schur)
    double timeLinearSolver;          ///< time for solving, excluding Schur setup
    int    iterationsLinearSolver;    ///< iterations of PCG, (0 if not used, i.e., Cholesky)
    double timeUpdate;                ///< time to apply the update
    double timeIteration;             ///< total time;

    double timeMarginals;             ///< computing the inverse elements (solve blocks) and thus the marginal covariances

    // information about the Hessian matrix
    size_t hessianDimension;          ///< rows / cols of the Hessian
    size_t hessianPoseDimension;      ///< dimension of the pose matrix in Schur
    size_t hessianLandmarkDimension;  ///< dimension of the landmark matrix in Schur
    size_t choleskyNNZ;               ///< number of non-zeros in the cholesky factor

    static G2OBatchStatistics* globalStats() {return _globalStats;}
    static void setGlobalStats(G2OBatchStatistics* b);
    protected:
    static G2OBatchStatistics* _globalStats;
  };

   std::ostream& operator<<(std::ostream&, const G2OBatchStatistics&);

  typedef std::vector<G2OBatchStatistics> BatchStatisticsContainer;
}

#endif
