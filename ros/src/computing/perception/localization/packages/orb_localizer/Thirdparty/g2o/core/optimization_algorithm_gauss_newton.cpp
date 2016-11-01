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

#include "g2o/core/optimization_algorithm_gauss_newton.h"

#include <iostream>

#include "g2o/core/batch_stats.h"
#include "g2o/core/solver.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/stuff/timeutil.h"
#include "../stuff/macros.h"

using namespace std;

namespace g2o {

  OptimizationAlgorithmGaussNewton::OptimizationAlgorithmGaussNewton(Solver* solver) :
    OptimizationAlgorithmWithHessian(solver)
  {
  }

  OptimizationAlgorithmGaussNewton::~OptimizationAlgorithmGaussNewton()
  {
  }

  OptimizationAlgorithm::SolverResult OptimizationAlgorithmGaussNewton::solve(int iteration, bool online)
  {
    assert(_optimizer && "_optimizer not set");
    assert(_solver->optimizer() == _optimizer && "underlying linear solver operates on different graph");
    bool ok = true;
    
    //here so that correct component for max-mixtures can be computed before the build structure
    double t=get_monotonic_time();
    _optimizer->computeActiveErrors();
    G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
    if (globalStats) {
      globalStats->timeResiduals = get_monotonic_time()-t;
    }
    
    if (iteration == 0 && !online) { // built up the CCS structure, here due to easy time measure
      ok = _solver->buildStructure();
      if (! ok) {
        cerr << __PRETTY_FUNCTION__ << ": Failure while building CCS structure" << endl;
        return OptimizationAlgorithm::Fail;
      }
    }

    t=get_monotonic_time();
    _solver->buildSystem();
    if (globalStats) {
      globalStats->timeQuadraticForm = get_monotonic_time()-t;
      t=get_monotonic_time();
    }

    ok = _solver->solve();
    if (globalStats) {
      globalStats->timeLinearSolution = get_monotonic_time()-t;
      t=get_monotonic_time();
    }

    _optimizer->update(_solver->x());
    if (globalStats) {
      globalStats->timeUpdate = get_monotonic_time()-t;
    }
    if (ok)
      return OK;
    else
      return Fail;
  }

  void OptimizationAlgorithmGaussNewton::printVerbose(std::ostream& os) const
  {
    os
      << "\t schur= " << _solver->schur();
  }

} // end namespace
