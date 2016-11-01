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

#ifndef G2O_BLOCK_SOLVER_H
#define G2O_BLOCK_SOLVER_H
#include <Eigen/Core>

#include "g2o/core/linear_solver.h"
#include "g2o/core/openmp_mutex.h"
#include "g2o/core/solver.h"
#include "g2o/core/sparse_block_matrix.h"
#include "g2o/core/sparse_block_matrix_diagonal.h"
#include "../config.h"

namespace g2o {
  using namespace Eigen;

  /**
   * \brief traits to summarize the properties of the fixed size optimization problem
   */
  template <int _PoseDim, int _LandmarkDim>
  struct BlockSolverTraits
  {
    static const int PoseDim = _PoseDim;
    static const int LandmarkDim = _LandmarkDim;
    typedef Matrix<double, PoseDim, PoseDim> PoseMatrixType;
    typedef Matrix<double, LandmarkDim, LandmarkDim> LandmarkMatrixType;
    typedef Matrix<double, PoseDim, LandmarkDim> PoseLandmarkMatrixType;
    typedef Matrix<double, PoseDim, 1> PoseVectorType;
    typedef Matrix<double, LandmarkDim, 1> LandmarkVectorType;

    typedef SparseBlockMatrix<PoseMatrixType> PoseHessianType;
    typedef SparseBlockMatrix<LandmarkMatrixType> LandmarkHessianType;
    typedef SparseBlockMatrix<PoseLandmarkMatrixType> PoseLandmarkHessianType;
    typedef LinearSolver<PoseMatrixType> LinearSolverType;
  };

  /**
   * \brief traits to summarize the properties of the dynamic size optimization problem
   */
  template <>
  struct BlockSolverTraits<Eigen::Dynamic, Eigen::Dynamic>
  {
    static const int PoseDim = Eigen::Dynamic;
    static const int LandmarkDim = Eigen::Dynamic;
    typedef MatrixXd PoseMatrixType;
    typedef MatrixXd LandmarkMatrixType;
    typedef MatrixXd PoseLandmarkMatrixType;
    typedef VectorXd PoseVectorType;
    typedef VectorXd LandmarkVectorType;

    typedef SparseBlockMatrix<PoseMatrixType> PoseHessianType;
    typedef SparseBlockMatrix<LandmarkMatrixType> LandmarkHessianType;
    typedef SparseBlockMatrix<PoseLandmarkMatrixType> PoseLandmarkHessianType;
    typedef LinearSolver<PoseMatrixType> LinearSolverType;
  };

  /**
   * \brief base for the block solvers with some basic function interfaces
   */
  class BlockSolverBase : public Solver
  {
    public:
      virtual ~BlockSolverBase() {}
      /**
       * compute dest = H * src
       */
      virtual void multiplyHessian(double* dest, const double* src) const = 0;
  };

  /**
   * \brief Implementation of a solver operating on the blocks of the Hessian
   */
  template <typename Traits>
  class BlockSolver: public BlockSolverBase {
    public:

      static const int PoseDim = Traits::PoseDim;
      static const int LandmarkDim = Traits::LandmarkDim;
      typedef typename Traits::PoseMatrixType PoseMatrixType;
      typedef typename Traits::LandmarkMatrixType LandmarkMatrixType; 
      typedef typename Traits::PoseLandmarkMatrixType PoseLandmarkMatrixType;
      typedef typename Traits::PoseVectorType PoseVectorType;
      typedef typename Traits::LandmarkVectorType LandmarkVectorType;

      typedef typename Traits::PoseHessianType PoseHessianType;
      typedef typename Traits::LandmarkHessianType LandmarkHessianType;
      typedef typename Traits::PoseLandmarkHessianType PoseLandmarkHessianType;
      typedef typename Traits::LinearSolverType LinearSolverType;

    public:

      /**
       * allocate a block solver ontop of the underlying linear solver.
       * NOTE: The BlockSolver assumes exclusive access to the linear solver and will therefore free the pointer
       * in its destructor.
       */
      BlockSolver(LinearSolverType* linearSolver);
      ~BlockSolver();

      virtual bool init(SparseOptimizer* optmizer, bool online = false);
      virtual bool buildStructure(bool zeroBlocks = false);
      virtual bool updateStructure(const std::vector<HyperGraph::Vertex*>& vset, const HyperGraph::EdgeSet& edges);
      virtual bool buildSystem();
      virtual bool solve();
      virtual bool computeMarginals(SparseBlockMatrix<MatrixXd>& spinv, const std::vector<std::pair<int, int> >& blockIndices);
      virtual bool setLambda(double lambda, bool backup = false);
      virtual void restoreDiagonal();
      virtual bool supportsSchur() {return true;}
      virtual bool schur() { return _doSchur;}
      virtual void setSchur(bool s) { _doSchur = s;}

      LinearSolver<PoseMatrixType>* linearSolver() const { return _linearSolver;}

      virtual void setWriteDebug(bool writeDebug);
      virtual bool writeDebug() const {return _linearSolver->writeDebug();}

      virtual bool saveHessian(const std::string& fileName) const;

      virtual void multiplyHessian(double* dest, const double* src) const { _Hpp->multiplySymmetricUpperTriangle(dest, src);}

    protected:
      void resize(int* blockPoseIndices, int numPoseBlocks, 
          int* blockLandmarkIndices, int numLandmarkBlocks, int totalDim);

      void deallocate();

      SparseBlockMatrix<PoseMatrixType>* _Hpp;
      SparseBlockMatrix<LandmarkMatrixType>* _Hll;
      SparseBlockMatrix<PoseLandmarkMatrixType>* _Hpl;

      SparseBlockMatrix<PoseMatrixType>* _Hschur;
      SparseBlockMatrixDiagonal<LandmarkMatrixType>* _DInvSchur;

      SparseBlockMatrixCCS<PoseLandmarkMatrixType>* _HplCCS;
      SparseBlockMatrixCCS<PoseMatrixType>* _HschurTransposedCCS;

      LinearSolver<PoseMatrixType>* _linearSolver;

      std::vector<PoseVectorType, Eigen::aligned_allocator<PoseVectorType> > _diagonalBackupPose;
      std::vector<LandmarkVectorType, Eigen::aligned_allocator<LandmarkVectorType> > _diagonalBackupLandmark;

#    ifdef G2O_OPENMP
      std::vector<OpenMPMutex> _coefficientsMutex;
#    endif

      bool _doSchur;

      double* _coefficients;
      double* _bschur;

      int _numPoses, _numLandmarks;
      int _sizePoses, _sizeLandmarks;
  };


  //variable size solver
  typedef BlockSolver< BlockSolverTraits<Eigen::Dynamic, Eigen::Dynamic> > BlockSolverX;
  // solver for BA/3D SLAM
  typedef BlockSolver< BlockSolverTraits<6, 3> > BlockSolver_6_3;  
  // solver fo BA with scale
  typedef BlockSolver< BlockSolverTraits<7, 3> > BlockSolver_7_3;  
  // 2Dof landmarks 3Dof poses
  typedef BlockSolver< BlockSolverTraits<3, 2> > BlockSolver_3_2;

} // end namespace

#include "g2o/core/block_solver.hpp"


#endif
