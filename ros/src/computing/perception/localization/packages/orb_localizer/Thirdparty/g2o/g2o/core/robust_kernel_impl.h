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

#ifndef G2O_ROBUST_KERNEL_IMPL_H
#define G2O_ROBUST_KERNEL_IMPL_H

#include "g2o/core/robust_kernel.h"

namespace g2o {

  /**
   * \brief scale a robust kernel to another delta (window size)
   *
   * Scales a robust kernel to another window size. Useful, in case if
   * one implements a kernel which only is designed for a fixed window
   * size.
   */
  class  RobustKernelScaleDelta : public RobustKernel
  {
    public:
      /**
       * construct the scaled kernel ontop of another kernel which might be shared accross
       * several scaled kernels
       */
      explicit RobustKernelScaleDelta(const RobustKernelPtr& kernel, double delta = 1.);
      explicit RobustKernelScaleDelta(double delta = 1.);

      //! return the underlying kernel
      const RobustKernelPtr kernel() const { return _kernel;}
      //! use another kernel for the underlying operation
      void setKernel(const RobustKernelPtr& ptr);

      void robustify(double error, Eigen::Vector3d& rho) const;

    protected:
      RobustKernelPtr _kernel;
  };

  /**
   * \brief Huber Cost Function
   *
   * Loss function as described by Huber
   * See http://en.wikipedia.org/wiki/Huber_loss_function
   *
   * If e^(1/2) < d
   * rho(e) = e
   *
   * else
   *
   *               1/2    2
   * rho(e) = 2 d e    - d
   */
  class  RobustKernelHuber : public RobustKernel
  {
    public:
      virtual void setDelta(double delta);
      virtual void setDeltaSqr(const double &delta, const double &deltaSqr);
      virtual void robustify(double e2, Eigen::Vector3d& rho) const;

    private:
      float dsqr;
  };

   /**
   * \brief Tukey Cost Function
   *
   *
   * If e^(1/2) < d
   * rho(e) = delta2(1-(1-e/delta2)^3)
   *
   * else
   *              
   * rho(e) = delta2
   */
  class  RobustKernelTukey : public RobustKernel
  {
    public:

      virtual void setDeltaSqr(const double &deltaSqr, const double &inv);
      virtual void robustify(double e2, Eigen::Vector3d& rho) const;
    private:
      float _deltaSqr;
      float _invDeltaSqr;
  };


  /**
   * \brief Pseudo Huber Cost Function
   *
   * The smooth pseudo huber cost function:
   * See http://en.wikipedia.org/wiki/Huber_loss_function
   *
   *    2       e
   * 2 d  (sqrt(-- + 1) - 1)
   *             2
   *            d
   */
  class  RobustKernelPseudoHuber : public RobustKernel
  {
    public:
      virtual void robustify(double e2, Eigen::Vector3d& rho) const;
  };

  /**
   * \brief Cauchy cost function
   *
   *  2     e
   * d  log(-- + 1)
   *         2
   *        d
   */
  class  RobustKernelCauchy : public RobustKernel
  {
    public:
      virtual void robustify(double e2, Eigen::Vector3d& rho) const;
  };

  /**
   * \brief Saturated cost function.
   *
   * The error is at most delta^2
   */
  class  RobustKernelSaturated : public RobustKernel
  {
    public:
      virtual void robustify(double e2, Eigen::Vector3d& rho) const;
  };

  /**
   * \brief Dynamic covariance scaling - DCS
   *  
   * See paper Robust Map Optimization from Agarwal et al.  ICRA 2013
   *
   * delta is used as $phi$
   */
  class  RobustKernelDCS : public RobustKernel
  {
    public:
      virtual void robustify(double e2, Eigen::Vector3d& rho) const;
  };

} // end namespace g2o

#endif
