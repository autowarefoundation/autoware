/**
 * @file   gtest.hpp
 * @author Paul Furgale <paul.furgale@gmail.com>
 * @date   Mon Dec 12 11:54:20 2011
 * @brief  Code to simplify Eigen integration into GTest. Pretty basic but the error messages are good.
 */

#pragma once

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <iostream>
#include <cmath>

namespace grid_map {

    template<int N>
    Eigen::Matrix<double,N,N> randomCovariance()
    {
      Eigen::Matrix<double,N,N> U;
      U.setRandom();
      return U.transpose() * U + 5.0 * Eigen::Matrix<double,N,N>::Identity();
    }

    inline Eigen::MatrixXd randomCovarianceXd(int N)
    {
        Eigen::MatrixXd U(N,N);
        U.setRandom();
        return U.transpose() * U + 5.0 * Eigen::MatrixXd::Identity(N,N);
    }

    template<typename M1, typename M2>
    void assertEqual(const M1 & A, const M2 & B, std::string const & message = "")
    {
        ASSERT_EQ((size_t)A.rows(),(size_t)B.rows()) << message << "\nMatrix A:\n" << A << "\nand matrix B\n" << B << "\nare not the same\n";
      ASSERT_EQ((size_t)A.cols(),(size_t)B.cols()) << message << "\nMatrix A:\n" << A << "\nand matrix B\n" << B << "\nare not the same\n";

      for(int r = 0; r < A.rows(); r++)
		{
		  for(int c = 0; c < A.cols(); c++)
			{
		    if (std::isnan(A(r,c))) {
		      ASSERT_TRUE(std::isnan(B(r,c)));
		    } else {
			    ASSERT_EQ(A(r,c),B(r,c)) << message << "\nEquality comparison failed at (" << r << "," << c << ")\n"
                                       << "\nMatrix A:\n" << A << "\nand matrix B\n" << B;
		    }
			}
		}
    }

    template<typename M1, typename M2, typename T>
    void assertNear(const M1 & A, const M2 & B, T tolerance, std::string const & message = "")
    {
      // Note: If these assertions fail, they only abort this subroutine.
      // see: http://code.google.com/p/googletest/wiki/AdvancedGuide#Using_Assertions_in_Sub-routines
      // \todo better handling of this
        ASSERT_EQ((size_t)A.rows(),(size_t)B.rows()) << message << "\nMatrix A:\n" << A << "\nand matrix B\n" << B << "\nare not the same\n";
      ASSERT_EQ((size_t)A.cols(),(size_t)B.cols()) << message << "\nMatrix A:\n" << A << "\nand matrix B\n" << B << "\nare not the same\n";

      for(int r = 0; r < A.rows(); r++)
		{
		  for(int c = 0; c < A.cols(); c++)
			{
		    if (std::isnan(A(r,c))) {
		      ASSERT_TRUE(std::isnan(B(r,c)));
		    } else {
			    ASSERT_NEAR(A(r,c),B(r,c),tolerance) << message << "\nTolerance comparison failed at (" << r << "," << c << ")\n"
												   << "\nMatrix A:\n" << A << "\nand matrix B\n" << B;
		    }
			}
		}
    }

    template<typename M1, typename M2, typename T>
    void expectNear(const M1 & A, const M2 & B, T tolerance, std::string const & message = "")
    {
      EXPECT_EQ((size_t)A.rows(),(size_t)B.rows()) << message << "\nMatrix A:\n" << A << "\nand matrix B\n" << B << "\nare not the same\n";
      EXPECT_EQ((size_t)A.cols(),(size_t)B.cols()) << message << "\nMatrix A:\n" << A << "\nand matrix B\n" << B << "\nare not the same\n";

      for(int r = 0; r < A.rows(); r++)
        {
          for(int c = 0; c < A.cols(); c++)
            {
              if (std::isnan(A(r,c))) {
                EXPECT_TRUE(std::isnan(B(r,c)));
              } else {
                EXPECT_NEAR(A(r,c),B(r,c),tolerance) << message << "\nTolerance comparison failed at (" << r << "," << c << ")\n"
                             << "\nMatrix A:\n" << A << "\nand matrix B\n" << B;
              }
            }
        }
    }

    template<typename M1>
    void assertFinite(const M1 & A, std::string const & message = "")
    {
      for(int r = 0; r < A.rows(); r++)
		{
		  for(int c = 0; c < A.cols(); c++)
			{
			  ASSERT_TRUE(std::isfinite(A(r,c))) << std::endl << "Check for finite values failed at A(" << r << "," << c << "). Matrix A:" << std::endl << A << std::endl;
			}
		}
    }

    inline bool compareRelative(double a, double b, double percentTolerance, double * percentError = NULL)
    {
	  // \todo: does anyone have a better idea?
      double fa = fabs(a);
      double fb = fabs(b);
      if( (fa < 1e-15 && fb < 1e-15) ||  // Both zero.
		  (fa == 0.0  && fb < 1e-6)  ||  // One exactly zero and the other small
		  (fb == 0.0  && fa < 1e-6) )    // ditto
		return true;

      double diff = fabs(a - b)/std::max(fa,fb);
      if(diff > percentTolerance * 1e-2)
		{
		  if(percentError)
			*percentError = diff * 100.0;
		  return false;
		}
      return true;
    }

#define ASSERT_DOUBLE_MX_EQ(A, B, PERCENT_TOLERANCE, MSG)				\
    ASSERT_EQ((size_t)(A).rows(), (size_t)(B).rows())  << MSG << "\nMatrix " << #A << ":\n" << A << "\nand matrix " << #B << "\n" << B << "\nare not the same size"; \
    ASSERT_EQ((size_t)(A).cols(), (size_t)(B).cols())  << MSG << "\nMatrix " << #A << ":\n" << A << "\nand matrix " << #B << "\n" << B << "\nare not the same size"; \
    for(int r = 0; r < (A).rows(); r++)									\
      {																	\
		for(int c = 0; c < (A).cols(); c++)								\
		  {																\
			double percentError = 0.0;									\
			ASSERT_TRUE(grid_map::compareRelative( (A)(r,c), (B)(r,c), PERCENT_TOLERANCE, &percentError)) \
			  << MSG << "\nComparing:\n"								\
			  << #A << "(" << r << "," << c << ") = " << (A)(r,c) << std::endl \
			  << #B << "(" << r << "," << c << ") = " << (B)(r,c) << std::endl \
			  << "Error was " << percentError << "% > " << PERCENT_TOLERANCE << "%\n" \
			  << "\nMatrix " << #A << ":\n" << A << "\nand matrix " << #B << "\n" << B; \
		  }																\
      }

#define ASSERT_DOUBLE_SPARSE_MX_EQ(A, B, PERCENT_TOLERANCE, MSG)       \
    ASSERT_EQ((size_t)(A).rows(), (size_t)(B).rows())  << MSG << "\nMatrix " << #A << ":\n" << A << "\nand matrix " << #B << "\n" << B << "\nare not the same size"; \
    ASSERT_EQ((size_t)(A).cols(), (size_t)(B).cols())  << MSG << "\nMatrix " << #A << ":\n" << A << "\nand matrix " << #B << "\n" << B << "\nare not the same size"; \
    for(int r = 0; r < (A).rows(); r++)                 \
      {                                 \
    for(int c = 0; c < (A).cols(); c++)               \
      {                               \
      double percentError = 0.0;                  \
      ASSERT_TRUE(grid_map::compareRelative( (A).coeff(r,c), (B).coeff(r,c), PERCENT_TOLERANCE, &percentError)) \
        << MSG << "\nComparing:\n"                \
        << #A << "(" << r << "," << c << ") = " << (A).coeff(r,c) << std::endl \
        << #B << "(" << r << "," << c << ") = " << (B).coeff(r,c) << std::endl \
        << "Error was " << percentError << "% > " << PERCENT_TOLERANCE << "%\n" \
        << "\nMatrix " << #A << ":\n" << A << "\nand matrix " << #B << "\n" << B; \
      }                               \
      }

} // namespace
