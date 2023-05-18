// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gtest/gtest.h"
#include "qp_interface/osqp_csc_matrix_conv.hpp"

#include <Eigen/Core>

#include <string>
#include <tuple>
#include <vector>

TEST(TestCscMatrixConv, Nominal)
{
  using qp::calCSCMatrix;
  using qp::CSC_Matrix;

  Eigen::MatrixXd rect1(1, 2);
  rect1 << 0.0, 1.0;

  const CSC_Matrix rect_m1 = calCSCMatrix(rect1);
  ASSERT_EQ(rect_m1.m_vals.size(), size_t(1));
  EXPECT_EQ(rect_m1.m_vals[0], 1.0);
  ASSERT_EQ(rect_m1.m_row_idxs.size(), size_t(1));
  EXPECT_EQ(rect_m1.m_row_idxs[0], c_int(0));
  ASSERT_EQ(rect_m1.m_col_idxs.size(), size_t(3));  // nb of columns + 1
  EXPECT_EQ(rect_m1.m_col_idxs[0], c_int(0));
  EXPECT_EQ(rect_m1.m_col_idxs[1], c_int(0));
  EXPECT_EQ(rect_m1.m_col_idxs[2], c_int(1));

  Eigen::MatrixXd rect2(2, 4);
  rect2 << 1.0, 0.0, 3.0, 0.0, 0.0, 6.0, 7.0, 0.0;

  const CSC_Matrix rect_m2 = calCSCMatrix(rect2);
  ASSERT_EQ(rect_m2.m_vals.size(), size_t(4));
  EXPECT_EQ(rect_m2.m_vals[0], 1.0);
  EXPECT_EQ(rect_m2.m_vals[1], 6.0);
  EXPECT_EQ(rect_m2.m_vals[2], 3.0);
  EXPECT_EQ(rect_m2.m_vals[3], 7.0);
  ASSERT_EQ(rect_m2.m_row_idxs.size(), size_t(4));
  EXPECT_EQ(rect_m2.m_row_idxs[0], c_int(0));
  EXPECT_EQ(rect_m2.m_row_idxs[1], c_int(1));
  EXPECT_EQ(rect_m2.m_row_idxs[2], c_int(0));
  EXPECT_EQ(rect_m2.m_row_idxs[3], c_int(1));
  ASSERT_EQ(rect_m2.m_col_idxs.size(), size_t(5));  // nb of columns + 1
  EXPECT_EQ(rect_m2.m_col_idxs[0], c_int(0));
  EXPECT_EQ(rect_m2.m_col_idxs[1], c_int(1));
  EXPECT_EQ(rect_m2.m_col_idxs[2], c_int(2));
  EXPECT_EQ(rect_m2.m_col_idxs[3], c_int(4));
  EXPECT_EQ(rect_m2.m_col_idxs[4], c_int(4));

  // Example from http://netlib.org/linalg/html_templates/node92.html
  Eigen::MatrixXd square2(6, 6);
  square2 << 10.0, 0.0, 0.0, 0.0, -2.0, 0.0, 3.0, 9.0, 0.0, 0.0, 0.0, 3.0, 0.0, 7.0, 8.0, 7.0, 0.0,
    0.0, 3.0, 0.0, 8.0, 7.0, 5.0, 0.0, 0.0, 8.0, 0.0, 9.0, 9.0, 13.0, 0.0, 4.0, 0.0, 0.0, 2.0, -1.0;

  const CSC_Matrix square_m2 = calCSCMatrix(square2);
  ASSERT_EQ(square_m2.m_vals.size(), size_t(19));
  EXPECT_EQ(square_m2.m_vals[0], 10.0);
  EXPECT_EQ(square_m2.m_vals[1], 3.0);
  EXPECT_EQ(square_m2.m_vals[2], 3.0);
  EXPECT_EQ(square_m2.m_vals[3], 9.0);
  EXPECT_EQ(square_m2.m_vals[4], 7.0);
  EXPECT_EQ(square_m2.m_vals[5], 8.0);
  EXPECT_EQ(square_m2.m_vals[6], 4.0);
  EXPECT_EQ(square_m2.m_vals[7], 8.0);
  EXPECT_EQ(square_m2.m_vals[8], 8.0);
  EXPECT_EQ(square_m2.m_vals[9], 7.0);
  EXPECT_EQ(square_m2.m_vals[10], 7.0);
  EXPECT_EQ(square_m2.m_vals[11], 9.0);
  EXPECT_EQ(square_m2.m_vals[12], -2.0);
  EXPECT_EQ(square_m2.m_vals[13], 5.0);
  EXPECT_EQ(square_m2.m_vals[14], 9.0);
  EXPECT_EQ(square_m2.m_vals[15], 2.0);
  EXPECT_EQ(square_m2.m_vals[16], 3.0);
  EXPECT_EQ(square_m2.m_vals[17], 13.0);
  EXPECT_EQ(square_m2.m_vals[18], -1.0);
  ASSERT_EQ(square_m2.m_row_idxs.size(), size_t(19));
  EXPECT_EQ(square_m2.m_row_idxs[0], c_int(0));
  EXPECT_EQ(square_m2.m_row_idxs[1], c_int(1));
  EXPECT_EQ(square_m2.m_row_idxs[2], c_int(3));
  EXPECT_EQ(square_m2.m_row_idxs[3], c_int(1));
  EXPECT_EQ(square_m2.m_row_idxs[4], c_int(2));
  EXPECT_EQ(square_m2.m_row_idxs[5], c_int(4));
  EXPECT_EQ(square_m2.m_row_idxs[6], c_int(5));
  EXPECT_EQ(square_m2.m_row_idxs[7], c_int(2));
  EXPECT_EQ(square_m2.m_row_idxs[8], c_int(3));
  EXPECT_EQ(square_m2.m_row_idxs[9], c_int(2));
  EXPECT_EQ(square_m2.m_row_idxs[10], c_int(3));
  EXPECT_EQ(square_m2.m_row_idxs[11], c_int(4));
  EXPECT_EQ(square_m2.m_row_idxs[12], c_int(0));
  EXPECT_EQ(square_m2.m_row_idxs[13], c_int(3));
  EXPECT_EQ(square_m2.m_row_idxs[14], c_int(4));
  EXPECT_EQ(square_m2.m_row_idxs[15], c_int(5));
  EXPECT_EQ(square_m2.m_row_idxs[16], c_int(1));
  EXPECT_EQ(square_m2.m_row_idxs[17], c_int(4));
  EXPECT_EQ(square_m2.m_row_idxs[18], c_int(5));
  ASSERT_EQ(square_m2.m_col_idxs.size(), size_t(7));  // nb of columns + 1
  EXPECT_EQ(square_m2.m_col_idxs[0], c_int(0));
  EXPECT_EQ(square_m2.m_col_idxs[1], c_int(3));
  EXPECT_EQ(square_m2.m_col_idxs[2], c_int(7));
  EXPECT_EQ(square_m2.m_col_idxs[3], c_int(9));
  EXPECT_EQ(square_m2.m_col_idxs[4], c_int(12));
  EXPECT_EQ(square_m2.m_col_idxs[5], c_int(16));
  EXPECT_EQ(square_m2.m_col_idxs[6], c_int(19));
}
TEST(TestCscMatrixConv, Trapezoidal)
{
  using qp::calCSCMatrixTrapezoidal;
  using qp::CSC_Matrix;

  Eigen::MatrixXd square1(2, 2);
  Eigen::MatrixXd square2(3, 3);
  Eigen::MatrixXd rect1(1, 2);
  square1 << 1.0, 2.0, 2.0, 4.0;
  square2 << 0.0, 2.0, 0.0, 4.0, 5.0, 6.0, 0.0, 0.0, 0.0;
  rect1 << 0.0, 1.0;

  const CSC_Matrix square_m1 = calCSCMatrixTrapezoidal(square1);
  // Trapezoidal: skip the lower left triangle (2.0 in this example)
  ASSERT_EQ(square_m1.m_vals.size(), size_t(3));
  EXPECT_EQ(square_m1.m_vals[0], 1.0);
  EXPECT_EQ(square_m1.m_vals[1], 2.0);
  EXPECT_EQ(square_m1.m_vals[2], 4.0);
  ASSERT_EQ(square_m1.m_row_idxs.size(), size_t(3));
  EXPECT_EQ(square_m1.m_row_idxs[0], c_int(0));
  EXPECT_EQ(square_m1.m_row_idxs[1], c_int(0));
  EXPECT_EQ(square_m1.m_row_idxs[2], c_int(1));
  ASSERT_EQ(square_m1.m_col_idxs.size(), size_t(3));
  EXPECT_EQ(square_m1.m_col_idxs[0], c_int(0));
  EXPECT_EQ(square_m1.m_col_idxs[1], c_int(1));
  EXPECT_EQ(square_m1.m_col_idxs[2], c_int(3));

  const CSC_Matrix square_m2 = calCSCMatrixTrapezoidal(square2);
  ASSERT_EQ(square_m2.m_vals.size(), size_t(3));
  EXPECT_EQ(square_m2.m_vals[0], 2.0);
  EXPECT_EQ(square_m2.m_vals[1], 5.0);
  EXPECT_EQ(square_m2.m_vals[2], 6.0);
  ASSERT_EQ(square_m2.m_row_idxs.size(), size_t(3));
  EXPECT_EQ(square_m2.m_row_idxs[0], c_int(0));
  EXPECT_EQ(square_m2.m_row_idxs[1], c_int(1));
  EXPECT_EQ(square_m2.m_row_idxs[2], c_int(1));
  ASSERT_EQ(square_m2.m_col_idxs.size(), size_t(4));
  EXPECT_EQ(square_m2.m_col_idxs[0], c_int(0));
  EXPECT_EQ(square_m2.m_col_idxs[1], c_int(0));
  EXPECT_EQ(square_m2.m_col_idxs[2], c_int(2));
  EXPECT_EQ(square_m2.m_col_idxs[3], c_int(3));

  try {
    const CSC_Matrix rect_m1 = calCSCMatrixTrapezoidal(rect1);
    FAIL() << "calCSCMatrixTrapezoidal should fail with non-square inputs";
  } catch (const std::invalid_argument & e) {
    EXPECT_EQ(e.what(), std::string("Matrix must be square (n, n)"));
  }
}
TEST(TestCscMatrixConv, Print)
{
  using qp::calCSCMatrix;
  using qp::calCSCMatrixTrapezoidal;
  using qp::CSC_Matrix;
  using qp::printCSCMatrix;
  Eigen::MatrixXd square1(2, 2);
  Eigen::MatrixXd rect1(1, 2);
  square1 << 1.0, 2.0, 2.0, 4.0;
  rect1 << 0.0, 1.0;
  const CSC_Matrix square_m1 = calCSCMatrixTrapezoidal(square1);
  const CSC_Matrix rect_m1 = calCSCMatrix(rect1);
  printCSCMatrix(square_m1);
  printCSCMatrix(rect_m1);
}
