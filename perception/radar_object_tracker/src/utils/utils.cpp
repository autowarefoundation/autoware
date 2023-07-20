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

#include "radar_object_tracker/utils/utils.hpp"

namespace utils
{
// concatenate matrices vertically
Eigen::MatrixXd stackMatricesVertically(const std::vector<Eigen::MatrixXd> & matrices)
{
  int totalRows = 0;
  int cols = -1;

  // calculate total number of rows and check that all matrices have the same number of columns
  for (const auto & matrix : matrices) {
    totalRows += matrix.rows();
    if (cols == -1) {
      cols = matrix.cols();
    } else if (cols != matrix.cols()) {
      throw std::invalid_argument("All matrices must have the same number of columns.");
    }
  }

  Eigen::MatrixXd result(totalRows, cols);

  int currentRow = 0;
  for (const auto & matrix : matrices) {
    // copy each matrix into result
    result.block(currentRow, 0, matrix.rows(), cols) = matrix;
    currentRow += matrix.rows();
  }
  return result;
}

// concatenate matrices diagonally
Eigen::MatrixXd stackMatricesDiagonally(const std::vector<Eigen::MatrixXd> & matrices)
{
  int dimension = 0;

  // calc dimension of result matrix
  for (const auto & matrix : matrices) {
    if (matrix.rows() != matrix.cols()) {
      throw std::invalid_argument("All matrices must be square.");
    }
    dimension += matrix.rows();
  }

  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(dimension, dimension);

  int currentDimension = 0;
  for (const auto & matrix : matrices) {
    // copy each matrix into result
    result.block(currentDimension, currentDimension, matrix.rows(), matrix.cols()) = matrix;
    currentDimension += matrix.rows();
  }

  return result;
}

}  // namespace utils
