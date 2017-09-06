#ifndef GMATRIX_H_
#define GMATRIX_H_

#include <cuda.h>
#include <cuda_runtime.h>
#include "common.h"
#include <float.h>

namespace gpu {

class Matrix {
public:
	CUDAH Matrix() {
		buffer_ = NULL;
		rows_ = cols_ = offset_ = 0;
	}

	CUDAH Matrix(int rows, int cols, int offset, double *buffer) {
		rows_ = rows;
		cols_ = cols;
		offset_ = offset;
		buffer_ = buffer;
	}

	CUDAH int rows() const {
		return rows_;
	}

	CUDAH int cols() const {
		return cols_;
	}

	CUDAH int offset() const {
		return offset_;
	}

	CUDAH double *buffer() const {
		return buffer_;
	}

	CUDAH void setRows(int rows) { rows_ = rows; }
	CUDAH void setCols(int cols) { cols_ = cols; }
	CUDAH void setOffset(int offset) { offset_ = offset; }
	CUDAH void setBuffer(double *buffer) { buffer_ = buffer; }
	CUDAH void setCellVal(int row, int col, double val) {
		buffer_[(row * cols_ + col) * offset_] = val;
	}

	CUDAH void copy(Matrix &output) {
		for (int i = 0; i < rows_; i++) {
			for (int j = 0; j < cols_; j++) {
				output(i, j) = buffer_[(i * cols_ + j) * offset_];
			}
		}
	}

	//Need to fix. Only reducing rows is OK now.
	CUDAH void resize(int rows, int cols) {
		rows_ = rows;
		cols_ = cols;
	}

	CUDAH double *cellAddr(int row, int col) {
		if (row >= rows_ || col >= cols_ || row < 0 || col < 0)
			return NULL;

		return buffer_ + (row * cols_ + col) * offset_;
	}

	CUDAH double *cellAddr(int index) {
		if (rows_ == 1 && index >= 0 && index < cols_) {
				return buffer_ + index * offset_;
		} 
		else if (cols_ == 1 && index >= 0 && index < rows_) {
				return buffer_ + index * offset_;
		}

		return NULL;
	}

	//Assignment operator
	CUDAH void operator=(const Matrix input) {
		rows_ = input.rows_;
		cols_ = input.cols_;
		offset_ = input.offset_;
		buffer_ = input.buffer_;
	}

	CUDAH double& operator()(int row, int col) {
		return buffer_[(row * cols_ + col) * offset_];
	}

	CUDAH void set(int row, int col, double val) {
		buffer_[(row * cols_ + col) * offset_] = val;
	}

	CUDAH double& operator()(int index) {
		return buffer_[index * offset_];
	}

	CUDAH double at(int row, int col) const {
		return buffer_[(row * cols_ + col) * offset_];
	}

	CUDAH bool operator*=(double val) {
		for (int i = 0; i < rows_; i++) {
			for (int j = 0; j < cols_; j++) {
				buffer_[(i * cols_ + j) * offset_] *= val;
			}
		}

		return true;
	}

	CUDAH bool operator/=(double val) {
		if (val == 0)
			return false;

		for (int i = 0; i < rows_ * cols_; i++) {
				buffer_[i * offset_] /= val;
		}

		return true;
	}

	CUDAH bool transpose(Matrix &output) {
		if (rows_ != output.cols_ || cols_ != output.rows_)
			return false;

		for (int i = 0; i < rows_; i++) {
			for (int j = 0; j < cols_; j++) {
				output(j, i) = buffer_[(i * cols_ + j) * offset_];
			}
		}

		return true;
	}

	//Only applicable for 3x3 matrix or below
	CUDAH bool inverse(Matrix &output) {
		if (rows_ != cols_ || rows_ == 0 || cols_ == 0)
			return false;

		if (rows_ == 1) {
			if (buffer_[0] != 0)
				output(0, 0) = 1 / buffer_[0];
			else
				return false;
		}

		if (rows_ == 2) {
			double det = at(0, 0) * at(1, 1) - at(0, 1) * at(1, 0);

			if (det != 0) {
				output(0, 0) = at(1, 1) / det;
				output(0, 1) = - at(0, 1) / det;

				output(1, 0) = - at(1, 0) / det;
				output(1, 1) = at(0, 0) / det;
			} else
				return false;
		}

		if (rows_ == 3) {
			double det = at(0, 0) * at(1, 1) * at(2, 2) + at(0, 1) * at(1, 2) * at(2, 0) + at(1, 0) * at (2, 1) * at(0, 2)
							- at(0, 2) * at(1, 1) * at(2, 0) - at(0, 1) * at(1, 0) * at(2, 2) - at(0, 0) * at(1, 2) * at(2, 1);
			double idet = 1.0 / det;

			if (det != 0) {
				output(0, 0) = (at(1, 1) * at(2, 2) - at(1, 2) * at(2, 1)) * idet;
				output(0, 1) = - (at(0, 1) * at(2, 2) - at(0, 2) * at(2, 1)) * idet;
				output(0, 2) = (at(0, 1) * at(1, 2) - at(0, 2) * at(1, 1)) * idet;

				output(1, 0) = - (at(1, 0) * at(2, 2) - at(1, 2) * at(2, 0)) * idet;
				output(1, 1) = (at(0, 0) * at(2, 2) - at(0, 2) * at(2, 0)) * idet;
				output(1, 2) = - (at(0, 0) * at(1, 2) - at(0, 2) * at(1, 0)) * idet;

				output(2, 0) = (at(1, 0) * at(2, 1) - at(1, 1) * at(2, 0)) * idet;
				output(2, 1) = - (at(0, 0) * at(2, 1) - at(0, 1) * at(2, 0)) * idet;
				output(2, 2) = (at(0, 0) * at(1, 1) - at(0, 1) * at(1, 0)) * idet;
			} else
				return false;
		}

		return true;
	}

	CUDAH Matrix col(int index) {
		return Matrix(rows_, 1, offset_ * cols_, buffer_ + index * offset_);
	}

	CUDAH Matrix row(int index) {
		return Matrix(1, cols_, offset_, buffer_ + index * cols_ * offset_);
	}

protected:
	double *buffer_;
	int rows_, cols_, offset_;
};

}

#endif


