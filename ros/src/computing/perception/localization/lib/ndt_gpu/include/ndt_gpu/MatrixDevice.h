#ifndef MATRIX_DEVICE_H_
#define MATRIX_DEVICE_H_

#include "Matrix.h"

namespace gpu {
class MatrixDevice : public Matrix {
public:
	CUDAH MatrixDevice();

	MatrixDevice(int rows, int cols);

	CUDAH MatrixDevice(int rows, int cols, int offset, double *buffer);

	CUDAH bool isEmpty();

	CUDAH MatrixDevice col(int index);

	CUDAH MatrixDevice row(int index);

	CUDAH void setBuffer(double *buffer);

	void memAlloc();

	void memFree();

private:
	bool fr_;
};

CUDAH MatrixDevice::MatrixDevice()
{
	rows_ = cols_ = offset_ = 0;
	buffer_ = NULL;
	fr_ = true;
}

CUDAH MatrixDevice::MatrixDevice(int rows, int cols, int offset, double *buffer)
{
	rows_ = rows;
	cols_ = cols;
	offset_ = offset;
	buffer_ = buffer;
	fr_ = false;
}

CUDAH bool MatrixDevice::isEmpty()
{
	return (rows_ == 0 || cols_ == 0 || buffer_ == NULL);
}

CUDAH MatrixDevice MatrixDevice::col(int index)
{
	return MatrixDevice(rows_, 1, offset_ * cols_, buffer_ + index * offset_);
}

CUDAH MatrixDevice MatrixDevice::row(int index)
{
	return MatrixDevice(1, cols_, offset_, buffer_ + index * cols_ * offset_);
}

CUDAH void MatrixDevice::setBuffer(double *buffer)
{
	buffer_ = buffer;
}



class SquareMatrixDevice : public MatrixDevice {
public:
	SquareMatrixDevice(int size);
};

}

#endif
