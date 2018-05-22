#include "ndt_gpu/MatrixDevice.h"
#include "ndt_gpu/debug.h"

namespace gpu {
MatrixDevice::MatrixDevice(int rows, int cols) {
	rows_ = rows;
	cols_ = cols;
	offset_ = 1;
	fr_ = true;
	buffer_ = NULL;
}

void MatrixDevice::memAlloc()
{
	if (buffer_ != NULL && fr_) {
		checkCudaErrors(cudaFree(buffer_));
		buffer_ = NULL;
	}

	checkCudaErrors(cudaMalloc(&buffer_, sizeof(double) * rows_ * cols_ * offset_));
	checkCudaErrors(cudaMemset(buffer_, 0, sizeof(double) * rows_ * cols_ * offset_));
	checkCudaErrors(cudaDeviceSynchronize());
	fr_ = true;
}

void MatrixDevice::memFree()
{
	if (fr_) {
		if (buffer_ != NULL) {
			checkCudaErrors(cudaFree(buffer_));
			buffer_ = NULL;
		}
	}
}


SquareMatrixDevice::SquareMatrixDevice(int size) :
	MatrixDevice(size, size)
{

}

}
