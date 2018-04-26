#include "ndt_gpu/SymmetricEigenSolver.h"
#include "ndt_gpu/debug.h"

namespace gpu {

SymmetricEigensolver3x3::SymmetricEigensolver3x3(int offset)
{
	offset_ = offset;

	checkCudaErrors(cudaMalloc(&buffer_, sizeof(double) * 18 * offset_));
	checkCudaErrors(cudaMalloc(&maxAbsElement_, sizeof(double) * offset_));
	checkCudaErrors(cudaMalloc(&norm_, sizeof(double) * offset_));
	checkCudaErrors(cudaMalloc(&i02_, sizeof(int) * 2 * offset_));

	eigenvectors_ = NULL;
	eigenvalues_ = NULL;
	input_matrices_ = NULL;

	is_copied_ = false;
}

void SymmetricEigensolver3x3::setInputMatrices(double *input_matrices)
{
	input_matrices_ = input_matrices;
}

void SymmetricEigensolver3x3::setEigenvectors(double *eigenvectors)
{
	eigenvectors_ = eigenvectors;
}

void SymmetricEigensolver3x3::setEigenvalues(double *eigenvalues)
{
	eigenvalues_ = eigenvalues;
}

double* SymmetricEigensolver3x3::getBuffer() const
{
	return buffer_;
}

void SymmetricEigensolver3x3::memFree()
{
	if (!is_copied_) {
		if (buffer_ != NULL) {
			checkCudaErrors(cudaFree(buffer_));
			buffer_ = NULL;
		}

		if (maxAbsElement_ != NULL) {
			checkCudaErrors(cudaFree(maxAbsElement_));
			maxAbsElement_ = NULL;
		}

		if (norm_ != NULL) {
			checkCudaErrors(cudaFree(norm_));
			norm_ = NULL;
		}

		if (i02_ != NULL) {
			checkCudaErrors(cudaFree(i02_));
			i02_ = NULL;
		}
	}
}
}
