#ifndef MATRIX_HOST_H_
#define MATRIX_HOST_H_

#include "Matrix.h"
#include "MatrixDevice.h"

namespace gpu {
class MatrixHost : public Matrix {
public:
	MatrixHost();
	MatrixHost(int rows, int cols);
	MatrixHost(int rows, int cols, int offset, double *buffer);
	MatrixHost(const MatrixHost& other);

	bool moveToGpu(MatrixDevice output);
	bool moveToHost(MatrixDevice input);

	MatrixHost &operator=(const MatrixHost &other);

	void debug();

	~MatrixHost();
private:
	bool fr_;
};

class SquareMatrixHost: public MatrixHost {
public:
	SquareMatrixHost(int size);
};

}

#endif
