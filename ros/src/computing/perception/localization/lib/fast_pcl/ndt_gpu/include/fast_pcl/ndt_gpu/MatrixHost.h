#ifndef MATRIX_HOST_H_
#define MATRIX_HOST_H_

#include "Matrix.h"
#include "MatrixDevice.h"

namespace gpu {
class MatrixHost : public Matrix {
public:
	MatrixHost() : Matrix() { fr_ = false; }
	MatrixHost(int rows, int cols);
	MatrixHost(const MatrixHost& other);


	MatrixHost(int rows, int cols, int offset, double *buffer) {
		rows_ = rows;
		cols_ = cols;
		offset_ = offset;
		buffer_ = buffer;
		fr_ = false;
	}

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
	SquareMatrixHost(int size) : MatrixHost(size, size) {};
};

}

#endif
