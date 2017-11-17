#ifndef GSYMMETRIC_EIGEN_
#define GSYMMETRIC_EIGEN_

#include <cuda.h>
#include <cuda_runtime.h>
#include "MatrixDevice.h"
#include <math.h>

namespace gpu {

class SymmetricEigensolver3x3 {
public:
	CUDAH SymmetricEigensolver3x3();

	SymmetricEigensolver3x3(int offset);

	CUDAH SymmetricEigensolver3x3(const SymmetricEigensolver3x3& other);

	void setInputMatrices(double *input_matrices);

	void setEigenvectors(double *eigenvectors);

	void setEigenvalues(double *eigenvalues);

	double *getBuffer() const;

	/* Normalize input matrices by dividing each matrix
	 * to the element that has the largest absolute value
	 * in each matrix */
	CUDAH void normalizeInput(int tid);

	/* Compute eigenvalues */
	CUDAH void computeEigenvalues(int tid);

	/* First step to compute the eigenvector 0
	 * Because computing the eigenvector 0 using
	 * only one kernel is too expensive (which causes
	 * the "too many resources requested for launch" error,
	 * I have to divide them into two distinct kernels. */
	CUDAH void computeEigenvector00(int tid);

	/* Second step to compute the eigenvector 0 */
	CUDAH void computeEigenvector01(int tid);

	/* First step to compute the eigenvector 1 */
	CUDAH void computeEigenvector10(int tid);

	/* Second step to compute the eigenvector 1 */
	CUDAH void computeEigenvector11(int tid);

	/* Compute the final eigenvector by crossing
	 * eigenvector 0 and eigenvector 1 */
	CUDAH void computeEigenvector2(int tid);

	/* Final step to compute eigenvalues */
	CUDAH void updateEigenvalues(int tid);

	/* Free memory */
	void memFree();

private:
	CUDAH void computeOrthogonalComplement(MatrixDevice w, MatrixDevice u, MatrixDevice v);

	//Operators
	CUDAH void multiply(MatrixDevice u, double mult, MatrixDevice output);

	CUDAH void subtract(MatrixDevice u, MatrixDevice v, MatrixDevice output);

	CUDAH void divide(MatrixDevice u, double div, MatrixDevice output);

	CUDAH double dot(MatrixDevice u, MatrixDevice v);

	CUDAH void cross(MatrixDevice in0, MatrixDevice in1, MatrixDevice out);

	int offset_;

	// Buffers for intermediate calculation
	double *buffer_;
	int *i02_;
	double *maxAbsElement_;
	double *norm_;

	double *eigenvectors_;
	double *eigenvalues_;
	double *input_matrices_;

	bool is_copied_;
};


CUDAH SymmetricEigensolver3x3::SymmetricEigensolver3x3()
{
		buffer_ = NULL;
		eigenvectors_ = NULL;
		eigenvalues_ = NULL;
		input_matrices_ = NULL;
		maxAbsElement_ = NULL;
		norm_ = NULL;
		i02_ = NULL;
		offset_ = 0;
		is_copied_ = false;
}

CUDAH SymmetricEigensolver3x3::SymmetricEigensolver3x3(const SymmetricEigensolver3x3& other)
{
	buffer_ = other.buffer_;
	offset_ = other.offset_;
	eigenvectors_ = other.eigenvectors_;
	eigenvalues_ = other.eigenvalues_;
	input_matrices_ = other.input_matrices_;

	maxAbsElement_ = other.maxAbsElement_;
	norm_ = other.norm_;
	i02_ = other.i02_;
	is_copied_ = true;
}

CUDAH void SymmetricEigensolver3x3::normalizeInput(int tid)
{
	MatrixDevice input(3, 3, offset_, input_matrices_ + tid);

	double a00 = input(0, 0);
	double a01 = input(0, 1);
	double a02 = input(0, 2);
	double a11 = input(1, 1);
	double a12 = input(1, 2);
	double a22 = input(2, 2);

	double max0 = (fabs(a00) > fabs(a01)) ? fabs(a00) : fabs(a01);
	double max1 = (fabs(a02) > fabs(a11)) ? fabs(a02) : fabs(a11);
	double max2 = (fabs(a12) > fabs(a22)) ? fabs(a12) : fabs(a22);

	double maxAbsElement = (max0 > max1) ? max0 : max1;

	maxAbsElement = (maxAbsElement > max2) ? maxAbsElement : max2;

	if (maxAbsElement == 0.0) {
		MatrixDevice evec(3, 3, offset_, eigenvectors_ + tid);

		evec(0, 0) = 1.0;
		evec(1, 1) = 1.0;
		evec(2, 2) = 1.0;

		norm_[tid] = 0.0;
		return;
	}

	double invMaxAbsElement = 1.0 / maxAbsElement;

	a00 *= invMaxAbsElement;
	a01 *= invMaxAbsElement;
	a02 *= invMaxAbsElement;
	a11 *= invMaxAbsElement;
	a12 *= invMaxAbsElement;
	a22 *= invMaxAbsElement;

	input(0, 0) = a00;
	input(0, 1) = a01;
	input(0, 2) = a02;
	input(1, 1) = a11;
	input(1, 2) = a12;
	input(2, 2) = a22;
	input(1, 0) = a01;
	input(2, 0) = a02;
	input(2, 1) = a12;

	norm_[tid] = a01 * a01 + a02 * a02 + a12 * a12;

	maxAbsElement_[tid] = maxAbsElement;
}

CUDAH void SymmetricEigensolver3x3::computeEigenvalues(int tid)
{
	MatrixDevice input(3, 3, offset_, input_matrices_ + tid);
	MatrixDevice eval(3, 1, offset_, eigenvalues_ + tid);

	double a00 = input(0, 0);
	double a01 = input(0, 1);
	double a02 = input(0, 2);
	double a11 = input(1, 1);
	double a12 = input(1, 2);
	double a22 = input(2, 2);

	double norm = norm_[tid];

	if (norm > 0.0) {
		double traceDiv3 = (a00 + a11 + a22) / 3.0;
		double b00 = a00 - traceDiv3;
		double b11 = a11 - traceDiv3;
		double b22 = a22 - traceDiv3;
		double denom = sqrt((b00 * b00 + b11 * b11 + b22 * b22 + norm * 2.0) / 6.0);
		double c00 = b11 * b22 - a12 * a12;
		double c01 = a01 * b22 - a12 * a02;
		double c02 = a01 * a12 - b11 * a02;
		double det = (b00 * c00 - a01 * c01 + a02 * c02) / (denom * denom * denom);
		double halfDet = det * 0.5;

		halfDet = (halfDet > -1.0) ? halfDet : -1.0;
		halfDet = (halfDet < 1.0) ? halfDet : 1.0;

		double angle = acos(halfDet) / 3.0;
		double beta2 = cos(angle) * 2.0;
		double beta0 = cos(angle + M_PI * 2.0 / 3.0) * 2.0;
		double beta1 = -(beta0 + beta2);

		eval(0) = traceDiv3 + denom * beta0;
		eval(1) = traceDiv3 + denom * beta1;
		eval(2) = traceDiv3 + denom * beta2;

		i02_[tid] = (halfDet >= 0) ? 2 : 0;
		i02_[tid + offset_] = (halfDet >= 0) ? 0 : 2;

	} else {
		eval(0) = a00;
		eval(1) = a11;
		eval(2) = a22;
	}
}

CUDAH void SymmetricEigensolver3x3::updateEigenvalues(int tid)
{
	double maxAbsElement = maxAbsElement_[tid];
	MatrixDevice eval(3, 1, offset_, eigenvalues_ + tid);

	eval(0) *= maxAbsElement;
	eval(1) *= maxAbsElement;
	eval(2) *= maxAbsElement;
}

CUDAH void SymmetricEigensolver3x3::computeEigenvector00(int tid)
{
	if (norm_[tid] > 0.0) {
		MatrixDevice input(3, 3, offset_, input_matrices_ + tid);
		MatrixDevice row_mat(3, 3, offset_, buffer_ + tid);
		double eval0 = eigenvalues_[tid + i02_[tid] * offset_];

		input.copy(row_mat);

		row_mat(0, 0) -= eval0;
		row_mat(1, 1) -= eval0;
		row_mat(2, 2) -= eval0;

		//row0 is r0xr1, row1 is r0xr2, row2 is r1xr2
		MatrixDevice rxr(3, 3, offset_, buffer_ + 3 * 3 * offset_ + tid);

		cross(row_mat.row(0), row_mat.row(1), rxr.row(0));
		cross(row_mat.row(0), row_mat.row(2), rxr.row(1));
		cross(row_mat.row(1), row_mat.row(2), rxr.row(2));

	} else {
		eigenvectors_[tid] = 1.0;
	}
}

CUDAH void SymmetricEigensolver3x3::computeEigenvector01(int tid)
{
	if (norm_[tid] > 0.0) {
		MatrixDevice evec0(3, 1, offset_ * 3, eigenvectors_ + tid + i02_[tid] * offset_);

		//row0 is r0xr1, row1 is r0xr2, row2 is r1xr2
		MatrixDevice rxr(3, 3, offset_, buffer_ + 3 * 3 * offset_ + tid);


		double d0 = rxr(0, 0) * rxr(0, 0) + rxr(0, 1) * rxr(0, 1) * rxr(0, 2) * rxr(0, 2);
		double d1 = rxr(1, 0) * rxr(1, 0) + rxr(1, 1) * rxr(1, 1) * rxr(1, 2) * rxr(1, 2);
		double d2 = rxr(2, 0) * rxr(2, 0) + rxr(2, 1) * rxr(2, 1) * rxr(2, 2) * rxr(2, 2);

		double dmax = (d0 > d1) ? d0 : d1;
		int imax = (d0 > d1) ? 0 : 1;

		dmax = (d2 > dmax) ? d2 : dmax;
		imax = (d2 > dmax) ? 2 : imax;

		divide(rxr.row(imax), sqrt(dmax), evec0);
	}
}

CUDAH void SymmetricEigensolver3x3::computeEigenvector10(int tid)
{
	if (norm_[tid] > 0.0) {
		MatrixDevice input(3, 3, offset_, input_matrices_ + tid);
		MatrixDevice evec0(3, 1, offset_ * 3, eigenvectors_ + tid + i02_[tid] * offset_);
		double eval1 = eigenvalues_[tid + offset_];

		MatrixDevice u(3, 1, offset_, buffer_ + tid);
		MatrixDevice v(3, 1, offset_, buffer_ + 3 * offset_ + tid);

		computeOrthogonalComplement(evec0, u, v);

		MatrixDevice au(3, 1, offset_, buffer_ + 6 * offset_ + tid);
		MatrixDevice av(3, 1, offset_, buffer_ + 9 * offset_ + tid);

		double t0, t1, t2;

		t0 = u(0);
		t1 = u(1);
		t2 = u(2);

		au(0) = (input(0, 0) - eval1) * t0 + input(0, 1) * t1 + input(0, 2) * t2;
		au(1) = input(0, 1) * t0 + (input(1, 1) - eval1) * t1 + input(1, 2) * t2;
		au(2) = input(0, 2) * t0 + input(1, 2) * t1 + (input(2, 2) - eval1) * t2;

		t0 = v(0);
		t1 = v(1);
		t2 = v(2);

		av(0) = (input(0, 0) - eval1) * t0 + input(0, 1) * t1 + input(0, 2) * t2;
		av(1) = input(0, 1) * t0 + (input(1, 1) - eval1) * t1 + input(1, 2) * t2;
		av(2) = input(0, 2) * t0 + input(1, 2) * t1 + (input(2, 2) - eval1) * t2;
	} else {
		eigenvectors_[tid + offset_ * 4] = 1.0;
	}
}

CUDAH void SymmetricEigensolver3x3::computeEigenvector11(int tid)
{
	if (norm_[tid] > 0.0) {
		MatrixDevice evec1(3, 1, offset_ * 3, eigenvectors_ + tid + offset_);

		MatrixDevice u(3, 1, offset_, buffer_ + tid);
		MatrixDevice v(3, 1, offset_, buffer_ + 3 * offset_ + tid);

		MatrixDevice au(3, 1, offset_, buffer_ + 6 * offset_ + tid);
		MatrixDevice av(3, 1, offset_, buffer_ + 9 * offset_ + tid);

		double m00 = u(0) * au(0) + u(1) * au(1) + u(2) * au(2);
		double m01 = u(0) * av(0) + u(1) * av(1) + u(2) * av(2);
		double m11 = v(0) * av(0) + v(1) * av(1) + v(2) * av(2);

		double abs_m00 = fabs(m00);
		double abs_m01 = fabs(m01);
		double abs_m11 = fabs(m11);

		if (abs_m00 > 0 || abs_m01 > 0 || abs_m11 > 0) {
			double u_mult = (abs_m00 >= abs_m11) ? m01 : m11;
			double v_mult = (abs_m00 >= abs_m11) ? m00 : m01;

			bool res = fabs(u_mult) >= fabs(v_mult);
			double *large = (res) ? &u_mult : &v_mult;
			double *small = (res) ? &v_mult : &u_mult;

			*small /= (*large);
			*large = 1.0 / sqrt(1.0 + (*small) * (*small));
			*small *= (*large);

			multiply(u, u_mult, u);
			multiply(v, v_mult, v);
			subtract(u, v, evec1);

		} else {
			u.copy(evec1);
		}
	}
}

CUDAH void SymmetricEigensolver3x3::multiply(MatrixDevice u, double mult, MatrixDevice output)
{
	output(0) = u(0) * mult;
	output(1) = u(1) * mult;
	output(2) = u(2) * mult;
}

CUDAH void SymmetricEigensolver3x3::subtract(MatrixDevice u, MatrixDevice v, MatrixDevice output)
{
	output(0) = u(0) - v(0);
	output(1) = u(1) - v(1);
	output(2) = u(2) - v(2);
}

CUDAH void SymmetricEigensolver3x3::divide(MatrixDevice u, double div, MatrixDevice output)
{
	output(0) = u(0) / div;
	output(1) = u(1) / div;
	output(2) = u(2) / div;
}

CUDAH double SymmetricEigensolver3x3::dot(MatrixDevice u, MatrixDevice v)
{
	return (u(0) * v(0) + u(1) * v(1) + u(2) * v(2));
}

CUDAH void SymmetricEigensolver3x3::cross(MatrixDevice u, MatrixDevice v, MatrixDevice out)
{
	out(0) = u(1) * v(2) - u(2) * v(1);
	out(1) = u(2) * v(0) - u(0) * v(2);
	out(2) = u(0) * v(1) - u(1) * v(0);
}

CUDAH void SymmetricEigensolver3x3::computeOrthogonalComplement(MatrixDevice w, MatrixDevice u, MatrixDevice v)
{
	bool c = (fabs(w(0)) > fabs(w(1)));

	double inv_length = (c) ? (1.0 / sqrt(w(0) * w(0) + w(2) * w(2))) : (1.0 / sqrt(w(1) * w(1) + w(2) * w(2)));

	u(0) = (c) ? -w(2) * inv_length : 0.0;
	u(1) = (c) ? 0.0 : w(2) * inv_length;
	u(2) = (c) ? w(0) * inv_length : -w(1) * inv_length;

	cross(w, u, v);
}

CUDAH void SymmetricEigensolver3x3::computeEigenvector2(int tid)
{
	if (norm_[tid] > 0.0) {
		MatrixDevice evec0(3, 1, offset_ * 3, eigenvectors_ + tid + i02_[tid] * offset_);
		MatrixDevice evec1(3, 1, offset_ * 3, eigenvectors_ + tid + offset_);
		MatrixDevice evec2(3, 1, offset_ * 3, eigenvectors_ + tid + i02_[tid + offset_] * offset_);

		cross(evec0, evec1, evec2);
	} else {
		eigenvectors_[tid + offset_ * 8] = 1.0;
	}
}
}

#endif
