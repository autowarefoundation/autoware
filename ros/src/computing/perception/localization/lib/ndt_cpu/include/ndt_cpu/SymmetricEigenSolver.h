#ifndef SYMMETRIC_EIGEN_
#define SYMMETRIC_EIGEN_

#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

namespace cpu {

class SymmetricEigensolver3x3 {
public:
	SymmetricEigensolver3x3();

	SymmetricEigensolver3x3(const Eigen::Matrix3d input_matrix);

	void compute();

	Eigen::Vector3d eigenvalues();

	Eigen::Matrix3d eigenvectors();

private:

	void computeEigenvector0(double a00, double a01, double a02, double a11, double a12, double a22, int i0);

	void computeEigenvector1(double a00, double a01, double a02, double a11, double a12, double a22, int i0, int i1);

	void computeEigenvector2(int i0, int i1, int i2);

	void computeOrthogonalComplement(Eigen::Vector3d &w, Eigen::Vector3d &u, Eigen::Vector3d &v);

	Eigen::Vector3d cross(Eigen::Vector3d u, Eigen::Vector3d v);

	Eigen::Matrix3d input_;
	Eigen::Matrix3d evecs_;
	Eigen::Vector3d evals_;
};


SymmetricEigensolver3x3::SymmetricEigensolver3x3()
{
	input_.setZero();
	evecs_.setZero();
	evals_.setZero();
}

SymmetricEigensolver3x3::SymmetricEigensolver3x3(const Eigen::Matrix3d input_matrix)
{
	input_ = input_matrix;
	evecs_.setZero();
	evals_.setZero();
}

void SymmetricEigensolver3x3::compute()
{
	double a00 = input_(0, 0);
	double a01 = input_(0, 1);
	double a02 = input_(0, 2);
	double a11 = input_(1, 1);
	double a12 = input_(1, 2);
	double a22 = input_(2, 2);

	double max0 = (fabs(a00) > fabs(a01)) ? fabs(a00) : fabs(a01);
	double max1 = (fabs(a02) > fabs(a11)) ? fabs(a02) : fabs(a11);
	double max2 = (fabs(a12) > fabs(a22)) ? fabs(a12) : fabs(a22);

	double maxAbsElement = (max0 > max1) ? max0 : max1;

	maxAbsElement = (maxAbsElement > max2) ? maxAbsElement : max2;

	if (maxAbsElement == 0.0) {
		evecs_.setIdentity();
		evals_.setZero();

		return;
	}

	double invMaxAbsElement = 1.0 / maxAbsElement;

	a00 *= invMaxAbsElement;
	a01 *= invMaxAbsElement;
	a02 *= invMaxAbsElement;
	a11 *= invMaxAbsElement;
	a12 *= invMaxAbsElement;
	a22 *= invMaxAbsElement;

	double norm = a01 * a01 + a02 * a02 + a12 * a12;

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

		evals_(0) = traceDiv3 + denom * beta0;
		evals_(1) = traceDiv3 + denom * beta1;
		evals_(2) = traceDiv3 + denom * beta2;

		int i0, i2, i1 = 1;

		if (halfDet >= 0.0) {
			i0 = 2;
			i2 = 0;
		} else {
			i0 = 0;
			i2 = 2;
		}

		computeEigenvector0(a00, a01, a02, a11, a12, a22, i0);
		computeEigenvector1(a00, a01, a02, a11, a12, a22, i0, i1);
		computeEigenvector2(i0, i1, i2);

	} else {
		evals_(0) = a00;
		evals_(1) = a11;
		evals_(2) = a22;
		evecs_.setIdentity();
	}

	evals_ *= maxAbsElement;
}

Eigen::Vector3d SymmetricEigensolver3x3::eigenvalues()
{
	return evals_;
}

Eigen::Matrix3d SymmetricEigensolver3x3::eigenvectors()
{
	return evecs_;
}


void SymmetricEigensolver3x3::computeEigenvector0(double a00, double a01, double a02, double a11, double a12, double a22, int i0)
{
	Eigen::Matrix3d row_mat;
	double eval0 = evals_(i0);

	row_mat(0, 0) = a00 - eval0;
	row_mat(0, 1) = a01;
	row_mat(0, 2) = a02;
	row_mat(1, 0) = a01;
	row_mat(1, 1) = a11 - eval0;
	row_mat(1, 2) = a12;
	row_mat(2, 0) = a02;
	row_mat(2, 1) = a12;
	row_mat(2, 2) = a22 - eval0;


	//row0 is r0xr1, row1 is r0xr2, row2 is r1xr2
	Eigen::Matrix3d rxr;

	rxr.row(0) = cross(row_mat.row(0), row_mat.row(1));
	rxr.row(1) = cross(row_mat.row(0), row_mat.row(2));
	rxr.row(2) = cross(row_mat.row(1), row_mat.row(2));

	double d0 = rxr(0, 0) * rxr(0, 0) + rxr(0, 1) * rxr(0, 1) * rxr(0, 2) * rxr(0, 2);
	double d1 = rxr(1, 0) * rxr(1, 0) + rxr(1, 1) * rxr(1, 1) * rxr(1, 2) * rxr(1, 2);
	double d2 = rxr(2, 0) * rxr(2, 0) + rxr(2, 1) * rxr(2, 1) * rxr(2, 2) * rxr(2, 2);

	double dmax = (d0 > d1) ? d0 : d1;
	int imax = (d0 > d1) ? 0 : 1;

	dmax = (d2 > dmax) ? d2 : dmax;
	imax = (d2 > dmax) ? 2 : imax;

	evecs_.col(i0) = rxr.row(imax) / sqrt(dmax);
}

void SymmetricEigensolver3x3::computeEigenvector1(double a00, double a01, double a02, double a11, double a12, double a22, int i0, int i1)
{
	Eigen::Vector3d u, v;
	Eigen::Vector3d evec0 = evecs_.col(i0);


	computeOrthogonalComplement(evec0, u, v);

	Eigen::Vector3d au, av;
	double t0, t1, t2;
	double eval1 = evals_(i1);

	t0 = u(0);
	t1 = u(1);
	t2 = u(2);

	au(0) = (a00 - eval1) * t0 + a01 * t1 + a02 * t2;
	au(1) = a01 * t0 + (a11 - eval1) * t1 + a12 * t2;
	au(2) = a02 * t0 + a12 * t1 + (a22 - eval1) * t2;

	t0 = v(0);
	t1 = v(1);
	t2 = v(2);

	av(0) = (a00 - eval1) * t0 + a01 * t1 + a02 * t2;
	av(1) = a01 * t0 + (a11 - eval1) * t1 + a12 * t2;
	av(2) = a02 * t0 + a12 * t1 + (a22 - eval1) * t2;

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

		u *= u_mult;
		v *= v_mult;
		evecs_.col(i1) = u - v;
	} else {
		evecs_.col(i1) = u;
	}
}


Eigen::Vector3d SymmetricEigensolver3x3::cross(Eigen::Vector3d u, Eigen::Vector3d v)
{
	Eigen::Vector3d out;

	out(0) = u(1) * v(2) - u(2) * v(1);
	out(1) = u(2) * v(0) - u(0) * v(2);
	out(2) = u(0) * v(1) - u(1) * v(0);

	return out;
}

void SymmetricEigensolver3x3::computeOrthogonalComplement(Eigen::Vector3d &w, Eigen::Vector3d &u, Eigen::Vector3d &v)
{
	bool c = (fabs(w(0)) > fabs(w(1)));

	double inv_length = (c) ? (1.0 / sqrt(w(0) * w(0) + w(2) * w(2))) : (1.0 / sqrt(w(1) * w(1) + w(2) * w(2)));

	u(0) = (c) ? -w(2) * inv_length : 0.0;
	u(1) = (c) ? 0.0 : w(2) * inv_length;
	u(2) = (c) ? w(0) * inv_length : -w(1) * inv_length;

	v = cross(w, u);
}

void SymmetricEigensolver3x3::computeEigenvector2(int i0, int i1, int i2)
{
	Eigen::Vector3d evec0 = evecs_.col(i0);
	Eigen::Vector3d evec1 = evecs_.col(i1);

	evecs_.col(i2) = cross(evec0, evec1);

}
}

#endif
