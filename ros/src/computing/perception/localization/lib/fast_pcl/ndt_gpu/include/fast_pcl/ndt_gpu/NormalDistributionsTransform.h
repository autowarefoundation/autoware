#ifndef GPU_NDT_H_
#define GPU_NDT_H_

#include <cuda.h>
#include <cuda_runtime.h>
#include "Registration.h"
#include "common.h"
#include "VoxelGrid.h"
#include "Eigen/Geometry"

namespace gpu {
class GNormalDistributionsTransform: public GRegistration {
public:
	GNormalDistributionsTransform();

	GNormalDistributionsTransform(const GNormalDistributionsTransform &other)
	{
		gauss_d1_ = other.gauss_d1_;
		gauss_d2_ = other.gauss_d2_;

		outlier_ratio_ = other.outlier_ratio_;

		j_ang_a_ = other.j_ang_a_;
		j_ang_b_ = other.j_ang_b_;
		j_ang_c_ = other.j_ang_c_;
		j_ang_d_ = other.j_ang_d_;
		j_ang_e_ = other.j_ang_e_;
		j_ang_f_ = other.j_ang_f_;
		j_ang_g_ = other.j_ang_g_;
		j_ang_h_ = other.j_ang_h_;

		h_ang_a2_ = other.h_ang_a2_;
		h_ang_a3_ = other.h_ang_a3_;
		h_ang_b2_ = other.h_ang_b2_;
		h_ang_b3_ = other.h_ang_b3_;
		h_ang_c2_ = other.h_ang_c2_;
		h_ang_c3_ = other.h_ang_c3_;
		h_ang_d1_ = other.h_ang_d2_;
		h_ang_d2_ = other.h_ang_d2_;
		h_ang_d3_ = other.h_ang_d3_;
		h_ang_e1_ = other.h_ang_e1_;
		h_ang_e2_ = other.h_ang_e2_;
		h_ang_f1_ = other.h_ang_f1_;
		h_ang_f2_ = other.h_ang_f2_;
		h_ang_f3_ = other.h_ang_f3_;

		dj_ang_a_ = other.dj_ang_a_;
		dj_ang_b_ = other.dj_ang_b_;
		dj_ang_c_ = other.dj_ang_c_;
		dj_ang_e_ = other.dj_ang_e_;
		dj_ang_f_ = other.dj_ang_f_;
		dj_ang_g_ = other.dj_ang_g_;
		dj_ang_h_ = other.dj_ang_h_;

		dh_ang_a2_ = other.dh_ang_a2_;
		dh_ang_a3_ = other.dh_ang_a3_;
		dh_ang_b2_ = other.dh_ang_b2_;
		dh_ang_b3_ = other.dh_ang_b3_;
		dh_ang_c2_ = other.dh_ang_c2_;
		dh_ang_c3_ = other.dh_ang_c3_;
		dh_ang_d1_ = other.dh_ang_d2_;
		dh_ang_d2_ = other.dh_ang_d2_;
		dh_ang_d3_ = other.dh_ang_d3_;
		dh_ang_e1_ = other.dh_ang_e1_;
		dh_ang_e2_ = other.dh_ang_e2_;
		dh_ang_f1_ = other.dh_ang_f1_;
		dh_ang_f2_ = other.dh_ang_f2_;
		dh_ang_f3_ = other.dh_ang_f3_;

		step_size_ = other.step_size_;
		resolution_ = other.resolution_;
		trans_probability_ = other.trans_probability_;
		real_iterations_ = other.real_iterations_;

		voxel_grid_ = other.voxel_grid_;
	}

	inline void setStepSize(double step_size)
	{
		step_size_ = step_size;
	}

	inline void setResolution(float resolution)
	{
		resolution_ = resolution;
	}

	inline void setOutlierRatio(double olr)
	{
		outlier_ratio_ = olr;
	}

	inline double getStepSize()
	{
		return step_size_;
	}

	inline float getResolution()
	{
		return resolution_;
	}

	inline double getOutlierRatio()
	{
		return outlier_ratio_;
	}

	inline double getTransformationProbability()
	{
		return trans_probability_;
	}

	/* Set the input map points */
	void setInputTarget(pcl::PointCloud<pcl::PointXYZI>::Ptr input);
	void setInputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr input);

	/* Compute and get fitness score */
	double getFitnessScore(double max_range = DBL_MAX);

	inline int getRealIterations() { return real_iterations_; }

	~GNormalDistributionsTransform();
protected:
	void computeTransformation(Eigen::Matrix<float, 4, 4> &guess);
	double computeDerivatives(Eigen::Matrix<double, 6, 1> &score_gradient, Eigen::Matrix<double, 6, 6> &hessian,
								float *trans_x, float *trans_y, float *trans_z,
								int points_num, Eigen::Matrix<double, 6, 1> pose, bool compute_hessian = true);

private:
	//Copied from ndt.h
    inline double auxilaryFunction_PsiMT (double a, double f_a, double f_0, double g_0, double mu = 1.e-4)
    {
      return (f_a - f_0 - mu * g_0 * a);
    }

    //Copied from ndt.h
    inline double auxilaryFunction_dPsiMT (double g_a, double g_0, double mu = 1.e-4)
    {
      return (g_a - mu * g_0);
    }

    double updateIntervalMT (double &a_l, double &f_l, double &g_l,
								double &a_u, double &f_u, double &g_u,
								double a_t, double f_t, double g_t);

    double trialValueSelectionMT (double a_l, double f_l, double g_l,
									double a_u, double f_u, double g_u,
									double a_t, double f_t, double g_t);

	void transformPointCloud(float *in_x, float *in_y, float *in_z,
								float *out_x, float *out_y, float *out_z,
								int points_number, Eigen::Matrix<float, 4, 4> transform);

	void computeAngleDerivatives(MatrixHost pose, bool compute_hessian = true);

	double computeStepLengthMT(const Eigen::Matrix<double, 6, 1> &x, Eigen::Matrix<double, 6, 1> &step_dir,
								double step_init, double step_max, double step_min, double &score,
								Eigen::Matrix<double, 6, 1> &score_gradient, Eigen::Matrix<double, 6, 6> &hessian,
								float *out_x, float *out_y, float *out_z, int points_num);

	void computeHessian(Eigen::Matrix<double, 6, 6> &hessian, float *trans_x, float *trans_y, float *trans_z, int points_num, Eigen::Matrix<double, 6, 1> &p);


	double gauss_d1_, gauss_d2_;
	double outlier_ratio_;
	MatrixHost j_ang_a_, j_ang_b_, j_ang_c_, j_ang_d_, j_ang_e_, j_ang_f_, j_ang_g_, j_ang_h_;

	MatrixHost h_ang_a2_, h_ang_a3_, h_ang_b2_, h_ang_b3_, h_ang_c2_, h_ang_c3_, h_ang_d1_, h_ang_d2_, h_ang_d3_,
				h_ang_e1_, h_ang_e2_, h_ang_e3_, h_ang_f1_, h_ang_f2_, h_ang_f3_;


	MatrixDevice dj_ang_a_, dj_ang_b_, dj_ang_c_, dj_ang_d_, dj_ang_e_, dj_ang_f_, dj_ang_g_, dj_ang_h_;

	MatrixDevice dh_ang_a2_, dh_ang_a3_, dh_ang_b2_, dh_ang_b3_, dh_ang_c2_, dh_ang_c3_, dh_ang_d1_, dh_ang_d2_, dh_ang_d3_,
				dh_ang_e1_, dh_ang_e2_, dh_ang_e3_, dh_ang_f1_, dh_ang_f2_, dh_ang_f3_;

	double step_size_;
	float resolution_;
	double trans_probability_;

	int real_iterations_;


	GVoxelGrid voxel_grid_;
};
}

#endif
