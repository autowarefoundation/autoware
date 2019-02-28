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

	GNormalDistributionsTransform(const GNormalDistributionsTransform &other);

	void setStepSize(double step_size);

	void setResolution(float resolution);

	void setOutlierRatio(double olr);

	double getStepSize() const;

	float getResolution() const;

	double getOutlierRatio() const;

	double getTransformationProbability() const;

	int getRealIterations();

	/* Set the input map points */
	void setInputTarget(pcl::PointCloud<pcl::PointXYZI>::Ptr input);
	void setInputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr input);

	/* Compute and get fitness score */
	double getFitnessScore(double max_range = DBL_MAX);

	~GNormalDistributionsTransform();

protected:
	void computeTransformation(const Eigen::Matrix<float, 4, 4> &guess);
	double computeDerivatives(Eigen::Matrix<double, 6, 1> &score_gradient, Eigen::Matrix<double, 6, 6> &hessian,
								float *trans_x, float *trans_y, float *trans_z,
								int points_num, Eigen::Matrix<double, 6, 1> pose, bool compute_hessian = true);

private:
	//Copied from ndt.h
    double auxilaryFunction_PsiMT (double a, double f_a, double f_0, double g_0, double mu = 1.e-4);

    //Copied from ndt.h
    double auxilaryFunction_dPsiMT (double g_a, double g_0, double mu = 1.e-4);

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
	//MatrixHost j_ang_a_, j_ang_b_, j_ang_c_, j_ang_d_, j_ang_e_, j_ang_f_, j_ang_g_, j_ang_h_;
	MatrixHost j_ang_;

	//MatrixHost h_ang_a2_, h_ang_a3_, h_ang_b2_, h_ang_b3_, h_ang_c2_, h_ang_c3_, h_ang_d1_, h_ang_d2_, h_ang_d3_,
	//			h_ang_e1_, h_ang_e2_, h_ang_e3_, h_ang_f1_, h_ang_f2_, h_ang_f3_;
	MatrixHost h_ang_;


	//MatrixDevice dj_ang_a_, dj_ang_b_, dj_ang_c_, dj_ang_d_, dj_ang_e_, dj_ang_f_, dj_ang_g_, dj_ang_h_;
	MatrixDevice dj_ang_;


	//MatrixDevice dh_ang_a2_, dh_ang_a3_, dh_ang_b2_, dh_ang_b3_, dh_ang_c2_, dh_ang_c3_, dh_ang_d1_, dh_ang_d2_, dh_ang_d3_,
	//			dh_ang_e1_, dh_ang_e2_, dh_ang_e3_, dh_ang_f1_, dh_ang_f2_, dh_ang_f3_;
	MatrixDevice dh_ang_;

	double step_size_;
	float resolution_;
	double trans_probability_;

	int real_iterations_;


	GVoxelGrid voxel_grid_;
};
}

#endif
