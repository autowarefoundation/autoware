#ifndef CPU_REG_H_
#define CPU_REG_H_

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace cpu {

template <typename PointSourceType, typename PointTargetType>
class Registration {
public:
	Registration();

	void align(const Eigen::Matrix<float, 4, 4> &guess);

	void align(typename pcl::PointCloud<PointSourceType> &output, const Eigen::Matrix<float, 4, 4> &guess);

	void setTransformationEpsilon(double trans_eps);

	double getTransformationEpsilon() const;

	void setMaximumIterations(int max_itr);

	int getMaximumIterations() const;

	Eigen::Matrix<float, 4, 4> getFinalTransformation() const;

	/* Set input Scanned point cloud.
	 * Simply set the point cloud input_ */
	void setInputSource(typename pcl::PointCloud<PointSourceType>::Ptr input);

	/* Set input reference map point cloud.
	 * Set the target point cloud ptr target_cloud_ */
	void setInputTarget(typename pcl::PointCloud<PointTargetType>::Ptr input);

	int getFinalNumIteration() const;

	bool hasConverged() const;

	void updateInputTarget(typename pcl::PointCloud<PointTargetType>::Ptr new_cloud);

	virtual ~Registration();

protected:

	virtual void computeTransformation(const Eigen::Matrix<float, 4, 4> &guess);

	double transformation_epsilon_;
	int max_iterations_;

	//Original scanned point clouds
	typename pcl::PointCloud<PointSourceType>::Ptr source_cloud_;

	//Transformed point clouds
	typename pcl::PointCloud<PointSourceType> trans_cloud_;

	bool converged_;
	int nr_iterations_;

	Eigen::Matrix<float, 4, 4> final_transformation_, transformation_, previous_transformation_;

	bool target_cloud_updated_;

	// Reference map point
	typename pcl::PointCloud<PointTargetType>::Ptr target_cloud_;
};
}

#endif
