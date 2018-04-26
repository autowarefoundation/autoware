#include "ndt_cpu/Registration.h"
#include "ndt_cpu/debug.h"
#include <iostream>

namespace cpu {

template <typename PointSourceType, typename PointTargetType>
Registration<PointSourceType, PointTargetType>::Registration()
{
	max_iterations_ = 0;

	converged_ = false;
	nr_iterations_ = 0;

	transformation_epsilon_ = 0;
	target_cloud_updated_ = true;

	trans_cloud_.points.clear();
}

template <typename PointSourceType, typename PointTargetType>
Registration<PointSourceType, PointTargetType>::~Registration()
{
	return;
}

template <typename PointSourceType, typename PointTargetType>
void Registration<PointSourceType, PointTargetType>::setTransformationEpsilon(double trans_eps)
{
	transformation_epsilon_ = trans_eps;
}

template <typename PointSourceType, typename PointTargetType>
double Registration<PointSourceType, PointTargetType>::getTransformationEpsilon() const
{
	return transformation_epsilon_;
}

template <typename PointSourceType, typename PointTargetType>
void Registration<PointSourceType, PointTargetType>::setMaximumIterations(int max_itr)
{
	max_iterations_ = max_itr;
}

template <typename PointSourceType, typename PointTargetType>
int Registration<PointSourceType, PointTargetType>::getMaximumIterations() const
{
	return max_iterations_;
}

template <typename PointSourceType, typename PointTargetType>
Eigen::Matrix<float, 4, 4> Registration<PointSourceType, PointTargetType>::getFinalTransformation() const
{
	return final_transformation_;
}

template <typename PointSourceType, typename PointTargetType>
int Registration<PointSourceType, PointTargetType>::getFinalNumIteration() const
{
	return nr_iterations_;
}

template <typename PointSourceType, typename PointTargetType>
bool Registration<PointSourceType, PointTargetType>::hasConverged() const
{
	return converged_;
}

template <typename PointSourceType, typename PointTargetType>
void Registration<PointSourceType, PointTargetType>::setInputSource(typename pcl::PointCloud<PointSourceType>::Ptr input)
{
	source_cloud_ = input;
}


//Set input MAP data
template <typename PointSourceType, typename PointTargetType>
void Registration<PointSourceType, PointTargetType>::setInputTarget(typename pcl::PointCloud<PointTargetType>::Ptr input)
{
	target_cloud_ = input;
}

template <typename PointSourceType, typename PointTargetType>
void Registration<PointSourceType, PointTargetType>::align(const Eigen::Matrix<float, 4, 4> &guess)
{
	converged_ = false;

	final_transformation_ = transformation_ = previous_transformation_ = Eigen::Matrix<float, 4, 4>::Identity();

	trans_cloud_.points.resize(source_cloud_->points.size());

	for (int i = 0; i < trans_cloud_.points.size(); i++) {
		trans_cloud_.points[i] = source_cloud_->points[i];
	}

	computeTransformation(guess);
}

template <typename PointSourceType, typename PointTargetType>
void Registration<PointSourceType, PointTargetType>::align(typename pcl::PointCloud<PointSourceType> &output, const Eigen::Matrix<float, 4, 4> &guess)
{
	align(guess);
}

template <typename PointSourceType, typename PointTargetType>
void Registration<PointSourceType, PointTargetType>::computeTransformation(const Eigen::Matrix<float, 4, 4> &guess) {
	printf("Unsupported by Registration\n");
}

template class Registration<pcl::PointXYZI, pcl::PointXYZI>;
template class Registration<pcl::PointXYZ, pcl::PointXYZ>;
}
