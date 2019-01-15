Scalar numberOfFinites() const
{
  if (SizeAtCompileTime==0 || (SizeAtCompileTime==Dynamic && size()==0)) return Scalar(0);
  return Scalar((derived().array() == derived().array()).count());
}

Scalar sumOfFinites() const
{
  if (SizeAtCompileTime==0 || (SizeAtCompileTime==Dynamic && size()==0)) return Scalar(0);
  return Scalar(this->redux(Eigen::internal::scalar_sum_of_finites_op<Scalar>()));
}

Scalar meanOfFinites() const
{
  return Scalar(this->redux(Eigen::internal::scalar_sum_of_finites_op<Scalar>())) / this->numberOfFinites();
}

Scalar minCoeffOfFinites() const
{
  return Scalar(this->redux(Eigen::internal::scalar_min_of_finites_op<Scalar>()));
}

Scalar maxCoeffOfFinites() const
{
  return Scalar(this->redux(Eigen::internal::scalar_max_of_finites_op<Scalar>()));
}
