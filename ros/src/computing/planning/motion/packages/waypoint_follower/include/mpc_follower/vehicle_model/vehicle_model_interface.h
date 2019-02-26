#pragma once
#include <eigen3/Eigen/Core>

class VehicleModelInterface
{
  protected:
    const int dim_x_;
    const int dim_u_;
    const int dim_y_;

  public:
    VehicleModelInterface(int dim_x, int dim_u, int dim_y);
    int getDimX();
    int getDimU();
    int getDimY();
    virtual void calculateDiscreteMatrix(Eigen::MatrixXd &Ad, Eigen::MatrixXd &Bd, Eigen::MatrixXd &Cd, Eigen::MatrixXd &Wd, double &dt) = 0;
    virtual void calculateReferenceInput(Eigen::MatrixXd &Uref) = 0;
};
