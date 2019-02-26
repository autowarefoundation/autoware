#pragma once
#include <eigen3/Eigen/Core>

class VehicleModelInterface
{
  protected:
    int dim_x_;
    int dim_u_;
    int dim_y_;

  public:
    int getDimX();
    int getDimU();
    int getDimY();
    virtual void calculateDiscreteMatrix(Eigen::MatrixXd &Ad, Eigen::MatrixXd &Bd, Eigen::MatrixXd &Cd, Eigen::MatrixXd &Wd, double &dt) = 0;
    virtual void calculateReferenceInput(Eigen::MatrixXd &Uref) = 0;
};

int VehicleModelInterface::getDimX() { return dim_x_; };
int VehicleModelInterface::getDimU() { return dim_u_; };
int VehicleModelInterface::getDimY() { return dim_y_; };