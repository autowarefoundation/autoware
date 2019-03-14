#pragma once
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include "mpc_follower/vehicle_model/vehicle_model_interface.h"

/* 
   The trajectory following error dynamics of the vehicle.
   This is valid in the vicinity of the target trajectory.
*/

class KinematicsBicycleModel : public VehicleModelInterface
{
  public:
    KinematicsBicycleModel();
    ~KinematicsBicycleModel();

    void calculateDiscreteMatrix(Eigen::MatrixXd &Ad, Eigen::MatrixXd &Bd, Eigen::MatrixXd &Cd, Eigen::MatrixXd &Wd, double &dt) override;
    void calculateReferenceInput(Eigen::MatrixXd &Uref) override;
    
    void calculateReferenceInput(Eigen::MatrixXd &Uref, const double &curvature);
    void setParams(double &wheelbase, double &steer_tau, double &steer_lim_deg);
    void setVel(const double &vel);
    void setCurvature(const double &curvature);

  private:
    double wheelbase_;
    double steer_tau_;
    double steer_lim_deg_;

    double vel_;
    double curvature_;
};
