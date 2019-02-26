#include "mpc_follower/vehicle_model/vehicle_model_interface.h"

VehicleModelInterface::VehicleModelInterface(int dim_x, int dim_u, int dim_y) : dim_x_(dim_x), dim_u_(dim_u), dim_y_(dim_y) {};
int VehicleModelInterface::getDimX() { return dim_x_; };
int VehicleModelInterface::getDimU() { return dim_u_; };
int VehicleModelInterface::getDimY() { return dim_y_; };
