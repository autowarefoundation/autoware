#include <eigen3/Eigen/Core>

/* 
   Following error dynamics of the vehicle and the trajectory 
   in the vicinity of the target trajectory.
*/

class KinematicsBicycleModel
{
  public:
    KinematicsBicycleModel();
    ~KinematicsBicycleModel();

    int getDimX();
    int getDimU();
    int getDimY();
    void calculateDiscreteMatrix(Eigen::MatrixXd &Ad, Eigen::MatrixXd &Bd,
                                 Eigen::MatrixXd &Cd, Eigen::MatrixXd &Wd, double &dt);
    void calculateReferenceInput(Eigen::MatrixXd &Uref);
    void setVel(double &vel);
    void setCurvature(double &curvature);

  private:
    int dim_x_;
    int dim_u_;
    int dim_y_;

    double wheelbase_;
    double steer_tau_;
    double steer_lim_deg_;

    double vel_;
    double curvature_;
};

KinematicsBicycleModel::KinematicsBicycleModel() : dim_x_(3), dim_u_(1), dim_y_(2),
                                                   wheelbase_(2.79), steer_tau_(0.2), steer_lim_deg_(35.0){};
KinematicsBicycleModel::~KinematicsBicycleModel(){};

int KinematicsBicycleModel::getDimX() { return dim_x_; };
int KinematicsBicycleModel::getDimU() { return dim_u_; };
int KinematicsBicycleModel::getDimY() { return dim_y_; };

void KinematicsBicycleModel::calculateDiscreteMatrix(Eigen::MatrixXd &Ad, Eigen::MatrixXd &Bd,
                                                     Eigen::MatrixXd &Cd, Eigen::MatrixXd &Wd, double &dt)
{
    auto sign = [](double x) { return (x > 0.0) - (x < 0.0); };
    static const double DEG2RAD = M_PI / 180.0;

    /* Linearize delta around delta_r (referece delta) */
    double delta_r = atan(wheelbase_ * curvature_);
    if (abs(delta_r) >= steer_lim_deg_ * DEG2RAD)
        delta_r = (steer_lim_deg_ * DEG2RAD) * (double)sign(delta_r);
    double cos_delta_r_squared_inv = 1 / (cos(delta_r) * cos(delta_r));

    Ad << 0.0, vel_, 0.0,
        0.0, 0.0, vel_ / wheelbase_ * cos_delta_r_squared_inv,
        0.0, 0.0, -1.0 / steer_tau_;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dim_x_, dim_x_);
    Ad = (I - dt * 0.5 * Ad).inverse() * (I + dt * 0.5 * Ad); // bilinear discretization

    Bd << 0.0, 0.0, 1.0 / steer_tau_;
    Bd *= dt;

    Cd << 1.0, 0.0, 0.0,
        0.0, 1.0, 0.0;

    Wd << 0.0,
        -vel_ * curvature_ + vel_ / wheelbase_ * (tan(delta_r) - delta_r * cos_delta_r_squared_inv),
        0.0;
    Wd *= dt;
}

void KinematicsBicycleModel::calculateReferenceInput(Eigen::MatrixXd &Uref)
{
    Uref(0, 0) = std::atan(wheelbase_ * curvature_);
}

void KinematicsBicycleModel::setVel(double &vel) { vel_ = vel; };
void KinematicsBicycleModel::setCurvature(double &curvature) { curvature_ = curvature; };
