#include <vector>

class MPCTrajectory {
public:
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> z;
  std::vector<double> yaw;
  std::vector<double> vx;
  std::vector<double> k;
  std::vector<double> relative_time;
  void push_back(const double &xp, const double &yp, const double &zp,
                 const double &yawp, const double &vxp, const double &kp,
                 const double &tp) {
    x.push_back(xp);
    y.push_back(yp);
    z.push_back(zp);
    yaw.push_back(yawp);
    vx.push_back(vxp);
    k.push_back(kp);
    relative_time.push_back(tp);
  };
  void clear() {
    x.clear();
    y.clear();
    z.clear();
    yaw.clear();
    vx.clear();
    k.clear();
    relative_time.clear();
  };
  uint size() {
    uint a = x.size();
    if (a == y.size() && a == z.size() && a == yaw.size() && a == vx.size() &&
        a == k.size() && a == relative_time.size()) {
      return a;
    } else {
      printf("trajectory size is inappropriate\n");
      return 0;
    }
  }
};
