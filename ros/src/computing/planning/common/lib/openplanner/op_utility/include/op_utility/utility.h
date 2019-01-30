/// \file UtilityH.h
/// \brief General Math and Control utility functions
/// \author Hatem Darweesh
/// \date May 14, 2016

#ifndef UTILITYH_H_
#define UTILITYH_H_

#include <assert.h>
#include <string>
#include <math.h>

namespace op_utility_ns
{

#define DEG2RAD M_PI / 180.
#define RAD2DEG 180. / M_PI
#define SIGN(x) (x > 0) ? 1 : ((x < 0) ? -1 : 0)
#define MIN(x, y) (x <= y ? x : y)
#define MAX(x, y) (x >= y ? x : y)

class UtilityH
{
public:
  UtilityH();
  virtual ~UtilityH();

  static double fixNegativeAngle(const double & a);
  static double splitPositiveAngle(const double & a);
  static double inverseAngle(const double & a);
  static double angleBetweenTwoAnglesPositive(
    const double & a1,
    const double & a2);
  static double getCircularAngle(
    const double & prevContAngle,
    const double & prevAngle, const double & currAngle);

  //Time Functions
  static void getTickCount(struct timespec & t);
  static std::string getFilePrefixHourMinuteSeconds();
  static double getTimeDiffNow(const struct timespec & old_t);
  static double getTimeDiff(
    const struct timespec & old_t,
    const struct timespec & curr_t);
  static std::string getDateTimeStr();
  static int tsCompare(
    struct timespec time1, struct timespec time2,
    int micro_tolerance = 10);
  static int getSign(double x);
  static std::string getHomeDirectory();
  static double getMomentumScaleFactor(const double & v);
  static timespec getTimeSpec(const time_t & srcT);
  static time_t getLongTime(const struct timespec & srcT);
};

class PIDController
{
public:
  PIDController();
  PIDController(const double & kp, const double & ki, const double & kd);
  void init(const double & kp, const double & ki, const double & kd);
  void setLimit(const double & upper, const double & lower);
  double getPID(const double & currValue, const double & targetValue);
  double getPID(const double & e);
  void resetD();
  void resetI();
  std::string toString();
  std::string toStringHeader();

private:
  double kp;
  double ki;
  double kd;
  double kp_v;
  double ki_v;
  double kd_v;
  double pid_v;
  double pid_lim;
  double upper_limit;
  double lower_limit;
  bool bEnableLimit;
  double accumErr;
  double prevErr;
  bool bResetD;
  bool bResetI;

};

class LowpassFilter
{
public:
  LowpassFilter();
  virtual ~LowpassFilter();

  LowpassFilter(
    const int & filterOrder, const double & sampleFreq,
    const double & cutOffFreq);
  void init(
    const int & filterOrder, const double & sampleFreq,
    const double & cutOffFreq);
  double getFilter(const double & value);

private:
  int m;
  double sampleF;
  double cutOffF;
  double A;
  double d1;
  double d2;
  double w0;
  double w1;
  double w2;

};

}

#endif
