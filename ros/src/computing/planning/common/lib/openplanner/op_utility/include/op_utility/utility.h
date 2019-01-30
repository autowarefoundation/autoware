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

  /// Make sure angle is in [0, 2 * pi)
  static double fixNegativeAngle(const double & a);

  /// Make sure angle is in [-pi, pi)
  static double splitPositiveAngle(const double & a);

  /** Rotate angle 180 degrees
   *
   * @param a Angle in [0, 2 * pi)
   * @return Reversed angle in [0, 2 * pi)
   */
  static double inverseAngle(const double & a);

  /// Angular difference in [0, pi]
  static double angleBetweenTwoAnglesPositive(
    const double & a1,
    const double & a2);

  // TODO: figure out and document what this does and test
  static double getCircularAngle(
    const double & prevContAngle,
    const double & prevAngle, const double & currAngle);

  /// Return -1 for negative, 1 for non-negative values
  static int getSign(double x);

  //Time Functions
  /** Gets time since the Epoch, using the system-wide realtime clock
   *
   * NB Spins (possibly indefinitely) when getting the time fails.
   */
  static void getTickCount(struct timespec & t);

  /** Turn a time_t representation of time into a timespec
   *
   * NB provided time is expected to be _nanoseconds_, rather than in seconds
   * (so unlike what the standard function `time` returns)
   */
  static timespec getTimeSpec(const time_t & srcT);

  /** Turn a timespec representation of time into a time_t
   *
   * NB return time is in _nanoseconds_, rather than in seconds
   * (so unlike what the standard function `time` returns)
   */
  static time_t getLongTime(const struct timespec & srcT);

  /// Determine time difference between provided past time and now
  static double getTimeDiffNow(const struct timespec & old_t);

  /// Determine time difference between provided times
  static double getTimeDiff(
    const struct timespec & old_t,
    const struct timespec & curr_t);

  /** Check the order of two timestamps
   *
   * @param time1 Time A
   * @param time2 Time B
   * @param micro_tolerance If absolute difference is less or equal than this amount of nanoseconds, ignore and return 0
   * @return -1 if time 1 is before time2, 0 if they are the same, 1 otherwise
   */
  static int tsCompare(
    struct timespec time1, struct timespec time2,
    int micro_tolerance = 10);

  /// Returns string with format 'Y<YEAR>-M<MONTH>-D<DAY>-H<HOUR>-M<MINUTE>-S<SECOND>', based on current time
  static std::string getFilePrefixHourMinuteSeconds();

  /// Returns string with format like 'Wed_Jun_30_21-49-08_1993'
  static std::string getDateTimeStr();

  // Get path to the current user's home directory
  static std::string getHomeDirectory();

  /// TODO: figure out and document what this does and test
  static double getMomentumScaleFactor(const double & v);
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
