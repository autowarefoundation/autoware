
/// \file UtilityH.h
/// \brief General Math and Control utility functions
/// \author Hatem Darweesh
/// \date May 14, 2016

#ifndef UTILITYH_H_
#define UTILITYH_H_

#include <assert.h>
#include <string>
#include <math.h>


namespace UtilityHNS
{

#define DEG2RAD M_PI / 180.
#define RAD2DEG 180. / M_PI
#define SIGN(x) (x > 0) ? 1 : ((x < 0) ? -1 : 0)
#define MIN(x,y) (x <= y ? x : y)
#define MAX(x,y) (x >= y ? x : y)


class UtilityH
{
public:
	UtilityH();
	virtual ~UtilityH(); 


	static double FixNegativeAngle(const double& a);
	static double SplitPositiveAngle(const double& a);
	static double InverseAngle(const double& a);
	static double AngleBetweenTwoAnglesPositive(const double& a1, const double& a2);
	static double GetCircularAngle(const double& prevContAngle, const double& prevAngle, const double& currAngle);

	//Time Functions
	static void GetTickCount(struct timespec& t);
	static std::string GetFilePrefixHourMinuteSeconds();
	static double GetTimeDiffNow(const struct timespec& old_t);
	static double GetTimeDiff(const struct timespec& old_t,const struct timespec& curr_t);
	static std::string GetDateTimeStr();
	static int tsCompare (struct  timespec  time1,   struct  timespec  time2, int micro_tolerance = 10);
	static int GetSign(double x);
	static std::string GetHomeDirectory();
	static double GetMomentumScaleFactor(const double& v);
	static timespec GetTimeSpec(const time_t& srcT);
	static time_t GetLongTime(const struct timespec& srcT);
};

class PIDController
{
public:
	PIDController();
	PIDController(const double& kp, const double& ki, const double& kd);
	void Init(const double& kp, const double& ki, const double& kd);
	void Setlimit(const double& upper,const double& lower);
	double getPID(const double& currValue, const double& targetValue);
	double getPID(const double& e);
	void ResetD();
	void ResetI();
	std::string ToString();
	std::string ToStringHeader();


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
	bool   bEnableLimit;
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

	LowpassFilter(const int& filterOrder, const double& sampleFreq, const double& cutOffFreq);
	void Init(const int& filterOrder, const double& sampleFreq, const double& cutOffFreq);
	double getFilter(const double& value);


private:
	int m;
	double sampleF;
	double cutOffF;
	double A  ;
	double d1 ;
	double d2 ;
	double w0 ;
	double w1 ;
	double w2 ;

};

}

#endif


