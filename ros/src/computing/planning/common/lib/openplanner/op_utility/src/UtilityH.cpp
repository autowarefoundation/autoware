/// \file UtilityH.cpp
/// \brief General Math and Control utility functions
/// \author Hatem Darweesh
/// \date May 14, 2016

#include "op_utility/UtilityH.h"
#include <iostream>
#include <sstream>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>


using namespace std;


namespace UtilityHNS
{


UtilityH::UtilityH()
{
}

 UtilityH::~UtilityH()
{
}

 std::string UtilityH::GetHomeDirectory()
 {
	struct passwd *pw = getpwuid(getuid());
	const char *homedir = pw->pw_dir;
	return string(homedir);
 }

 double UtilityH::GetMomentumScaleFactor(const double& v)
 {
 	if(v < 0.3)
 		return 0.6;
 	else if(v <6.4)
 		return 0.3;
 	else if(v < 20)
 	{
 		double m = 0.7/3.6;
 		return m*(v - 6.4) + 0.3;
 	}
 	else
 		return 0.9;
 }

 int UtilityH::GetSign(double x)
 {
	 if(x < 0 )
		 return -1;
	 else
		 return 1;
 }

 double UtilityH::FixNegativeAngle(const double& a)
{
   double angle = 0;
   if (a < -2.0*M_PI || a > 2.0*M_PI)
	{
	   angle = fmod(a, 2.0*M_PI);
	}
   else
	   angle = a;


   if(angle < 0)
   {
	   angle = 2.0*M_PI + angle;
   }

   return angle;
}

 double UtilityH::SplitPositiveAngle(const double& a)
{
	 double angle = a;

	if (a < -2.0*M_PI || a > 2.0*M_PI)
	{
		angle = fmod(a, 2.0*M_PI);
	}

	if (angle > M_PI)
	{
		angle -= 2.0*M_PI;
	}
	else if (angle < -M_PI)
	{
		angle += 2.0*M_PI;
	}

	return angle;
}

double UtilityH::InverseAngle(const double& a)
{

   double angle = 0;
   if(a <= M_PI)
		angle =  a + M_PI;
	else
		angle = a - M_PI;

   return angle;
}

double UtilityH::AngleBetweenTwoAnglesPositive(const double& a1, const double& a2)
{
   double diff = a1 - a2;
   if(diff < 0)
	   diff = a2 - a1;

   if(diff > M_PI)
	   diff = 2.0*M_PI - diff;

   return diff;
}

double UtilityH::GetCircularAngle(const double& prevContAngle, const double& prevAngle, const double& currAngle)
{

	double diff = currAngle - prevAngle;
	if(diff > M_PI)
		diff = diff - 2.0*M_PI;
	if(diff < -M_PI)
		diff = diff + 2.0*M_PI;

	double c_ang = 0;
	if(prevContAngle == 0 || fabs(diff) < M_PI_2)
		 c_ang = prevContAngle + diff;
	else
		c_ang = prevContAngle;

	return c_ang;
}

void UtilityH::GetTickCount(struct timespec& t)
{
	while(clock_gettime(0, & t) == -1);
}

double UtilityH::GetTimeDiff(const struct timespec& old_t,const struct timespec& curr_t)
{
	return (curr_t.tv_sec - old_t.tv_sec) + ((double)(curr_t.tv_nsec - old_t.tv_nsec)/ 1000000000.0);
}

double UtilityH::GetTimeDiffNow(const struct timespec& old_t)
{
	struct timespec curr_t;
	GetTickCount(curr_t);
	return (curr_t.tv_sec - old_t.tv_sec) + ((double)(curr_t.tv_nsec - old_t.tv_nsec)/ 1000000000.0);
}

string UtilityH::GetFilePrefixHourMinuteSeconds()
{
	struct timespec now_time;
	UtilityH::GetTickCount(now_time);
	tm *gmtm = localtime(&now_time.tv_sec);
	ostringstream str;

	str << "Y" << gmtm->tm_year;
	str << "-";
	str << "M" << gmtm->tm_mon;
	str << "-";
	str << "D" << gmtm->tm_mday;
	str << "-";
	str << "H" << gmtm->tm_hour;
	str << "-";
	str << "M" << gmtm->tm_min;
	str << "-";
	str << "S" << gmtm->tm_sec;

	return str.str();
}

string UtilityH::GetDateTimeStr()
{
	time_t now = time(0);
	char* dateStr = ctime(&now);
	string str(dateStr, strlen(dateStr)-1);
	int index = str.find(" ");
	while(index > 0)
	{
		str.replace(index,1, "_");
		index = str.find(" ");
	}

	index = str.find(":");
	while(index > 0)
	{
		str.replace(index,1, "-");
		index = str.find(":");
	}
	return str;
}

int  UtilityH::tsCompare (struct  timespec  time1,   struct  timespec  time2, int micro_tolerance)
{

    if (time1.tv_sec < time2.tv_sec)
        return (-1) ;				/* Less than. */
    else if (time1.tv_sec > time2.tv_sec)
        return (1) ;				/* Greater than. */

    long diff = time1.tv_nsec - time2.tv_nsec;
    if (diff < -micro_tolerance)
        return (-1) ;				/* Less than. */
    else if (diff > micro_tolerance)
        return (1) ;				/* Greater than. */
    else
        return (0) ;				/* Equal. */

}

timespec UtilityH::GetTimeSpec(const time_t& srcT)
{
	timespec dstT;
	dstT.tv_sec = srcT/1000000000;
	dstT.tv_nsec = srcT - (dstT.tv_sec*1000000000);
	return dstT;
}

time_t UtilityH::GetLongTime(const struct timespec& srcT)
{
	time_t dstT;
	dstT = srcT.tv_sec * 1000000000 + srcT.tv_nsec;
	return dstT;
}

PIDController::PIDController()
{
	kp = kp_v = 0;
	ki = ki_v = 0;
	kd = kd_v = 0;
	pid_lim = pid_v = 0;
	upper_limit = lower_limit = 0;
	bEnableLimit= false;
	accumErr = 0;
	prevErr = 0;
	bResetD = false;
	bResetI = false;
}

PIDController::PIDController(const double& kp, const double& ki, const double& kd)
{
	Init(kp, ki, kd);
	upper_limit = lower_limit = 0;
	bEnableLimit= false;
	accumErr = 0;
	prevErr  = 0;
	bResetD = false;
	bResetI = false;

}

void PIDController::Setlimit(const double& upper,const double& lower)
{
	upper_limit = upper;
	lower_limit = lower;
	bEnableLimit = true;
}

double PIDController::getPID(const double& currValue, const double& targetValue)
{
	double e = targetValue - currValue;
	return getPID(e);
}

double PIDController::getPID(const double& e)
{
	//TODO Remember to add sampling time and multiply the time elapsed by the error
	//complex PID error calculation
	//TODO //De = ( e(i) + 3*e(i-1) - 3*e(i-2) - e(i-3) ) / 6


	if(bResetI)
	{
		bResetI = false;
		accumErr = 0;
	}

	if(bResetD)
	{
		bResetD = false;
		prevErr = e;
	}

	if(pid_v < upper_limit && pid_v > lower_limit)
		accumErr += e;

	double edot= e - prevErr;

	kp_v = kp * e;
	ki_v = ki *  accumErr;
	kd_v = kd * edot;

	pid_v = kp_v + ki_v + kd_v;
	pid_lim = pid_v;
	if(bEnableLimit)
	{
		if(pid_v > upper_limit)
		{
			pid_lim = upper_limit;
		}
		else if ( pid_v < lower_limit)
		{
			pid_lim = lower_limit;
		}
	}

	prevErr = e;

	return pid_lim;
}

std::string PIDController::ToStringHeader()
{
	std::ostringstream str_out;
	str_out << "Time" << "," <<"KP" << "," << "KI" << "," << "KD" << "," << "KP_v" << "," << "KI_v" << "," << "KD_v"
			<< "," << "pid_v" << "," << "," << "pid_lim" << "," << "," << "prevErr" << "," << "accumErr" << "," ;
	return str_out.str();
}

std::string PIDController::ToString()
{
	std::ostringstream str_out;
	timespec t_stamp;
	UtilityH::GetTickCount(t_stamp);
	str_out << UtilityH::GetLongTime(t_stamp) << "," <<kp << "," << ki << "," << kd << "," << kp_v << "," << ki_v << "," << kd_v
				<< "," << pid_v << "," << "," << pid_lim << "," << "," << prevErr << "," << accumErr << "," ;

	return str_out.str();

}

void PIDController::ResetD()
{
	bResetD = true;
}

void PIDController::ResetI()
{
	bResetI = true;
}

void PIDController::Init(const double& kp, const double& ki, const double& kd)
{
	this->kp = kp;
	this->ki = ki;
	this->kd = kd;
}

LowpassFilter::LowpassFilter()
{
	A = 0;
	d1 = 0;
	d2 = 0;
	w0 = 0;
	w1 = 0;
	w2 = 0;

	m = 0;
	sampleF = 0;
	cutOffF = 0;
}

LowpassFilter::~LowpassFilter()
{
//	if(A)
//		delete A;
//	if(d1)
//		delete d1;
//	if(d2)
//		delete d2;
//	if(w0)
//		delete w0;
//	if(w1)
//		delete w1;
//	if(w2)
//		delete w2;
}

LowpassFilter::LowpassFilter(const int& filterOrder, const double& sampleFreq, const double& cutOffFreq)
{
	Init(filterOrder, sampleFreq, cutOffFreq);
}

void LowpassFilter::Init(const int& n, const double& sampleFreq, const double& cutOffFreq)
{
	if(!(n == 2 || n == 4 || n == 6 || n == 8))
	{
		cout << "Undefined LowpassFilter order ! " << endl;

		A = 0;
		d1 = 0;
		d2 = 0;
		w0 = 0;
		w1 = 0;
		w2 = 0;

		m = 0;
		sampleF = 0;
		cutOffF = 0;
	}
	else
	{
		m = n/2;
		sampleF = sampleFreq;
		cutOffF = cutOffFreq;
		double ep = 1;
		double s = sampleFreq;
		double f = cutOffFreq;
		double a = tan(M_PI*f/s);
		double a2 = a*a;
		double u = log((1.0+sqrt(1.0+ep*ep))/ep);
		double su = sinh(u/(double)n);
		double cu = cosh(u/(double)n);
		double b, c;

//		A  = new double[m];
//		d1 = new double[m];
//		d2 = new double[m];
//		w0 = new double[m];
//		w1 = new double[m];
//		w2 = new double[m];
//
//		for(int i=0; i < m ; i++)
//		{
//			A[i]  = 0;
//			d1[i] = 0;
//			d2[i] = 0;
//			w0[i] = 0;
//			w1[i] = 0;
//			w2[i] = 0;
//		}

		for(int i=0; i< m; ++i)
		{
		    b = sin(M_PI*(2.0*i+1.0)/(2.0*n))*su;
		    c = cos(M_PI*(2.0*i+1.0)/(2.0*n))*cu;
		    c = b*b + c*c;
		    s = a2*c + 2.0*a*b + 1.0;
		    A = a2/(4.0*s); // 4.0
		    d1 = 2.0*(1-a2*c)/s;
		    d2 = -(a2*c - 2.0*a*b + 1.0)/s;
		}
	}
}

double LowpassFilter::getFilter(const double& value)
{
	double ep = 2.3/1.0; // used to normalize
	double x = value;
	for(int i=0; i<m; ++i)
	{
		w0 = d1*w1 + d2*w2 + x;
		x = A*(w0 + 2.0*w1 + w2);
		w2 = w1;
		w1 = w0;
	}
	return ep*x;
}


}
