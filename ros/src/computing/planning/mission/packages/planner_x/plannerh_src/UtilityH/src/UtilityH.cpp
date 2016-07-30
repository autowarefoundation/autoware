/*
 * UtilityH.cpp
 *
 *  Created on: May 14, 2016
 *      Author: hatem
 */

#include "UtilityH.h"
#include <math.h>
#include <iostream>
#include <sstream>
#include <string.h>

using namespace std;


namespace UtilityHNS
{


UtilityH::UtilityH()
{
}

 UtilityH::~UtilityH()
{
}

 int UtilityH::GetSign(double x)
 {
	 if(x < 0 )
		 return -1;
	 else return 0;
 }

 double UtilityH::FixNegativeAngle(const double& a)
{
   double angle = 0;
   if (a < -2*M_PI || a > 2*M_PI)
	{
	   angle = fmod(a, 2*M_PI);
	}
   else
	   angle = a;


   if(angle < 0)
   {
	   angle = 2*M_PI + angle;
   }

   return angle;
}

 double UtilityH::SplitPositiveAngle(const double& a)
{
	 double angle = a;

	if (a < -2*M_PI || a > 2*M_PI)
	{
		angle = fmod(a, 2*M_PI);
	}

	if (angle > M_PI)
	{
		angle -= 2*M_PI;
	}
	else if (angle < -M_PI)
	{
		angle += 2*M_PI;
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

double UtilityH::GetCircularAngle(const double& prevAngle, const double& currAngle)
{
	double delta = currAngle - prevAngle;

	if(delta < -M_PI)
		delta += M_PI*2.0;
	else if (delta > M_PI)
		delta -= M_PI*2.0;

	return prevAngle+delta;
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

PIDController::PIDController()
{
	kp = 0;
	ki = 0;
	kd = 0;
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
	//TODO Remember to add sampling time and multiply the time elapsed by the error
	//complex PID error calculation
	//TODO //De = ( e(i) + 3*e(i-1) - 3*e(i-2) - e(i-3) ) / 6

	double e = targetValue - currValue;
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

	accumErr += e;
	double edot= e - prevErr;

	double p_part = kp * e;
	double i_part = ki *  accumErr;
	double d_part = kd * edot;

	double res = p_part + i_part + d_part;

	if(bEnableLimit)
	{
		if(res > upper_limit)
			res = upper_limit;
		else if ( res < lower_limit)
			res = lower_limit;
	}

	prevErr = e;

	return res;
}

void PIDController::ResetD()
{
	bResetI = true;
}

void PIDController::ResetI()
{
	bResetD = true;
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
	delete [] A;
	delete [] d1;
	delete [] d2;
	delete [] w0;
	delete [] w1;
	delete [] w2;
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

		A  = new double[m];
		d1 = new double[m];
		d2 = new double[m];
		w0 = new double[m];
		w1 = new double[m];
		w2 = new double[m];

		for(int i=0; i < m ; i++)
		{
			A[i]  = 0;
			d1[i] = 0;
			d2[i] = 0;
			w0[i] = 0;
			w1[i] = 0;
			w2[i] = 0;
		}

		for(int i=0; i< m; ++i)
		{
		    b = sin(M_PI*(2.0*i+1.0)/(2.0*n))*su;
		    c = cos(M_PI*(2.0*i+1.0)/(2.0*n))*cu;
		    c = b*b + c*c;
		    s = a2*c + 2.0*a*b + 1.0;
		    A[i] = a2/(4.0*s); // 4.0
		    d1[i] = 2.0*(1-a2*c)/s;
		    d2[i] = -(a2*c - 2.0*a*b + 1.0)/s;
		}
	}
}

double LowpassFilter::getFilter(const double& value)
{
	double ep = 2.3/1.0; // used to normalize
	double x = value;
	for(int i=0; i<m; ++i)
	{
		w0[i] = d1[i]*w1[i] + d2[i]*w2[i] + x;
		x = A[i]*(w0[i] + 2.0*w1[i] + w2[i]);
		w2[i] = w1[i];
		w1[i] = w0[i];
	}
	return ep*x;
}


}
