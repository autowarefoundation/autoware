/*
 * RSPlanner.h
 *
 *  Created on: Aug 9, 2015
 *      Author: hatem
 */

#include "PlanningHelpers.h"
#ifndef RSPLANNERSA_H_
#define RSPLANNERSA_H_

namespace PlannerHNS
{

#define EPS1 1.0e-12
#define EPS2 1.0e-12
#define EPS3 1.0e-12
#define EPS4 1.0e-12
#define MYINFINITY 1000000

#define MPI 3.1415926536
#define MPIMUL2 6.2831853072
#define MPIDIV2 1.5707963268




class RSPlanner
{
public:
	double RADCURV ;
	double RADCURVMUL2 ;
	double RADCURVMUL4 ;
	double SQRADCURV ;
	double SQRADCURVMUL2 ;
	double PATHDENSITY;
	RSPlanner(double curvatureFactor = 15.6);
	virtual ~RSPlanner();

	double min_length_rs(const double& x1,const double& y1,const double& t1,const double& x2,const double& y2,const double& t2,int& numero,double& t,double& u,double& v);
	int constRS(int num,double t,double u,double v,double x1,double y1,double t1,double delta,std::vector<WayPoint>& path);

private:
	struct ConfigItem
	{
		double length;
		int num;
		double t,u,v;
	};


	double mod2pi(const double& a);
	double my_atan2(const double&  y, const double& x);
	double c_c_c(const double& x,const double& y,const double& phi,const double& rs,const double& rc,double& t,double& u,double& v);
	double c_cc(const double& x, const double& y,const double& phi,const double& rs,const double& rc,double& t,double& u,double& v);
	double csca(const double& x, const double& y,const double& phi,const double& rs,const double& rc,double& t,double& u,double& v);
	double cscb(const double& x, const double& y,const double& phi,const double& rs,const double& rc,double& t,double& u,double& v);
	double ccu_cuc(const double& x, const double& y,const double& phi,const double& rs,const double& rc,double& t,double& u,double& v);
	double c_cucu_c(const double& x, const double& y,const double& phi,const double& rs,const double& rc,double& t,double& u,double& v);
	double c_c2sca(const double& x, const double& y,const double& phi,const double& rs,const double& rc,double& t,double& u,double& v);
	double c_c2scb(const double& x, const double& y,const double& phi,const double& rs,const double& rc,double& t,double& u,double& v);
	double c_c2sc2_c(const double& x, const double& y,const double& phi,const double& rs,const double& rc,double& t,double& u,double& v);
	double cc_c(const double& x, const double& y,const double& phi,const double& rs,const double& rc,double& t,double& u,double& v);
	double csc2_ca(const double& x, const double& y,const double& phi,const double& rs,const double& rc,double& t,double& u,double& v);
	double csc2_cb(const double& x, const double& y,const double& phi,const double& rs,const double& rc,double& t,double& u,double& v);

	double reed_shepp(const double& x1,const double& y1,const double& t1,const double& x2,const double& y2,const double& t2,int& numero,double& tr,double& ur,double& vr);

	int fct_curve(const int& ty,const int& orientation,const double& val,double& x1,double& y1,double& t1,const double& delta,std::vector<WayPoint>& path,int n);



};

} /* namespace PlannerZNS */

#endif /* RSPLANNER_H_ */
