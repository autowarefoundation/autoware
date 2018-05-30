/*
 * RSPlannerSA.cpp
 *
 *  Created on: Aug 9, 2015
 *      Author: hatem
 */

#include "op_planner/RSPlanner.h"


using namespace std;

namespace PlannerHNS
{

RSPlanner::RSPlanner(double curvatureFactor)
{
	RADCURV = curvatureFactor;
	RADCURVMUL2  = 2. * RADCURV;
	RADCURVMUL4  = 4. * RADCURV;
	SQRADCURV = RADCURV*RADCURV;
	SQRADCURVMUL2 = 4. * RADCURV*RADCURV;
	PATHDENSITY = 0.1; // 10 cm

}

RSPlanner::~RSPlanner()
{
	// TODO Auto-generated destructor stub
}


double RSPlanner::mod2pi(const double& a)
	{
	double angle = a;
	   while (angle < 0.0) angle = angle + MPIMUL2;
	   while (angle >= MPIMUL2) angle = angle - MPIMUL2;
	   return angle;
	}

double RSPlanner::my_atan2(const double & y, const double& x)
{
//   double a;
//   if ((x == 0.0) && (y == 0.0)) return 0.0;
//   if (x == 0.0)
//   {
//      if (y > 0)
//    	  return MPIDIV2;
//      else
//    	  return -MPIDIV2;
//   }
//   a = atan(y/x);
//   if (a > 0.0)
//      if (x > 0) return a;
//            else return (a+MPI);
//   else
//      if (x > 0) return (a+MPIMUL2);
//            else return (a+MPI);
	return atan2(y,x);
}

double RSPlanner::c_c_c(const double& x,const double& y,const double& phi,const double& rs,const double& rc,double& t,double& u,double& v)
{
	double a,b,u1,theta,alpha,length_rs;

	   a = x-rs;
	   b = y+rc;
	   if ((fabs(a)<EPS3) && (fabs(b)<EPS3)) return(INFINITY);
	   u1 = sqrt(a*a+b*b);
	   if (u1>RADCURVMUL4) return(INFINITY);
	   theta = my_atan2(b,a);
	   alpha = acos(u1/RADCURVMUL4);
	   t = mod2pi(MPIDIV2 + alpha + theta);
	   u = mod2pi(MPI-2*alpha);
	   v = mod2pi(phi-t-u);

	   length_rs = RADCURV*(t+u+v);
	   return(length_rs);
}

double RSPlanner::c_cc(const double& x, const double& y,const double& phi,const double& rs,const double& rc,double& t,double& u,double& v)
{
   double a,b,u1,theta,alpha,length_rs;

   a = x-rs;
   b = y+rc;
   if ((fabs(a)<EPS3) && (fabs(b)<EPS3)) return(INFINITY);
   u1 = sqrt(a*a+b*b);
   if (u1>RADCURVMUL4) return(INFINITY);
   theta = my_atan2(b,a);
   alpha = acos(u1/RADCURVMUL4);
   t = mod2pi(MPIDIV2 + alpha + theta);
   u = mod2pi(MPI-2*alpha);
   v = mod2pi(t+u-phi);

   length_rs = RADCURV*(t+u+v);
   return(length_rs);
}

double RSPlanner::csca(const double& x, const double& y,const double& phi,const double& rs,const double& rc,double& t,double& u,double& v)
{
   double a,b,length_rs;

   a = x-rs;
   b = y+rc;
   t = mod2pi(my_atan2(b,a));
   u = sqrt(a*a+b*b);
   v = mod2pi(phi-t);

   length_rs = RADCURV*(t+v) + u;
   return(length_rs);
}

double RSPlanner::cscb(const double& x, const double& y,const double& phi,const double& rs,const double& rc,double& t,double& u,double& v)
{
   double a,b,u1,theta,alpha,length_rs;

   a = x+rs;
   b = y-rc;
   u1 = sqrt(a*a+b*b);
   if (u1 < RADCURVMUL2) return(INFINITY);
   theta = my_atan2(b,a);
   u = sqrt(u1*u1 - SQRADCURVMUL2);
   alpha = my_atan2(RADCURVMUL2,u);
   t = mod2pi(theta+alpha);
   v = mod2pi(t-phi);

   length_rs = RADCURV*(t+v) + u;
   return(length_rs);
}

double RSPlanner::ccu_cuc(const double& x, const double& y,const double& phi,const double& rs,const double& rc,double& t,double& u,double& v)
{
   double a,b,u1,theta,alpha,length_rs;

   a = x+rs;
   b = y-rc;
   if ((fabs(a)<EPS3) && (fabs(b)<EPS3)) return(INFINITY);
   u1 = sqrt(a*a+b*b);
   if (u1 > RADCURVMUL4) return(INFINITY);
   theta = my_atan2(b,a);
   if (u1>RADCURVMUL2)
     {
      alpha = acos((u1/2-RADCURV)/RADCURVMUL2);
      t = mod2pi(MPIDIV2+theta-alpha);
      u = mod2pi(MPI-alpha);
      v = mod2pi(phi-t+2*(u));
     }
   else
     {
      alpha = acos((u1/2+RADCURV)/(RADCURVMUL2));
      t = mod2pi(MPIDIV2+theta+alpha);
      u = mod2pi(alpha);
      v = mod2pi(phi-t+2*(u));
     }

   length_rs = RADCURV*(2*(u)+t+v);
   return(length_rs);
}

double RSPlanner::c_cucu_c(const double& x, const double& y,const double& phi,const double& rs,const double& rc,double& t,double& u,double& v)
{
   double a,b,u1,theta,alpha,length_rs,va1,va2;

   a = x+rs;
   b = y-rc;
   if ((fabs(a)<EPS3) && (fabs(b)<EPS3)) return(INFINITY);
   u1 = sqrt(a*a+b*b);
   if (u1 > 6*RADCURV) return(INFINITY);
   theta = my_atan2(b,a);
   va1 = (5*SQRADCURV - u1*u1/4)/SQRADCURVMUL2;
   if ((va1 < 0.0) || (va1 > 1.0)) return(INFINITY);
   u = acos(va1);
   va2 = sin(u);
   alpha = asin(RADCURVMUL2*va2/u1);
   t = mod2pi(MPIDIV2+theta+alpha);
   v = mod2pi(t-phi);

   length_rs = RADCURV*(2*(u)+t+v);
   return(length_rs);
}

double RSPlanner::c_c2sca(const double& x, const double& y,const double& phi,const double& rs,const double& rc,double& t,double& u,double& v)
{
   double a,b,u1,theta,alpha,length_rs;

   a = x-rs;
   b = y+rc;
   u1 = sqrt(a*a+b*b);
   if (u1 < RADCURVMUL2) return(INFINITY);
   theta = my_atan2(b,a);
   u = sqrt(u1*u1-SQRADCURVMUL2) - RADCURVMUL2;
   if (u < 0.0) return(INFINITY);
   alpha = my_atan2(RADCURVMUL2,(u+RADCURVMUL2));
   t = mod2pi(MPIDIV2+theta+alpha);
   v = mod2pi(t+MPIDIV2-phi);

   length_rs = RADCURV*(t+MPIDIV2+v) + u;
   return(length_rs);
}

double RSPlanner::c_c2scb(const double& x, const double& y,const double& phi,const double& rs,const double& rc,double& t,double& u,double& v)
{
   double a,b,u1,theta,length_rs;

   a = x+rs;
   b = y-rc;
   u1 = sqrt(a*a+b*b);
   if (u1 < RADCURVMUL2) return(INFINITY);
   theta = my_atan2(b,a);
   t = mod2pi(MPIDIV2+theta);
   u = u1-RADCURVMUL2;
   v = mod2pi(phi-t-MPIDIV2);

   length_rs = RADCURV*(t+MPIDIV2+v) + u;
   return(length_rs);
}

double RSPlanner::c_c2sc2_c(const double& x, const double& y,const double& phi,const double& rs,const double& rc,double& t,double& u,double& v)
{
   double a,b,u1,theta,alpha,length_rs;

   a = x+rs;
   b = y-rc;
   u1 = sqrt(a*a+b*b);
   if (u1 < RADCURVMUL4) return(INFINITY);
   theta = my_atan2(b,a);
   u = sqrt(u1*u1-SQRADCURVMUL2) - RADCURVMUL4;
   if (u < 0.0) return(INFINITY);
   alpha = my_atan2(RADCURVMUL2,(u+RADCURVMUL4));
   t = mod2pi(MPIDIV2+theta+alpha);
   v = mod2pi(t-phi);

   length_rs = RADCURV*(t+MPI+v) + u;
   return(length_rs);
}

double RSPlanner::cc_c(const double& x, const double& y,const double& phi,const double& rs,const double& rc,double& t,double& u,double& v)
{
   double a,b,u1,theta,alpha,length_rs,va;

   a = x-rs;
   b = y+rc;
   if ((fabs(a)<EPS3) && (fabs(b)<EPS3))
	   return(INFINITY);

   u1 = sqrt(a*a+b*b);

   if (u1>RADCURVMUL4)
	   return(INFINITY);

   theta = my_atan2(b,a);

   u = acos((8*SQRADCURV - u1*u1)/(8*SQRADCURV));

   va = sin(u);

   if (fabs(va)<0.001)
	   va = 0.0;

   if ((fabs(va)<0.001) && (fabs(u1)<0.001))
	   return(INFINITY);

   alpha = asin(RADCURVMUL2*va/u1);
   t = mod2pi(MPIDIV2 - alpha + theta);
   v = mod2pi(t-u-phi);

   length_rs = RADCURV*(t+u+v);
   return(length_rs);
}

double RSPlanner::csc2_ca(const double& x, const double& y,const double& phi,const double& rs,const double& rc,double& t,double& u,double& v)
{
   double a,b,u1,theta,alpha,length_rs;

   a = x-rs;
   b = y+rc;
   u1 = sqrt(a*a+b*b);
   if (u1 < RADCURVMUL2) return(INFINITY);
   theta = my_atan2(b,a);
   u = sqrt(u1*u1-SQRADCURVMUL2) - RADCURVMUL2;
   if (u < 0.0) return(INFINITY);
   alpha = my_atan2((u+RADCURVMUL2),RADCURVMUL2);
   t = mod2pi(MPIDIV2+theta-alpha);
   v = mod2pi(t-MPIDIV2-phi);

   length_rs = RADCURV*(t+MPIDIV2+v) + u;
   return(length_rs);
}

double RSPlanner::csc2_cb(const double& x, const double& y,const double& phi,const double& rs,const double& rc,double& t,double& u,double& v)
{
   double a,b,u1,theta,length_rs;

   a = x+rs;
   b = y-rc;
   u1 = sqrt(a*a+b*b);
   if (u1 < RADCURVMUL2) return(INFINITY);
   theta = my_atan2(b,a);
   t = mod2pi(theta);
   u = u1 - RADCURVMUL2;
   v = mod2pi(-t-MPIDIV2+phi);

   length_rs = RADCURV*(t+MPIDIV2+v) + u;
   return(length_rs);
}


double RSPlanner::reed_shepp(const double& x1,const double& y1,const double& t1,const double& x2,const double& y2,const double& t2,int& numero,double& tr,double& ur,double& vr)
{
   double x,y,phi;
   double tn,un,vn;

   double vard,theta,alpha,dx,dy;
   double sphi,cphi;
   double ap,am,b1,b2;

/* coordinate change */
   dx = x2 - x1;
   dy = y2 - y1;
   theta = my_atan2(dy,dx);
   alpha = theta - t1;
   vard = sqrt(dx*dx+dy*dy);
   x = cos(alpha)*vard;
   y = sin(alpha)*vard;
   phi = t2 - t1;

   sphi = sin(phi);
   cphi = cos(phi);

   ap = RADCURV*sphi;
   am = -RADCURV*sphi;
   b1 = RADCURV*(cphi-1);
   b2 = RADCURV*(cphi+1);

/*   C | C | C   */
   vector<ConfigItem> configList;
   ConfigItem conf;

   conf.length = (float)c_c_c(x,y,phi,ap,b1,tn,un,vn);
   conf.num = 1;
   conf.t = tn; conf.u = un; conf.v = vn;
   if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)c_c_c(-x,y,-phi,am,b1,tn,un,vn);
   conf.num = 2;
   conf.t = tn; conf.u = un; conf.v = vn;
   if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)c_c_c(x,-y,-phi,am,b1,tn,un,vn);
   conf.num = 3;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)c_c_c(-x,-y,phi,ap,b1,tn,un,vn);
   conf.num = 4;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

/*   C | C C   */

   conf.length = (float)c_cc(x,y,phi,ap,b1,tn,un,vn);
   conf.num = 5;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)c_cc(-x,y,-phi,am,b1,tn,un,vn);
   conf.num = 6;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)c_cc(x,-y,-phi,am,b1,tn,un,vn);
   conf.num = 7;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)c_cc(-x,-y,phi,ap,b1,tn,un,vn);
   conf.num = 8;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

/*   C S C   */

   conf.length = (float)csca(x,y,phi,ap,b1,tn,un,vn);
   conf.num = 9;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)csca(x,-y,-phi,am,b1,tn,un,vn);
   conf.num = 10;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)csca(-x,y,-phi,am,b1,tn,un,vn);
   conf.num = 11;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)csca(-x,-y,phi,ap,b1,tn,un,vn);
   conf.num = 12;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)cscb(x,y,phi,ap,b2,tn,un,vn);
   conf.num = 13;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)cscb(x,-y,-phi,am,b2,tn,un,vn);
   conf.num = 14;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)cscb(-x,y,-phi,am,b2,tn,un,vn);
   conf.num = 15;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)cscb(-x,-y,phi,ap,b2,tn,un,vn);
   conf.num = 16;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

/*   C Cu | Cu C   */

   conf.length = (float)ccu_cuc(x,y,phi,ap,b2,tn,un,vn);
   conf.num = 17;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)ccu_cuc(x,-y,-phi,am,b2,tn,un,vn);
   conf.num = 18;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)ccu_cuc(-x,y,-phi,am,b2,tn,un,vn);
   conf.num = 19;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)ccu_cuc(-x,-y,phi,ap,b2,tn,un,vn);
   conf.num = 20;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

/*   C | Cu Cu | C   */

   conf.length = (float)c_cucu_c(x,y,phi,ap,b2,tn,un,vn);
   conf.num = 21;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)c_cucu_c(x,-y,-phi,am,b2,tn,un,vn);
   conf.num = 22;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)c_cucu_c(-x,y,-phi,am,b2,tn,un,vn);
   conf.num = 23;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)c_cucu_c(-x,-y,phi,ap,b2,tn,un,vn);
   conf.num = 24;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

/*   C | C2 S C   */

   conf.length = (float)c_c2sca(x,y,phi,ap,b1,tn,un,vn);
   conf.num = 25;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)c_c2sca(x,-y,-phi,am,b1,tn,un,vn);
   conf.num = 26;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)c_c2sca(-x,y,-phi,am,b1,tn,un,vn);
   conf.num = 27;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)c_c2sca(-x,-y,phi,ap,b1,tn,un,vn);
   conf.num = 28;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)c_c2scb(x,y,phi,ap,b2,tn,un,vn);
   conf.num = 29;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)c_c2scb(x,-y,-phi,am,b2,tn,un,vn);
   conf.num = 30;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)c_c2scb(-x,y,-phi,am,b2,tn,un,vn);
   conf.num = 31;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)c_c2scb(-x,-y,phi,ap,b2,tn,un,vn);
   conf.num = 32;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

/*   C | C2 S C2 | C   */

   conf.length = (float)c_c2sc2_c(x,y,phi,ap,b2,tn,un,vn);
   conf.num = 33;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)c_c2sc2_c(x,-y,-phi,am,b2,tn,un,vn);
   conf.num = 34;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)c_c2sc2_c(-x,y,-phi,am,b2,tn,un,vn);
   conf.num = 35;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)c_c2sc2_c(-x,-y,phi,ap,b2,tn,un,vn);
   conf.num = 36;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

/*   C C | C   */

   conf.length = (float)cc_c(x,y,phi,ap,b1,tn,un,vn);
   conf.num = 37;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)cc_c(x,-y,-phi,am,b1,tn,un,vn);
   conf.num = 38;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)cc_c(-x,y,-phi,am,b1,tn,un,vn);
   conf.num = 39;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)cc_c(-x,-y,phi,ap,b1,tn,un,vn);
   conf.num = 40;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

/*   C S C2 | C   */

   conf.length = (float)csc2_ca(x,y,phi,ap,b1,tn,un,vn);
   conf.num = 41;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)csc2_ca(x,-y,-phi,am,b1,tn,un,vn);
   conf.num = 42;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)csc2_ca(-x,y,-phi,am,b1,tn,un,vn);
   conf.num = 43;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)csc2_ca(-x,-y,phi,ap,b1,tn,un,vn);
   conf.num = 44;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)csc2_cb(x,y,phi,ap,b2,tn,un,vn);
   conf.num = 45;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)csc2_cb(x,-y,-phi,am,b2,tn,un,vn);
   conf.num = 46;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)csc2_cb(-x,y,-phi,am,b2,tn,un,vn);
   conf.num = 47;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);

   conf.length = (float)csc2_cb(-x,-y,phi,ap,b2,tn,un,vn);
   conf.num = 48;
      conf.t = tn; conf.u = un; conf.v = vn;
      if(conf.length > 0) configList.push_back(conf);


	  if(configList.size() > 0)
	  {
		   conf = configList.at(0);
		  for(unsigned int i=0; i< configList.size(); i++)
		  {
			  if(configList.at(i).length > 0 &&  configList.at(i).length < conf.length)
			  {
				  conf = configList.at(i);
			  }
		  }
	  }

   tr = conf.t; ur = conf.u; vr = conf.v;
   numero = conf.num;
   return(conf.length);
}

double RSPlanner::min_length_rs(const double& x1,const double& y1,const double& t1,const double& x2,const double& y2,const double& t2,int& numero,double& t,double& u,double& v)
{
   double length_rs;

   if ((fabs(x1-x2)<EPS1) && (fabs(y1-y2)<EPS1) && (fabs(t1-t2)<EPS1))
	   length_rs = 0.0;
   else
	   length_rs = reed_shepp(x1,y1,t1,x2,y2,t2,numero,t,u,v);

   return(length_rs);
}

int RSPlanner::fct_curve(const int& ty,const int& orientation,const double& val,double& x1,double& y1,double& t1,const double& delta,vector<WayPoint>& path,int n)
{
   int i;
   double va1,va2,newval,incrt,remain;
   double center_x,center_y;
   double x2,y2,t2;
   int nnew;
   double local_delta = delta;

   if (ty == 3) //TODO check this if the planner doesn't work
   {
      if (fabs(val/RADCURV)<EPS4) return(0);
   else
      if (fabs(val)<EPS4) return(0);
   }

   if(path.size()==1)
   {
	   if(orientation == -1) path.at(0).bDir = BACKWARD_DIR;
	   	else path.at(0).bDir = FORWARD_DIR;
   }

   WayPoint p;

   switch(ty)
     {
      case 1 : /* circular arc toward the right */
      {
    	  local_delta = local_delta/RADCURV;
			center_x = x1 + RADCURV*sin(t1);
			center_y = y1 - RADCURV*cos(t1);
			va1 = t1+MPIDIV2;

			if (orientation == 1)
				va2 = va1-val;
			else
				va2 = va1+val;

			x2 = center_x + RADCURV*cos(va2);
			y2 = center_y + RADCURV*sin(va2);
			t2 = t1 - orientation*val;

			nnew = val/local_delta;

			remain = val - nnew*local_delta;

			nnew = nnew+n;

			if (orientation == -1)
				local_delta = -local_delta;

			incrt = 0;
			for (i = n; i<nnew; i++)
			{
			   va1 = va1-local_delta;
//			   *(pathx+i) = center_x + RADCURV*cos(va1);
//			   *(pathy+i) = center_y + RADCURV*sin(va1);

			   incrt = incrt - local_delta;
//			   *(patht+i) = mod2pi(t1 + incrt);
			   p.pos.x = center_x + RADCURV*cos(va1);
			   p.pos.y = center_y + RADCURV*sin(va1);
			   p.pos.a = mod2pi(t1 + incrt);
			   if(orientation == -1) p.bDir = BACKWARD_DIR;
			   else p.bDir = FORWARD_DIR;
			   path.push_back(p);
			}
			n = nnew;
			if (remain > fabs(local_delta)/5.)
			{
				p = WayPoint(x2, y2, 0, mod2pi(t2));
				if(orientation == -1) p.bDir = BACKWARD_DIR;
				else p.bDir = FORWARD_DIR;
				path.push_back(p);
//			   *(pathx+nnew) = x2;
//			   *(pathy+nnew) = y2;
//			   *(patht+nnew) = mod2pi(t2);
			   n++;
			}
			else
			{
				p = WayPoint(x2, y2,0, mod2pi(t2));
				if(orientation == -1) p.bDir = BACKWARD_DIR;
				else p.bDir = FORWARD_DIR;
				path.at(path.size()-1) = p;
//			   *(pathx+nnew-1) = x2;
//			   *(pathy+nnew-1) = y2;
//			   *(patht+nnew-1) = mod2pi(t2);
			}
		}
        break;

      case 2 : /* circular arc toward the left */
      {
    	  local_delta = local_delta/RADCURV;
        center_x = x1 - RADCURV*sin(t1);
        center_y = y1 + RADCURV*cos(t1);
        va1 = t1-MPIDIV2;
        if (orientation == 1)
        	va2 = va1+val;
        else
        	va2 = va1-val;

        x2 = center_x + RADCURV*cos(va2);
        y2 = center_y + RADCURV*sin(va2);
        t2 = t1 + orientation*val;

        nnew = val/local_delta;
        remain = val - nnew*local_delta;
        nnew = nnew+n;

        if (orientation == -1) local_delta = -local_delta;
        incrt = 0;
        for (i = n; i<nnew; i++)
	  {
           va1 = va1+local_delta;
           incrt = incrt + local_delta;
//           *(pathx+i) = center_x + RADCURV*cos(va1);
//           *(pathy+i) = center_y + RADCURV*sin(va1);
//           *(patht+i) = mod2pi(t1 + incrt);
           p.pos.x = center_x + RADCURV*cos(va1);
		   p.pos.y = center_y + RADCURV*sin(va1);
		   p.pos.a = mod2pi(t1 + incrt);
		   if(orientation == -1) p.bDir = BACKWARD_DIR;
		   else p.bDir = FORWARD_DIR;
		   path.push_back(p);
	  }
        n = nnew;
        if (remain > fabs(local_delta)/5.)
	  {
//           *(pathx+nnew) = x2;
//           *(pathy+nnew) = y2;
//           *(patht+nnew) = mod2pi(t2);
        	p = WayPoint(x2, y2, 0, mod2pi(t2));
			if(orientation == -1) p.bDir = BACKWARD_DIR;
			else p.bDir = FORWARD_DIR;
			path.push_back(p);
           n++;
	 }
        else
	  {
//           *(pathx+nnew-1) = x2;
//           *(pathy+nnew-1) = y2;
//           *(patht+nnew-1) = mod2pi(t2);
        	p = WayPoint(x2, y2,0, mod2pi(t2));
			if(orientation == -1) p.bDir = BACKWARD_DIR;
			else p.bDir = FORWARD_DIR;
			path.at(path.size()-1) = p;
	  }
      }
        break;

      case 3 : /* straight line */
        x2 = x1 + orientation*val*cos(t1);
        y2 = y1 + orientation*val*sin(t1);
        t1 = mod2pi(t1);
        t2 = t1;

        va1 = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
        i = va1/PATHDENSITY;
        remain = va1 - i*PATHDENSITY;
        nnew = n+i;
        newval = PATHDENSITY;
        va1 = orientation*cos(t1);
        va2 = orientation*sin(t1);
        for (i = n; i<nnew; i++)
	  {
//           *(pathx+i) = x1 + va1*newval;
//           *(pathy+i) = y1 + va2*newval;
//           *(patht+i) = t1;
        	p = WayPoint(x1 + va1*newval, y1 + va2*newval,0, t1);
			if(orientation == -1) p.bDir = BACKWARD_DIR;
			else p.bDir = FORWARD_DIR;
			path.push_back(p);
           newval = newval + PATHDENSITY;

	  }
        n = nnew;
        if (remain > 0.04)
	  {
//            *(pathx+nnew) = x2;
//            *(pathy+nnew) = y2;
//            *(patht+nnew) = t2;
        	p = WayPoint(x2, y2,0, t2);
			if(orientation == -1) p.bDir = BACKWARD_DIR;
			else p.bDir = FORWARD_DIR;
			path.push_back(p);
            n = nnew+1;
	  }
        else
	  {
//            *(pathx+nnew-1) = x2;
//            *(pathy+nnew-1) = y2;
//            *(patht+nnew-1) = t2;
        	p = WayPoint(x2, y2,0, t2);
			if(orientation == -1) p.bDir = BACKWARD_DIR;
			else p.bDir = FORWARD_DIR;
			path.at(path.size()-1) = p;
            n = nnew;
	  }
     }

   x1 = x2;
   y1 = y2;
   t1 = t2;

   return(n);
}

int RSPlanner::constRS(int num,double t,double u,double v,double x1,double y1,double t1,double delta,vector<WayPoint>& path)
{
   int left,right,straight,fwd,bwd;
   int n;

//   *pathx = x1;
//   *pathy = y1;
//   *patht = t1;
//
   path.push_back(WayPoint(x1,y1,0,t1));
   n = 1;

   right = 1; left = 2; straight = 3;
   fwd = 1; bwd = -1;

   switch(num)
     {

/*   C | C | C   */

       case 1 :
         n = fct_curve(left,fwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(right,bwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(left,fwd,v,x1,y1,t1,delta,path,n);
         break;

       case 2 :
         n = fct_curve(left,bwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(right,fwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(left,bwd,v,x1,y1,t1,delta,path,n);
         break;

       case 3 :
         n = fct_curve(right,fwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(left,bwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(right,fwd,v,x1,y1,t1,delta,path,n);
         break;

       case 4 :
         n = fct_curve(right,bwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(left,fwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(right,bwd,v,x1,y1,t1,delta,path,n);
         break;

/*   C | C C   */

       case 5 :
         n = fct_curve(left,fwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(right,bwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(left,bwd,v,x1,y1,t1,delta,path,n);
         break;

       case 6 :
         n = fct_curve(left,bwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(right,fwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(left,fwd,v,x1,y1,t1,delta,path,n);
         break;

       case 7 :
         n = fct_curve(right,fwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(left,bwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(right,bwd,v,x1,y1,t1,delta,path,n);
         break;

       case 8 :
         n = fct_curve(right,bwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(left,fwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(right,fwd,v,x1,y1,t1,delta,path,n);
         break;

/*   C S C   */

       case 9 :
         n = fct_curve(left,fwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(straight,fwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(left,fwd,v,x1,y1,t1,delta,path,n);
         break;

       case 10 :
         n = fct_curve(right,fwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(straight,fwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(right,fwd,v,x1,y1,t1,delta,path,n);
         break;

       case 11 :
         n = fct_curve(left,bwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(straight,bwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(left,bwd,v,x1,y1,t1,delta,path,n);
         break;

       case 12 :
         n = fct_curve(right,bwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(straight,bwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(right,bwd,v,x1,y1,t1,delta,path,n);
         break;

       case 13 :
         n = fct_curve(left,fwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(straight,fwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(right,fwd,v,x1,y1,t1,delta,path,n);
         break;

       case 14 :
         n = fct_curve(right,fwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(straight,fwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(left,fwd,v,x1,y1,t1,delta,path,n);
         break;

       case 15 :
         n = fct_curve(left,bwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(straight,bwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(right,bwd,v,x1,y1,t1,delta,path,n);
         break;

       case 16 :
         n = fct_curve(right,bwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(straight,bwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(left,bwd,v,x1,y1,t1,delta,path,n);
         break;

/*   C Cu | Cu C   */

       case 17 :
         n = fct_curve(left,fwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(right,fwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(left,bwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(right,bwd,v,x1,y1,t1,delta,path,n);
         break;

       case 18 :
         n = fct_curve(right,fwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(left,fwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(right,bwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(left,bwd,v,x1,y1,t1,delta,path,n);
         break;

       case 19 :
         n = fct_curve(left,bwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(right,bwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(left,fwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(right,fwd,v,x1,y1,t1,delta,path,n);
         break;

       case 20 :
         n = fct_curve(right,bwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(left,bwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(right,fwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(left,fwd,v,x1,y1,t1,delta,path,n);
         break;

/*   C | Cu Cu | C   */

       case 21 :
         n = fct_curve(left,fwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(right,bwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(left,bwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(right,fwd,v,x1,y1,t1,delta,path,n);
         break;

       case 22 :
         n = fct_curve(right,fwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(left,bwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(right,bwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(left,fwd,v,x1,y1,t1,delta,path,n);
         break;

       case 23 :
         n = fct_curve(left,bwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(right,fwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(left,fwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(right,bwd,v,x1,y1,t1,delta,path,n);
         break;

       case 24 :
         n = fct_curve(right,bwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(left,fwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(right,fwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(left,bwd,v,x1,y1,t1,delta,path,n);
         break;

/*   C | C2 S C   */

       case 25 :
         n = fct_curve(left,fwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(right,bwd,MPIDIV2,x1,y1,t1,delta,path,n);
         n = fct_curve(straight,bwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(left,bwd,v,x1,y1,t1,delta,path,n);
         break;

       case 26 :
         n = fct_curve(right,fwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(left,bwd,MPIDIV2,x1,y1,t1,delta,path,n);
         n = fct_curve(straight,bwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(right,bwd,v,x1,y1,t1,delta,path,n);
         break;

       case 27 :
         n = fct_curve(left,bwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(right,fwd,MPIDIV2,x1,y1,t1,delta,path,n);
         n = fct_curve(straight,fwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(left,fwd,v,x1,y1,t1,delta,path,n);
         break;

       case 28 :
         n = fct_curve(right,bwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(left,fwd,MPIDIV2,x1,y1,t1,delta,path,n);
         n = fct_curve(straight,fwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(right,fwd,v,x1,y1,t1,delta,path,n);
         break;

       case 29 :
         n = fct_curve(left,fwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(right,bwd,MPIDIV2,x1,y1,t1,delta,path,n);
         n = fct_curve(straight,bwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(right,bwd,v,x1,y1,t1,delta,path,n);
         break;

       case 30 :
         n = fct_curve(right,fwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(left,bwd,MPIDIV2,x1,y1,t1,delta,path,n);
         n = fct_curve(straight,bwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(left,bwd,v,x1,y1,t1,delta,path,n);
         break;

       case 31 :
         n = fct_curve(left,bwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(right,fwd,MPIDIV2,x1,y1,t1,delta,path,n);
         n = fct_curve(straight,fwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(right,fwd,v,x1,y1,t1,delta,path,n);
         break;

       case 32 :
         n = fct_curve(right,bwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(left,fwd,MPIDIV2,x1,y1,t1,delta,path,n);
         n = fct_curve(straight,fwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(left,fwd,v,x1,y1,t1,delta,path,n);
         break;

/*   C | C2 S C2 | C   */

       case 33 :
         n = fct_curve(left,fwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(right,bwd,MPIDIV2,x1,y1,t1,delta,path,n);
         n = fct_curve(straight,bwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(left,bwd,MPIDIV2,x1,y1,t1,delta,path,n);
         n = fct_curve(right,fwd,v,x1,y1,t1,delta,path,n);
         break;

       case 34 :
         n = fct_curve(right,fwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(left,bwd,MPIDIV2,x1,y1,t1,delta,path,n);
         n = fct_curve(straight,bwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(right,bwd,MPIDIV2,x1,y1,t1,delta,path,n);
         n = fct_curve(left,fwd,v,x1,y1,t1,delta,path,n);
         break;

       case 35 :
         n = fct_curve(left,bwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(right,fwd,MPIDIV2,x1,y1,t1,delta,path,n);
         n = fct_curve(straight,fwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(left,fwd,MPIDIV2,x1,y1,t1,delta,path,n);
         n = fct_curve(right,bwd,v,x1,y1,t1,delta,path,n);
         break;

       case 36 :
         n = fct_curve(right,bwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(left,fwd,MPIDIV2,x1,y1,t1,delta,path,n);
         n = fct_curve(straight,fwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(right,fwd,MPIDIV2,x1,y1,t1,delta,path,n);
         n = fct_curve(left,bwd,v,x1,y1,t1,delta,path,n);
         break;

/*   C C | C   */

       case 37 :
         n = fct_curve(left,fwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(right,fwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(left,bwd,v,x1,y1,t1,delta,path,n);
         break;

       case 38 :
         n = fct_curve(right,fwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(left,fwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(right,bwd,v,x1,y1,t1,delta,path,n);
         break;

       case 39 :
         n = fct_curve(left,bwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(right,bwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(left,fwd,v,x1,y1,t1,delta,path,n);
         break;

       case 40 :
         n = fct_curve(right,bwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(left,bwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(right,fwd,v,x1,y1,t1,delta,path,n);
         break;

/*   C S C2 | C   */

       case 41 :
         n = fct_curve(left,fwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(straight,fwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(right,fwd,MPIDIV2,x1,y1,t1,delta,path,n);
         n = fct_curve(left,bwd,v,x1,y1,t1,delta,path,n);
         break;

       case 42 :
         n = fct_curve(right,fwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(straight,fwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(left,fwd,MPIDIV2,x1,y1,t1,delta,path,n);
         n = fct_curve(right,bwd,v,x1,y1,t1,delta,path,n);
         break;

       case 43 :
         n = fct_curve(left,bwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(straight,bwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(right,bwd,MPIDIV2,x1,y1,t1,delta,path,n);
         n = fct_curve(left,fwd,v,x1,y1,t1,delta,path,n);
         break;

       case 44 :
         n = fct_curve(right,bwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(straight,bwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(left,bwd,MPIDIV2,x1,y1,t1,delta,path,n);
         n = fct_curve(right,fwd,v,x1,y1,t1,delta,path,n);
         break;

       case 45 :
         n = fct_curve(left,fwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(straight,fwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(left,fwd,MPIDIV2,x1,y1,t1,delta,path,n);
         n = fct_curve(right,bwd,v,x1,y1,t1,delta,path,n);
         break;

       case 46 :
         n = fct_curve(right,fwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(straight,fwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(right,fwd,MPIDIV2,x1,y1,t1,delta,path,n);
         n = fct_curve(left,bwd,v,x1,y1,t1,delta,path,n);
         break;

       case 47 :
         n = fct_curve(left,bwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(straight,bwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(left,bwd,MPIDIV2,x1,y1,t1,delta,path,n);
         n = fct_curve(right,fwd,v,x1,y1,t1,delta,path,n);
         break;

       case 48 :
         n = fct_curve(right,bwd,t,x1,y1,t1,delta,path,1);
         n = fct_curve(straight,bwd,u,x1,y1,t1,delta,path,n);
         n = fct_curve(right,bwd,MPIDIV2,x1,y1,t1,delta,path,n);
         n = fct_curve(left,fwd,v,x1,y1,t1,delta,path,n);
         break;
       default:
    	   break;
         //printf("Error: RS curve type %d unknown\n",num);
     }

   return n;
}

} /* namespace PlannerZNS */
