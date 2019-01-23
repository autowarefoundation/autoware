/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CALCOORDINATES
#define CALCOORDINATES

#include <math.h>
#include <stdio.h>
#include "structure.h"

//this calculation refer to http://vldb.gsi.go.jp/sokuchi/surveycalc/surveycalc/algorithm/xy2bl/xy2bl.htm
class calcoordinates{
public:
    calcoordinates(){};

    RESULT cal(double x0,double y0 ,double phizero, double lamzero){
        x = x0;
        y = y0;
        phi0 = phizero*M_PI/180;
        lam0 = lamzero*M_PI/180;
        
        //circle n depend on previous number circle.
         
        //circle 1
        double n = 1/(2*F-1); //*

        //circle 2  pow(n,)
        double B[6]; //*
        B[0] = 0;
        B[1] = n/2 - (2/3)*n*n + (37/96)*pow(n,3) - (1/360)*pow(n,4) - (81*512)*pow(n,5);
        B[2] = (1/48)*n*n + (1/15)*pow(n,3) - (437/1440)*pow(n,4) + (46/105)*pow(n,5);
        B[3] = (17/480)*pow(n,3) - (37/840)*pow(n,4) - (209/4480)*pow(n,5);
        B[4] = (4397/161280)*pow(n,4) - (11/504)*pow(n,5);
        B[5] = (4583/161280)*pow(n,5);

        double A[6]; //*
        A[0] = 1 + (1/4)*n*n + (1/64)*pow(n,4);
        A[1] = -(3/2)*(n-(1/8)*pow(n,3)-(1/64)*pow(n,5));
        A[2] = (15/16)*(n*n-(1/4)*pow(n,4));
        A[3] = -(35/48)*(pow(n,3)-(5/16)*pow(n,5));
        A[4] = (315/512)*pow(n,4);
        A[5] = -(693/1280)*pow(n,5);
        
        double delta[7];
        delta[0] = 0;
        delta[1] = 2*n - (2/3)*n*n - 2*pow(n,3) + (116/45)*pow(n,4) + (26/45)*pow(n,5) - (2854/675)*pow(n,6);
        delta[2] = (7/3)*n*n - (8/5)*pow(n,3) - (227/45)*pow(n,4) + (2704/315)*pow(n,5) + (2323/945)*pow(n,6);
        delta[3] = (56/15)*pow(n,3) - (136/35)*pow(n,4) - (1262/105)*pow(n,5) + (73814/2835)*pow(n,6);
        delta[4] = (4279/630)*pow(n,4) - (332/35)*pow(n,5) - (399572/14175)*pow(n,6);
        delta[5] = (4174/315)*pow(n,5) - (144838/6237)*pow(n,6);
        delta[6] = (601676/22275)*pow(n,6);

        //circle 3
        double Abar = m0*a*A[0]/(1+n); //*

        double temp1 = A[0]*phi0/rowdd;
        for(int j=1; j<6 ; j++){
            temp1 += A[j]*sin(2*j*phi0);
        }
        double Spbar = m0*a*temp1/(1+n); //*

        //circle 4
        double xi = (x+Spbar)/Abar;
        double eta = y/Abar;
        
        //circle 5
        double xid = xi;
        for(int j=1 ; j<6 ; j++){
            xid -= B[j]*sin(2*j*xi)*cosh(2*j*eta); 
        }
        printf("%f %f\n",xi,xid);

        double etad = eta;
        for(int j=1 ; j<6 ; j++){
            etad -= B[j]*cos(2*j*xi)*sinh(2*j*eta); 
        }

        //circle 6
        double chi = asin(sin(xid)/cosh(etad));

        //result
        res.lat = chi;
        for(int j=1; j<7 ; j++){
            res.lat += rowdd*delta[j]*sin(2*j*chi);

        }
	res.lat = res.lat*180/M_PI;

        res.lon = (lam0 + atan(sinh(etad)/cos(xid)))*180/M_PI;
        //printf("%f\n",atan(sinh(etad)/cos(xid)));
        
        return res;
    }

private:
    const double a = 6378137;
    const double F = 1/298.257222101;

    const double m0 = 0.9999;
    const double rowdd = (180/M_PI)*3600;

    double x,y;
    double phi0,lam0;
    RESULT res;

};

#endif
