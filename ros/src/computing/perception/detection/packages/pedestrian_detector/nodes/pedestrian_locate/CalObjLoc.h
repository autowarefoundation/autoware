
#ifndef CALOBJLOC
#define CALOBJLOC

#include <math.h>
#include "structure.h"

class objLocation{

public:
    objLocation(){};

    void setCameraParam(double fx,double fy,double ox,double oy){
        fkx = fx;
        fky = fy;
        Ox = ox;
        Oy = oy;
    }

    void setOriginalValue(double u,double v,double d){
        U = u;
        V = v;
        distance = d;
    }

    LOCATION cal(){

        LOCATION res;

	double a = (U-Ox)*(U-Ox)/(fkx*fkx);
	double b = (V-Oy)*(V-Oy)/(fky*fky);

	res.Z = sqrt((distance*distance/(a+b+1)));
	res.X = (U-Ox)*res.Z/fkx;
        res.Y = (V-Oy)*res.Z/fky;

	//mili meter -> meter
        res.X = res.X/100;
        res.Y = res.Y/100;
	res.Z = res.Z/100;

        return res;

    }

    
private:


    double U;//X from camera picture
    double V;//Y from camera picture
    double distance;


    //camera parameter
    double fkx;
    double fky;
    double Ox;
    double Oy;

};

#endif
