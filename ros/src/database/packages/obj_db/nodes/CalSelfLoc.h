
#ifndef CALSELFLOC
#define CALSELFLOC

#include <math.h>
#include "structure.h"

class selfLocation{

public:
    selfLocation(){};

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

	res.Y = sqrt((distance*distance/(a+b+1)));
	res.X = (U-Ox)*res.Y/fkx;
        res.Z = (V-Oy)*res.Y/fky;

	//mili meter -> meter
        res.X = res.X/1000;
        res.Y = res.Y/1000;
	res.Z = res.Z/1000;

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
