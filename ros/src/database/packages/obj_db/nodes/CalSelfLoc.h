
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

        double a,b,c;
        a = U*U/(fkx*fkx) + V*V/(fky*fky) + 1;
        b = -Ox*U/fkx - Oy*V/fky;
        c = Ox*Ox/(fkx*fkx) + Oy*Oy/(fky*fky) - distance*distance;
        
        res.Z = (-b + sqrt(b*b-a*c))/a;
        res.X = (U*res.Z-Ox)/fkx;
        res.Y = (V*res.Z-Oy)/fky;

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
