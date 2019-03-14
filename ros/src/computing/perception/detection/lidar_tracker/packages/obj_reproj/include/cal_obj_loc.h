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

#ifndef CALSELFLOC
#define CALSELFLOC

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
