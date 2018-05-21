/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
