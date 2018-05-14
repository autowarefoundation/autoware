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

#ifndef AXIALMOVE
#define AXIALMOVE

#include <math.h>
#include "cal_obj_loc.h"
#include "structure.h"

class axiMove{
public:
    axiMove(){};

    LOCATION cal(LOCATION loc,double matrix[4][4]){
/**
calibration file is assumed that
axial z is front of car,
axial y is upper way
and axial x is left and right.

 */
        LOCATION newloc;
        //rotation around X
        newloc.X =
            matrix[0][0]*loc.X +
            matrix[0][1]*loc.Y +
            matrix[0][2]*loc.Z +
            matrix[0][3];

        newloc.Y =
            matrix[1][0]*loc.X +
            matrix[1][1]*loc.Y +
            matrix[1][2]*loc.Z +
            matrix[1][3];

        newloc.Z =
            matrix[2][0]*loc.X +
            matrix[2][1]*loc.Y +
            matrix[2][2]*loc.Z +
            matrix[2][3];

        newloc.W = 1;

        return newloc;
    }

    LOCATION cal(LOCATION loc,ANGLE ang){
        LOCATION newloc;
        //rotation around X
        newloc.Y = loc.Y*cos(ang.thiX) + loc.Z*sin(ang.thiX);
        newloc.Z = -loc.Y*sin(ang.thiX) + loc.Z*cos(ang.thiX);
        loc.Y = newloc.Y;
        loc.Z = newloc.Z;

        //rotation around Y
        newloc.X = loc.X*cos(ang.thiY) - loc.Z*sin(ang.thiY);
        newloc.Z = loc.X*sin(ang.thiY) + loc.Z*cos(ang.thiY);
        loc.X = newloc.X;
        loc.Z = newloc.Z;

        //rotation around Z
        newloc.X = loc.X*cos(ang.thiZ) + loc.Y*sin(ang.thiZ);
        newloc.Y = -loc.X*sin(ang.thiZ) + loc.Y*cos(ang.thiZ);
        loc.X = newloc.X;
        loc.Y = newloc.Y;

        return loc;
    }
};

#endif
