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
