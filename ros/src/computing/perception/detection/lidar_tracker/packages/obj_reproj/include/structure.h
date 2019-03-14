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

#ifndef STRUCTURE_H
#define STRUCTURE_H

//converted value to view from above
typedef struct _LOCATION{
    double X;
    double Y;
    double Z;
    double W;
}LOCATION;

typedef struct _ANGLE{
    double thiX;
    double thiY;
    double thiZ;
}ANGLE;

typedef struct _RESULT{
    double lat;
    double lon;

}RESULT;

#endif
