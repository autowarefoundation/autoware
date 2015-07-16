#ifndef _DT_H_
#define _DT_H_

#include "switch_float.h"

//decide optimum part position
extern FLOAT *dt(FLOAT *vals,FLOAT ax,FLOAT bx,FLOAT ay,FLOAT by,int *dims,int *Ix,int *Iy);
//add part score
extern void add_part_calculation(FLOAT *score, FLOAT*M,int *rootsize,int *partsize,int ax,int ay);

#endif /* _DT_H_ */
