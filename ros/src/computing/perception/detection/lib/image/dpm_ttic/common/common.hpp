#ifndef _COMMON_H_
#define _COMMON_H_

#include <cstdlib>
#include <opencv/cv.h>

#include "switch_float.h"

//release function (for safety release)
#define s_free(a) do {	\
	std::free(a);	\
	a = nullptr;	\
} while(0)

extern void dpm_ttic_add_part_calculation(FLOAT *score, FLOAT*M,int *rootsize,int *partsize,int ax,int ay);
extern FLOAT *dpm_ttic_init_accumulated_score(IplImage *image, size_t& accumulated_size);

#endif /* _COMMON_H_ */
