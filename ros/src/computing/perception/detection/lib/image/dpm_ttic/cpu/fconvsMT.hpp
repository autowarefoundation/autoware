#ifndef _FCONVS_MT_H_
#define _FCONVS_MT_H_

#include "switch_float.h"

//convolve A and B
FLOAT **dpm_ttic_cpu_fconvsMT(FLOAT*feat,FLOAT*flfeat,FLOAT**filter,int *sym_info,
			      int start,int end,int *A_SIZE,int **B_SIZE,int *M_size);

#endif /* _FCONVS_MT_H_ */
