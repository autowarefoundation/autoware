#ifndef _FCONVS_MT_H_
#define _FCONVS_MT_H_

extern int part_error_array_num;
extern int *part_error_array;
extern size_t SUM_SIZE_C;
extern FLOAT *dst_C;

/* function for GPU execution correspond to fconvsMT */
extern FLOAT ***fconvsMT_GPU(FLOAT **featp2,
			     size_t SUM_SIZE_feat,
			     FLOAT **filter,
			     int *sym_info,
			     int start,
			     int end,
			     int *A_SIZE,
			     int **B_SIZE,
			     int **M_size_array,
			     int L_MAX,
			     int interval,
			     int *FSIZE,
			     int padx,
			     int pady,
			     int max_X,
			     int max_Y,
			     int calc_flag);

#endif /* _FCONVS_MT_H_ */
