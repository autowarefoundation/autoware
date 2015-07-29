#ifndef _DT_GPU_H_
#define _DT_GPU_H_

extern FLOAT ****dt_GPU(int ****Ix_array,
			int ****Iy_array,
			int ***PIDX_array,
			int **size_array,
			int NoP,
			const int *numpart,
			int NoC,
			int interval,
			int L_MAX,
			int *FSIZE,
			int padx,
			int pady,
			int max_X,
			int max_Y,
			FLOAT *def,
			int tmp_array_size,
			int *dst_PIDX,
			int *dst_DID_4);

#endif /* _DT_GPU_H_ */

