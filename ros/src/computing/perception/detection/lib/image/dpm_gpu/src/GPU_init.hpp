#ifndef _GPU_INIT_H_
#define _GPU_INIT_H_

/* functions for using GPU and to calculate on GPU */
extern void init_cuda(void);
extern void init_cuda_with_cubin(const char *cubin_path);
extern void clean_cuda(void);

#endif /* _GPU_INIT_H_ */
