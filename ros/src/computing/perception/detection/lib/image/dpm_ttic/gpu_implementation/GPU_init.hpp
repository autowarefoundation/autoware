#ifndef _GPU_INIT_H_
#define _GPU_INIT_H_

/* functions for using GPU and to calculate on GPU */
extern void dpm_ttic_gpu_init_cuda(void);
extern void dpm_ttic_gpu_init_cuda_with_cubin(const char *cubin_path);
extern void dpm_ttic_gpu_clean_cuda(void);

#endif /* _GPU_INIT_H_ */
