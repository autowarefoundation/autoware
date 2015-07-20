#ifndef _LOAD_MODEL_H_
#define _LOAD_MODEL_H_

extern GPUModel *dpm_ttic_gpu_load_model(FLOAT ratio, const char *com_csv, const char *root_csv, const char *part_csv);
extern void dpm_ttic_gpu_free_model(GPUModel *MO);

#endif /* _LOAD_MODEL_H_ */
