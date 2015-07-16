#ifndef _LOAD_MODEL_H_
#define _LOAD_MODEL_H_

extern MODEL *load_model(FLOAT ratio, const char *com_csv, const char *root_csv, const char *part_csv);
extern void free_model(MODEL *MO);

#endif /* _LOAD_MODEL_H_ */
