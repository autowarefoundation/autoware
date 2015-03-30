#ifndef _common_2d_common_
#define _common_2d_common_

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <vector>

extern CvMat *v_g2l;
extern CvMat *v_g2c;
extern CvMat *v_rotation;
extern CvMat *m_rotation;
extern CvMat *m_intrinsic;
extern CvMat *m_dist;

extern void release_common();
#endif
