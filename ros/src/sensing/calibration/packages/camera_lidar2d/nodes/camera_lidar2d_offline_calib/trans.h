#ifndef _trans_
#define _trans_

#include "data_struct.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>

extern Two_dimensional_vector trans_scan2scan_image(Three_dimensional_vector* scan, CvMat* v_g2l, CvMat* v_g2c, CvMat* m_rotation, CvMat* m_intrinsic);

#endif
