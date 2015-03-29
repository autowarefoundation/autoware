#include "common_2d_calib.h"

CvMat *v_g2l = cvCreateMat (1, 3, CV_32FC1);
CvMat *v_g2c = cvCreateMat (1, 3, CV_32FC1);
CvMat *v_rotation = cvCreateMat (1, 3, CV_32FC1);
CvMat *m_rotation = cvCreateMat (3, 3, CV_32FC1);
CvMat *m_intrinsic = cvCreateMat (3, 3, CV_32FC1);
CvMat *m_dist = cvCreateMat (1, 4, CV_32FC1);

void release_common()
{
    cvReleaseMat (&m_dist);
    cvReleaseMat (&m_intrinsic);
    cvReleaseMat (&v_rotation);
    cvReleaseMat (&v_g2l);
    cvReleaseMat (&v_g2c);
    cvReleaseMat (&m_rotation);
}
