#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include "image_window.h"
#include "common_2d_calib.h"

CvMat *v_g2c_const = cvCreateMat (1, 3, CV_32FC1);
CvMat *m_rotation_temp = cvCreateMat (3, 3, CV_32FC1);

void vector_x_bar(int val)
{
    cvmSet(v_g2c, 0, 0, cvmGet(v_g2c_const, 0, 0) + val - 1000);
}
void vector_y_bar(int val)
{
    cvmSet(v_g2c, 0, 1, cvmGet(v_g2c_const, 0, 1) + val - 1000);
}
void vector_z_bar(int val)
{
    cvmSet(v_g2c, 0, 2, cvmGet(v_g2c_const, 0, 2) + val - 1000);
}
void rad_x_bar(int val)
{
    static int previous_val_x = 180;
    CvMat m_rotation_x;
    float init_m_rotation_x[9] ={
        1, 0, 0,
        0, (float)cos((val-previous_val_x)/180.0*M_PI), (float)-sin((val-previous_val_x)/180.0*M_PI),
        0, (float)sin((val-previous_val_x)/180.0*M_PI), (float)cos((val-previous_val_x)/180.0*M_PI)};

    cvInitMatHeader(&m_rotation_x, 3, 3, CV_32FC1, init_m_rotation_x, CV_AUTOSTEP);
    cvCopy(m_rotation, m_rotation_temp);
    cvmMul(m_rotation_temp, &m_rotation_x, m_rotation);
    previous_val_x = val;
}
void rad_y_bar(int val)
{
    static int previous_val_y = 180;
    CvMat m_rotation_y;
    float init_m_rotation_y[9] ={
        (float)cos((val-previous_val_y)/180.0*M_PI), 0, (float)sin((val-previous_val_y)/180.0*M_PI),
        0, 1, 0,
        (float)-sin((val-previous_val_y)/180.0*M_PI), 0, (float)cos((val-previous_val_y)/180.0*M_PI)};

    cvInitMatHeader(&m_rotation_y, 3, 3, CV_32FC1, init_m_rotation_y, CV_AUTOSTEP);
    cvCopy(m_rotation, m_rotation_temp);
    cvmMul(m_rotation_temp, &m_rotation_y, m_rotation);
    previous_val_y = val;
}
void rad_z_bar(int val)
{
    static int previous_val_z = 180;
    CvMat m_rotation_z;
    float init_m_rotation_z[9] ={
        (float)cos((val-previous_val_z)/180.0*M_PI), (float)-sin((val-previous_val_z)/180.0*M_PI), 0,
        (float)sin((val-previous_val_z)/180.0*M_PI), (float)cos((val-previous_val_z)/180.0*M_PI), 0,
        0, 0, 1};

    cvInitMatHeader(&m_rotation_z, 3, 3, CV_32FC1, init_m_rotation_z);
    cvCopy(m_rotation, m_rotation_temp);
    cvmMul(m_rotation_temp, &m_rotation_z, m_rotation);
    previous_val_z = val;
}

void set_rotation(CvMat* m_rotation)
{
    cvCopy(m_rotation, m_rotation_temp);
}

void set_g2c(CvMat* v_g2c)
{
    cvCopy(v_g2c, v_g2c_const);
}

void release_image_window()
{
    cvReleaseMat (&v_g2c_const);
    cvReleaseMat (&m_rotation_temp);
}
