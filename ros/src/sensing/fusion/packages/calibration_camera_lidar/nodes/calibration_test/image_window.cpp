/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include "image_window.h"
#include "common_2d_calib.h"

CvMat *v_g2c_const = cvCreateMat (1, 3, CV_64FC1);
CvMat *m_rotation_temp = cvCreateMat (3, 3, CV_64FC1);

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
    double init_m_rotation_x[9] ={
        1, 0, 0,
        0, (double)cos((val-previous_val_x)/180.0*M_PI), (double)-sin((val-previous_val_x)/180.0*M_PI),
        0, (double)sin((val-previous_val_x)/180.0*M_PI), (double)cos((val-previous_val_x)/180.0*M_PI)};

    cvInitMatHeader(&m_rotation_x, 3, 3, CV_64FC1, init_m_rotation_x, CV_AUTOSTEP);
    cvCopy(m_rotation, m_rotation_temp);
    cvMatMulAdd(m_rotation_temp, &m_rotation_x, 0, m_rotation);
    previous_val_x = val;
}
void rad_y_bar(int val)
{
    static int previous_val_y = 180;
    CvMat m_rotation_y;
    double init_m_rotation_y[9] ={
        (double)cos((val-previous_val_y)/180.0*M_PI), 0, (double)sin((val-previous_val_y)/180.0*M_PI),
        0, 1, 0,
        (double)-sin((val-previous_val_y)/180.0*M_PI), 0, (double)cos((val-previous_val_y)/180.0*M_PI)};

    cvInitMatHeader(&m_rotation_y, 3, 3, CV_64FC1, init_m_rotation_y, CV_AUTOSTEP);
    cvCopy(m_rotation, m_rotation_temp);
    cvMatMulAdd(m_rotation_temp, &m_rotation_y, 0, m_rotation);
    previous_val_y = val;
}
void rad_z_bar(int val)
{
    static int previous_val_z = 180;
    CvMat m_rotation_z;
    double init_m_rotation_z[9] ={
        (double)cos((val-previous_val_z)/180.0*M_PI), (double)-sin((val-previous_val_z)/180.0*M_PI), 0,
        (double)sin((val-previous_val_z)/180.0*M_PI), (double)cos((val-previous_val_z)/180.0*M_PI), 0,
        0, 0, 1};

    cvInitMatHeader(&m_rotation_z, 3, 3, CV_64FC1, init_m_rotation_z);
    cvCopy(m_rotation, m_rotation_temp);
    cvMatMulAdd(m_rotation_temp, &m_rotation_z, 0, m_rotation);
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
