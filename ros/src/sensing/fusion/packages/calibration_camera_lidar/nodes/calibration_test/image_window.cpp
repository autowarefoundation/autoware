/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
