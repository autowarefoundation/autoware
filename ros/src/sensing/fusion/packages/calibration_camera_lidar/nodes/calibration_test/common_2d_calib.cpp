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

#include "common_2d_calib.h"

CvMat *v_g2l = cvCreateMat (1, 3, CV_64FC1);
CvMat *v_g2c = cvCreateMat (1, 3, CV_64FC1);
CvMat *v_rotation = cvCreateMat (1, 3, CV_64FC1);
CvMat *m_rotation = cvCreateMat (3, 3, CV_64FC1);
CvMat *m_intrinsic = cvCreateMat (3, 3, CV_64FC1);
CvMat *m_dist = cvCreateMat (1, 4, CV_64FC1);

void release_common()
{
    cvReleaseMat (&m_dist);
    cvReleaseMat (&m_intrinsic);
    cvReleaseMat (&v_rotation);
    cvReleaseMat (&v_g2l);
    cvReleaseMat (&v_g2c);
    cvReleaseMat (&m_rotation);
}
