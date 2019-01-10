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

#ifndef _common_2d_common_
#define _common_2d_common_

#include <opencv/cv.h>

extern CvMat *v_g2l;
extern CvMat *v_g2c;
extern CvMat *v_rotation;
extern CvMat *m_rotation;
extern CvMat *m_intrinsic;
extern CvMat *m_dist;

extern void release_common();
#endif
