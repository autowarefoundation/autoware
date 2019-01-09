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

#ifndef _trans_
#define _trans_

#include "data_struct.h"
#include <opencv/cv.h>

extern Two_dimensional_vector trans_scan2scan_image(Three_dimensional_vector* scan, CvMat* v_g2l, CvMat* v_g2c, CvMat* m_rotation, CvMat* m_intrinsic);

#endif
