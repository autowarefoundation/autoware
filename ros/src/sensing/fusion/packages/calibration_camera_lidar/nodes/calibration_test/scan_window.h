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

#ifndef _scan_window_
#define _scan_window_

#include <opencv/cv.h>
#include "data_struct.h"

extern void plot_vertical_line(IplImage* image, int line_division_num, int window_width, int window_height);
extern void plot_horizontal_line(IplImage* image, int window_width, int window_height);
extern CvPoint get_center_pt(int window_width, Three_dimensional_vector* scan, int scale);
extern void plot_center_pt_line(IplImage *image, CvPoint center_pt, int chess_size, int pat_col, int margin, int window_width, int window_height, int scale);
extern void plot_string(IplImage* image, const char* text, int thickness, int x, int y, CvScalar color);
extern void plot_string_on_buttun(IplImage* image, const char* text, int thickness, int x, int y, bool on_mouse);
extern void plot_scan_image(IplImage* image, Two_dimensional_vector* scan_image);

#endif
