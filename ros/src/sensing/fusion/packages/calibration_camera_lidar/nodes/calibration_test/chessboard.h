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

#ifndef _chessboard_
#define _chessboard_

#include <opencv/cv.h>
#include <opencv2/calib3d/calib3d.hpp>

extern bool findChessboard(IplImage* image, int pat_col, int pat_row, int chess_size, CvMat* object_points, int* corner_count, CvPoint2D32f* corners, CvPoint3D32f* objects);
extern void drawChessboard(IplImage* image, int pat_col, int pat_row, int corner_count, CvPoint2D32f* corners, CvMat* image_points);

#endif
