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

#include "chessboard.h"
#include "data_struct.h"

bool findChessboard(IplImage* image, int pat_col, int pat_row, int chess_size, CvMat* object_points, int* corner_count, CvPoint2D32f* corners, CvPoint3D32f* objects) {
//    CvPoint3D32f objects[pat_row*pat_col];
    int found;
    CvSize pattern_size = cvSize(pat_col, pat_row);
    //printf("unkunk:%p\n", corners);
    // Set chessboard paramater
    for (int i = 0; i < pat_row; i++) {
        for (int j = 0; j < pat_col; j++) {
            objects[i * pat_col + j].x = i * chess_size;
            objects[i * pat_col + j].y = j * chess_size;
            objects[i * pat_col + j].z = 0.0;
        }
    }
    cvInitMatHeader(object_points, pat_row*pat_col, 3, CV_32FC1, objects);

    // Find chessboard's corners
    found = cvFindChessboardCorners (image, pattern_size, &corners[0], corner_count);
    if (found) {return true;}
    else {return false;}
}

void drawChessboard(IplImage* image, int pat_col, int pat_row, int corner_count, CvPoint2D32f* corners, CvMat* image_points) {
    IplImage* src_gray;
    CvSize pattern_size = cvSize (pat_col, pat_row);
    int p_count;
    CvMat point_counts;

    // Draw chessbaord's corners on image
    src_gray = cvCreateImage (cvGetSize (image), IPL_DEPTH_8U, 1);
    cvCvtColor (image, src_gray, CV_BGR2GRAY);
    cvFindCornerSubPix (src_gray, &corners[0], corner_count,
                        cvSize (3, 3), cvSize (-1, -1), cvTermCriteria (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));
    cvDrawChessboardCorners (image, pattern_size, &corners[0], corner_count, 1);
    p_count = corner_count;
    cvInitMatHeader (image_points, pat_row*pat_col, 1, CV_32FC2, corners);
    cvInitMatHeader (&point_counts, 1, 1, CV_32SC1, &p_count);
    cvReleaseImage (&src_gray);
}
