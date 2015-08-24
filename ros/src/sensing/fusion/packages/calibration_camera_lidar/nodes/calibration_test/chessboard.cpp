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
