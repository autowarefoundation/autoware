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

#include "scan_window.h"

void plot_vertical_line(IplImage* image, int line_division_num, int window_width, int window_height)
{
    CvPoint line_start, line_end;
    for (int i = 0; i < line_division_num; i++) {
        line_start.x = 0;
        line_start.y = window_height / line_division_num * (i + 1);
        line_end.x = window_width;
        line_end.y = window_height / line_division_num * (i + 1);

        cvLine(image, line_start, line_end, CV_RGB (120, 120, 120), 1, 8, 0);
    }
}

void plot_horizontal_line(IplImage* image, int window_width, int window_height)
{
    CvPoint line_start, line_end;
    line_start.x = window_width / 2;
    line_start.y = 0;
    line_end.x = window_width /2;
    line_end.y = window_height;
    cvLine(image, line_start, line_end, CV_RGB (120, 120, 120), 1, 8, 0);
}

void plot_center_pt_line(IplImage *image, CvPoint center_pt, int chess_size, int pat_col, int margin, int window_width, int window_height, int scale)
{
    CvPoint line_start, line_end;
    /* chessboard horizontal line */
    line_start.x = window_width / 2 - ((chess_size / 1000) * (pat_col + 1) + (margin /1000)) * scale / 2;
    line_start.y = center_pt.y;
    line_end.x = window_width /2 + ((chess_size / 1000) * (pat_col + 1) + (margin /1000)) * scale / 2;
    line_end.y = center_pt.y;
    cvLine(image, line_start, line_end, CV_RGB (255, 255, 0), 1, 8, 0);

    /* chessboard vertical line */
    // left line
    line_start.x = window_width/2 - ((chess_size/1000.0) * (pat_col+1) + (margin/1000.0)) * scale / 2;
    line_start.y = 0;
    line_end.x = window_width/2 - ((chess_size/1000.0) * (pat_col+1) + (margin/1000.0)) * scale / 2;
    line_end.y = window_height;
    cvLine(image, line_start, line_end, CV_RGB (255, 255, 0), 1, 8, 0);
    // right line
    line_start.x = window_width/2 + (((float)chess_size/1000.0) * (pat_col+1) + ((float)margin/1000.0)) * scale / 2;
    line_start.y = 0;
    line_end.x = window_width/2 + (((float)chess_size/1000.0) * (pat_col+1) + ((float)margin/1000.0)) * scale / 2;
    line_end.y = window_height;
    cvLine(image, line_start, line_end, CV_RGB (255, 255, 0), 1, 8, 0);
}


CvPoint get_center_pt(int window_width, Three_dimensional_vector* scan, int scale)
{
    CvPoint center_pt;
    center_pt.x = scan->x[scan->x.size() / 2] * scale + (window_width / 2);
    center_pt.y = scan->z[scan->x.size() / 2] * scale;
    return center_pt;
}

void plot_string(IplImage* image, const char* text, int thickness, int x, int y, CvScalar color)
{
    CvFont dfont;
    float hscale      = 1.0f;
    float vscale      = 1.0f;
    float italicscale = 0.0f;

    cvInitFont (&dfont, CV_FONT_HERSHEY_SIMPLEX , hscale, vscale, italicscale, thickness, CV_AA);
    cvPutText(image, text, cvPoint(x, y), &dfont, color);
}

void plot_string_on_buttun(IplImage* image, const char* text, int thickness, int x, int y, bool on_mouse)
{
    if (on_mouse)
        plot_string(image, text, thickness, x, y, CV_RGB(250, 250, 250));
    else
        plot_string(image, text, thickness, x, y, CV_RGB(150, 150, 150));
}

void plot_scan_image(IplImage* image, Two_dimensional_vector* scan_image)
{
    CvSeq *points;
    CvPoint pt;
    CvMemStorage *storage = cvCreateMemStorage (0);

    points = cvCreateSeq (CV_SEQ_ELTYPE_POINT, sizeof (CvSeq), sizeof (CvPoint), storage);

    for (int i = 0; i < (int)scan_image->x.size(); i++) {
        if(0 > scan_image->x[i] || scan_image->x[i] > 639) {
            continue;
        }
        if(0 > scan_image->y[i] || scan_image->y[i] > 479) {
            continue;
        }
        pt.x = scan_image->x[i];
        pt.y = scan_image->y[i];

        cvSeqPush (points, &pt);
        cvCircle(image, pt, 2, CV_RGB (0, 255, 0), CV_FILLED, 8, 0);
    }

    cvClearSeq(points);
    cvReleaseMemStorage(&storage);
}
