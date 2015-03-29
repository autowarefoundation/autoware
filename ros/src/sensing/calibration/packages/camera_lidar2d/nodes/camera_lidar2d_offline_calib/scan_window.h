#ifndef _scan_window_
#define _scan_window_

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include "data_struct.h"

extern void plot_vertical_line(IplImage* image, int line_division_num, int window_width, int window_height);
extern void plot_horizontal_line(IplImage* image, int window_width, int window_height);
extern CvPoint get_center_pt(int window_width, Three_dimensional_vector* scan, int scale);
extern void plot_center_pt_line(IplImage *image, CvPoint center_pt, int chess_size, int pat_col, int margin, int window_width, int window_height, int scale);
extern void plot_string(IplImage* image, const char* text, int thickness, int x, int y, CvScalar color);
extern void plot_string_on_buttun(IplImage* image, const char* text, int thickness, int x, int y, bool on_mouse);
extern void plot_scan_image(IplImage* image, Two_dimensional_vector* scan_image);

#endif
