#ifndef _chessboard_
#define _chessboard_

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>

extern bool findChessboard(IplImage* image, int pat_col, int pat_row, int chess_size, CvMat* object_points, int* corner_count, CvPoint2D32f* corners, CvPoint3D32f* objects);
extern void drawChessboard(IplImage* image, int pat_col, int pat_row, int corner_count, CvPoint2D32f* corners, CvMat* image_points);

#endif
