#ifndef _image_window_
#define _image_window_

void vector_x_bar(int val);
void vector_y_bar(int val);
void vector_z_bar(int val);
void rad_x_bar(int val);
void rad_y_bar(int val);
void rad_z_bar(int val);

extern void set_rotation(CvMat* m_rotation);
extern void set_g2c(CvMat* v_g2c);
extern void release_image_window();

#endif
