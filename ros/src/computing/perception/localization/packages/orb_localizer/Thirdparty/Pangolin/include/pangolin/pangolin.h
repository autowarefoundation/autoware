/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2011 Steven Lovegrove
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef PANGOLIN_H
#define PANGOLIN_H

#include <pangolin/platform.h>

#ifdef BUILD_PANGOLIN_GUI
  #include <pangolin/gl/gl.h>
  #include <pangolin/gl/gldraw.h>
  #include <pangolin/gl/glvbo.h>
  #include <pangolin/gl/glstate.h>
  #include <pangolin/gl/colour.h>
  #include <pangolin/display/display.h>
  #include <pangolin/display/view.h>
  #ifdef HAVE_GLUT
    #include <pangolin/display/device/display_glut.h>
  #endif // HAVE_GLUT
  #ifdef _ANDROID_
    #include <pangolin/display/device/display_android.h>
  #endif
  #if !defined(HAVE_GLES) || defined(HAVE_GLES_2)
    #include <pangolin/plot/plotter.h>
  #endif
#endif // BUILD_PANGOLIN_GUI

#ifdef BUILD_PANGOLIN_VARS
  #include <pangolin/var/varextra.h>
  #ifdef BUILD_PANGOLIN_GUI
    #include <pangolin/display/widgets/widgets.h>
  #endif // BUILD_PANGOLIN_GUI
#endif // BUILD_PANGOLIN_VARS

#ifdef BUILD_PANGOLIN_VIDEO
  #include <pangolin/video/video.h>
  #include <pangolin/video/video_output.h>
#endif // BUILD_PANGOLIN_VIDEO

#include <pangolin/image/image_io.h>

// Let other libraries headers know about Pangolin
#define HAVE_PANGOLIN

#endif // PANGOLIN_H

