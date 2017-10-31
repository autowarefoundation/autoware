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

#ifndef PANGOLIN_DISPLAY_GLUT_H
#define PANGOLIN_DISPLAY_GLUT_H

#include <pangolin/platform.h>
#include <pangolin/gl/glinclude.h>

#include <string>

#ifndef GLUT_DOUBLE
// Avoid pulling in all of glut just for these defines.
#define  GLUT_RGB         0x0000
#define  GLUT_RGBA        0x0000
#define  GLUT_INDEX       0x0001
#define  GLUT_SINGLE      0x0000
#define  GLUT_DOUBLE      0x0002
#define  GLUT_ACCUM       0x0004
#define  GLUT_ALPHA       0x0008
#define  GLUT_DEPTH       0x0010
#define  GLUT_STENCIL     0x0020
#define  GLUT_MULTISAMPLE 0x0080
#define  GLUT_STEREO      0x0100
#define  GLUT_LUMINANCE   0x0200
#endif

namespace pangolin
{

/// Create GLUT window and bind Pangolin to it.
/// All GLUT initialisation is taken care of. This prevents you
/// from needing to call BindToContext() and TakeGlutCallbacks().
PANGOLIN_EXPORT
void CreateGlutWindowAndBind(std::string window_title, int w = 640, int h = 480, unsigned int mode = GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH );

/// Applies any post-render events if they are defined,
/// swaps buffers and processes events. Also resets viewport to
/// entire window and disables scissoring.
PANGOLIN_EXPORT
void FinishGlutFrame();

/// Swaps OpenGL Buffers and processes input events.
PANGOLIN_EXPORT
void SwapGlutBuffersProcessGlutEvents();

/// Allow Pangolin to take GLUT callbacks for its own uses.
/// Not needed if you instantiated a window through CreateWindowAndBind().
PANGOLIN_EXPORT
void TakeGlutCallbacks();

}

#endif // PANGOLIN_DISPLAY_GLUT_H
