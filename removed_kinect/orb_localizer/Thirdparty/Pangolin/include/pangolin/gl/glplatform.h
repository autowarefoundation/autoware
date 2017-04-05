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

#ifndef PANGOLIN_GLPLATFORM_H
#define PANGOLIN_GLPLATFORM_H

//////////////////////////////////////////////////////////
// Attempt to portably include Necessary OpenGL headers
//////////////////////////////////////////////////////////

#include <pangolin/platform.h>

#ifdef _WIN_
    // Define maths quantities when using <cmath> to match posix systems
    #define _USE_MATH_DEFINES

    // Don't define min / max macros in windows.h or other unnecessary macros
    #define NOMINMAX
    #define WIN32_LEAN_AND_MEAN
    #include <Windows.h>

    // Undef nuisance Windows.h macros which interfere with our methods
    #undef LoadImage
    #undef near
    #undef far
#endif

#ifdef HAVE_GLEW
    #include <GL/glew.h>
#endif

#ifdef HAVE_GLES
    #if defined(_ANDROID_)
        #include <EGL/egl.h>
        #ifdef HAVE_GLES_2
            #include <GLES2/gl2.h>
            #include <GLES2/gl2ext.h>
        #else
            #include <GLES/gl.h>
            #define GL_GLEXT_PROTOTYPES
            #include <GLES/glext.h>
        #endif
    #elif defined(_APPLE_IOS_)
        #include <OpenGLES/ES2/gl.h>
        #include <OpenGLES/ES2/glext.h>
    #endif
#else
    #ifdef _OSX_
        #include <OpenGL/gl.h>
    #else
        #include <GL/gl.h>
    #endif
#endif // HAVE_GLES

#include <pangolin/gl/glpangoglu.h>

#endif // PANGOLIN_GLPLATFORM_H
