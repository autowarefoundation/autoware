/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2014 Steven Lovegrove
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

#ifndef PANGOLIN_GLPANGOGLU_H
#define PANGOLIN_GLPANGOGLU_H

#include <pangolin/gl/glplatform.h>

namespace pangolin {

/// Clone of gluProject
PANGOLIN_EXPORT
const GLubyte* glErrorString(GLenum error);

/// Clone of gluProject
PANGOLIN_EXPORT
GLint glProject(
    float objx, float objy, float objz,
    const float modelMatrix[16],
    const float projMatrix[16],
    const GLint viewport[4],
    float* winx, float* winy, float* winz
);


/// Clone of gluUnProject
PANGOLIN_EXPORT
GLint glUnProject(
    float winx, float winy, float winz,
    const float modelMatrix[16],
    const float projMatrix[16],
    const GLint viewport[4],
    float* objx, float* objy, float* objz
);

/// Clone of gluProject
PANGOLIN_EXPORT
GLint glProject(
    double objx, double objy, double objz,
    const double modelMatrix[16],
    const double projMatrix[16],
    const GLint viewport[4],
    double* winx, double* winy, double* winz
);


/// Clone of gluUnProject
PANGOLIN_EXPORT
GLint glUnProject(
    double winx, double winy, double winz,
    const double modelMatrix[16],
    const double projMatrix[16],
    const GLint viewport[4],
    double* objx, double* objy, double* objz
);

}

#endif // PANGOLIN_GLPANGOGLU_H
