/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2013 Steven Lovegrove
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

#ifndef PANGOLIN_COMPAT_GLCONSOLE_H
#define PANGOLIN_COMPAT_GLCONSOLE_H

#include <pangolin/platform.h>

// If we don't have GLUT, coerce GLConsole to work with our own font rendering.
#ifdef HAVE_GLES
#include <pangolin/glinclude.h>
#include <pangolin/glfont.h>

// Define our own GLFont class, preventing GLConsoles from being included.
#define __GL_FONT_H__

class GLFont
{
public:
    void glPrintf(int x, int y, const char *fmt, ...)
    {
        glPushMatrix();
        glTranslatef(x,y,1.0f);
        pangolin::GlFont::I().Text(fmt).Draw();
        glPopMatrix();
    }
    void glPrintf(int x, int y, const std::string fmt, ...){ glPrintf(x,y, fmt.c_str()); }
    void glPrintfFast(int x, int y, const char *fmt, ...) { glPrintf(x,y,fmt); }
    void glPrintfFast(int x, int y, const std::string fmt, ...){ glPrintfFast(x,y, fmt.c_str()); }
    unsigned int CharHeight() { return 10; }
    unsigned int CharWidth() { return 10; }
};

// TODO: We should implement these or something...
#define glPushAttrib(x)
#define glPopAttrib(x)
#define glutGetModifiers(x) (0)

#define GLUT_ACTIVE_SHIFT 0
#define GLUT_ACTIVE_CTRL 1
#define GLUT_ACTIVE_ALT 2
#define GLUT_KEY_UP 3
#define GLUT_KEY_DOWN 4
#define GLUT_KEY_LEFT 5
#define GLUT_KEY_RIGHT 6
#define GLUT_KEY_PAGE_UP 7
#define GLUT_KEY_PAGE_DOWN 8
#define GLUT_KEY_HOME 9
#define GLUT_KEY_END 10

#endif // HAVE_GLES

#include <GLConsole/GLConsole.h>

#endif // PANGOLIN_COMPAT_GLCONSOLE_H
