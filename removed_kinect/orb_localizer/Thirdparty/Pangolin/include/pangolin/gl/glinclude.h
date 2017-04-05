/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2011 Steven Lovegrove, Richard Newcombe
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

#ifndef PANGOLIN_GLINCLUDE_H
#define PANGOLIN_GLINCLUDE_H

#include <pangolin/gl/glplatform.h>

#ifdef HAVE_GLES
#include <pangolin/gl/gl_es_compat.h>
#endif

#define CheckGlDieOnError() pangolin::_CheckGlDieOnError( __FILE__, __LINE__ );
namespace pangolin {
inline void _CheckGlDieOnError( const char *sFile, const int nLine )
{
    GLenum glError = glGetError();
    if( glError != GL_NO_ERROR ) {
        pango_print_error( "OpenGL Error: %s (%d)\n", glErrorString(glError), glError );
		pango_print_error("In: %s, line %d\n", sFile, nLine);
    }
}
}

#endif // PANGOLIN_GLINCLUDE_H
