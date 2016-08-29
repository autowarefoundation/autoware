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

#ifndef PANGOLIN_GLTEXT_H
#define PANGOLIN_GLTEXT_H

#include <pangolin/gl/gl.h>
#include <pangolin/gl/glchar.h>

#include <vector>
#include <string>

namespace pangolin {

class PANGOLIN_EXPORT GlText
{
public:
    GlText();

    GlText(const GlText& txt);

    GlText(const GlTexture& font_tex);
    
    void AddSpace(GLfloat s);

    // Add specified charector to this string.
    void Add(unsigned char c, const GlChar& glc);

    // Clear text
    void Clear();

    // Render without transform in text-centric pixel coordinates
    void Draw() const;
    void DrawGlSl() const;

#ifdef BUILD_PANGOLIN_GUI
    // Render at (x,y,z)' in object coordinates,
    // keeping text size and orientation constant
    void Draw(GLfloat x, GLfloat y, GLfloat z = 0.0f) const;

    // Render at (x,y,z)' in window coordinates.
    void DrawWindow(GLfloat x, GLfloat y, GLfloat z = 0.0f) const;
#endif // BUILD_PANGOLIN_GUI
    
    // Return text that this object signifies.
    const std::string& Text() const {
        return str;
    }
    
    // Return width in pixels of this text.
    GLfloat Width() const {
        return width;
    }

    // Return height in pixels of this text.
    GLfloat Height() const {
        return ymax;
    }
    
    // Return height in pixels of this text, including under baseline
    GLfloat FullHeight() const {
        return ymax - ymin;
    }

//protected:
    const GlTexture* tex;
    std::string str;
    GLfloat width;
    GLfloat ymin;
    GLfloat ymax;
    
    std::vector<XYUV> vs;
};

}

#endif // PANGOLIN_GLTEXT_H
