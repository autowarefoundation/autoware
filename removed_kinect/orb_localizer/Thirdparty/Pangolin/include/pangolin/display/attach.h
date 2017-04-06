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

#ifndef PANGOLIN_ATTACH_H
#define PANGOLIN_ATTACH_H

#include <iostream>
#include <string>
#include <cmath>

namespace pangolin
{

/// Units for measuring screen distances.
enum Unit {
    Fraction,
    Pixel,
    ReversePixel
};

/// Defines absolute or relative position from parent viewport edge.
/// Constructors distinguised by whole pixels, or floating
/// fraction in interval [0,1]
struct PANGOLIN_EXPORT Attach {
    /// Attach to Left/Bottom edge
    Attach() : unit(Fraction), p(0) {}

    /// General constructor
    Attach(Unit unit, GLfloat p) : unit(unit), p(p) {}

    /// Specify relative position in range [0,1].
    /// 0 represents leftmost / bottom-most edge,
    /// 1 represents rightmost / topmost edge
    Attach(GLfloat p) : unit(Fraction), p(p) {
        // Allow for numerical imprecision when checking usage.
        if( p < -1E-3 || 1.001 < p ) {
            std::cerr << "Pangolin API Change: Display::SetBounds must be used with Attach::Pix or Attach::ReversePix to specify pixel bounds relative to an edge. See the code samples for details." << std::endl;
            throw std::exception();
        }
    }

    /// Specify absolute position from leftmost / bottom-most edge.
    static Attach Pix(int p) {
        return Attach(p >=0 ? Pixel : ReversePixel, std::abs((float)p));
    }

    /// Specify absolute position from rightmost / topmost edge.
    static Attach ReversePix(int p) {
        return Attach(ReversePixel, (GLfloat)p);
    }

    /// Specify relative position in range [0,1].
    /// 0 represents leftmost / bottom-most edge,
    /// 1 represents rightmost / topmost edge
    static Attach Frac(float frac) {
        return Attach(frac);
    }

    Unit unit;
    GLfloat p;
};

} // namespace pangolin

#endif // PANGOLIN_ATTACH_H
