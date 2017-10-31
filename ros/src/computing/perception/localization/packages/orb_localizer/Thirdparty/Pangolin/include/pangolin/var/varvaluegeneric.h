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

#ifndef PANGOLIN_VARVALUEGENERIC_H
#define PANGOLIN_VARVALUEGENERIC_H

#include <string>

namespace pangolin
{

struct VarMeta
{
    std::string full_name;
    std::string friendly;
    double range[2];
    double increment;
    int flags;
    bool gui_changed;
    bool logscale;
    bool generic;
};

// Forward declaration
template<typename T>
class VarValueT;

//! Abstract base class for named Pangolin variables
class VarValueGeneric
{
public:
    VarValueGeneric()
        : str(0)
    {
    }

    virtual ~VarValueGeneric()
    {
    }

    virtual const char* TypeId() const = 0;
    virtual void Reset() = 0;
    virtual VarMeta& Meta() = 0;

//protected:
    // String serialisation object.
    VarValueT<std::string>* str;
};

}

#endif // PANGOLIN_VARVALUEGENERIC_H
