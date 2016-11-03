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

#ifndef PANGOLIN_VARWRAPPER_H
#define PANGOLIN_VARWRAPPER_H

#include <pangolin/var/varvaluegeneric.h>
#include <pangolin/compat/type_traits.h>
#include <pangolin/utils/type_convert.h>

namespace pangolin
{

template<typename T, typename S>
class VarWrapper : public VarValueT<T>
{
public:
    typedef typename boostd::remove_reference<S>::type VarS;

    VarWrapper(VarValueT<S>& src)
        : src(src)
    {
        this->str = src.str;
    }

    const char* TypeId() const
    {
        return typeid(T).name();
    }

    void Reset()
    {
        src.Reset();

        // If reset throws, it will go up to the user, since there is nothing
        // more we can do
        cache = Convert<T,VarS>::Do(src.Get());
    }

    VarMeta& Meta()
    {
        return src.Meta();
    }

    const T& Get() const
    {
        // This might throw, but we can't reset because this is a const method
        cache = Convert<T,VarS>::Do(src.Get());
        return cache;
    }

    void Set(const T& val)
    {
        cache = val;
        try {
            src.Set( Convert<VarS, T>::Do(val) );
        }catch(BadInputException) {
            pango_print_warn("Unable to set variable with type %s from %s. Resetting.", typeid(VarS).name(), typeid(T).name() );
            Reset();
        }
    }

protected:
    mutable T cache;
    VarValueT<S>& src;
};

}

#endif // PANGOLIN_VARWRAPPER_H
