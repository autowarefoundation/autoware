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

#ifndef PANGOLIN_VARVALUE_H
#define PANGOLIN_VARVALUE_H

#include <pangolin/var/varvaluet.h>
#include <pangolin/var/varwrapper.h>

namespace pangolin
{

template<typename T>
class VarValue : public VarValueT<typename boostd::remove_reference<T>::type>
{
public:
    typedef typename boostd::remove_reference<T>::type VarT;

    ~VarValue()
    {
        delete str_ptr;
    }

    VarValue()
    {
        Init();
    }

    VarValue(const T& value)
        : value(value), default_value(value)
    {
        Init();
    }

    VarValue(const T& value, const VarT& default_value)
        : value(value), default_value(default_value)
    {
        Init();
    }

    const char* TypeId() const
    {
        return typeid(VarT).name();
    }

    void Reset()
    {
        value = default_value;
    }

    VarMeta& Meta()
    {
        return meta;
    }

    const VarT& Get() const
    {
        return value;
    }

    VarT& Get()
    {
        return value;
    }

    void Set(const VarT& val)
    {
        value = val;
    }

protected:
    void Init()
    {
        if(boostd::is_same<VarT,std::string>::value) {
            str_ptr = 0;
            this->str = (VarValueT<std::string>*)this;
        }else{
            str_ptr = new VarWrapper<std::string,VarT>(*this);
            this->str = str_ptr;
        }
    }

    // If non-zero, this class owns this str pointer in the base-class.
    VarValueT<std::string>* str_ptr;

    T value;
    VarT default_value;
    VarMeta meta;
};

}

#endif // PANGOLIN_VARVALUE_H
