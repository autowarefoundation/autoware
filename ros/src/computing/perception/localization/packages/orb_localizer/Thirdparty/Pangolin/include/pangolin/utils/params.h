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

#ifndef PANGOLIN_PARAMS_H
#define PANGOLIN_PARAMS_H

#include <pangolin/platform.h>
#include <pangolin/utils/type_convert.h>

#include <string>
#include <map>

namespace pangolin
{

class PANGOLIN_EXPORT Params
{
public:
    typedef std::map<std::string,std::string> ParamMap;

    bool Contains(const std::string& key) const
    {
        return params.find(key) != params.end();
    }

    template<typename T>
    T Get(const std::string& key, T default_val) const
    {
        ParamMap::const_iterator v = params.find(key);
        if(v != params.end()) {
            return Convert<T, std::string>::Do(v->second);
        }else{
            return default_val;
        }
    }

    template<typename T>
    void Set(const std::string& key, const T& val)
    {
        params[key] = Convert<std::string,T>::Do(val);
    }

    ParamMap params;
};

}

#endif // PANGOLIN_PARAMS_H
