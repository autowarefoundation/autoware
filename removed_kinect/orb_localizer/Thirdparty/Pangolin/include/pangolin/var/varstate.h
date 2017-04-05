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

#ifndef PANGOLIN_VARSTATE_H
#define PANGOLIN_VARSTATE_H

#include <map>
#include <vector>
#include <pangolin/platform.h>
#include <pangolin/var/varvalue.h>
#include <pangolin/utils/file_utils.h>

namespace pangolin
{

typedef void (*NewVarCallbackFn)(void* data, const std::string& name, VarValueGeneric& var, bool brand_new);
typedef void (*GuiVarChangedCallbackFn)(void* data, const std::string& name, VarValueGeneric& var);

struct PANGOLIN_EXPORT NewVarCallback
{
    NewVarCallback(const std::string& filter, NewVarCallbackFn fn, void* data)
        :filter(filter),fn(fn),data(data) {}
    std::string filter;
    NewVarCallbackFn fn;
    void* data;
};

struct PANGOLIN_EXPORT GuiVarChangedCallback
{
    GuiVarChangedCallback(const std::string& filter, GuiVarChangedCallbackFn fn, void* data)
        :filter(filter),fn(fn),data(data) {}
    std::string filter;
    GuiVarChangedCallbackFn fn;
    void* data;
};

class PANGOLIN_EXPORT VarState
{
public:
    static VarState& I();

    ~VarState();

    void Clear();

    template<typename T>
    void NotifyNewVar(const std::string& name, VarValue<T>& var )
    {
        var_adds.push_back(name);

        // notify those watching new variables
        for(std::vector<NewVarCallback>::iterator invc = new_var_callbacks.begin(); invc != new_var_callbacks.end(); ++invc) {
            if( StartsWith(name,invc->filter) ) {
               invc->fn( invc->data, name, var, true);
            }
        }
    }

    VarValueGeneric*& operator[](const std::string& str)
    {
        return vars[str];
    }

    bool Exists(const std::string& str) const
    {
        return vars.find(str) != vars.end();
    }

//protected:
    typedef std::map<std::string, VarValueGeneric*> VarStoreContainer;
    typedef std::vector<std::string> VarStoreAdditions;

    VarStoreContainer vars;
    VarStoreAdditions var_adds;

    std::vector<NewVarCallback> new_var_callbacks;
    std::vector<GuiVarChangedCallback> gui_var_changed_callbacks;
};

}

#endif // PANGOLIN_VARSTATE_H
