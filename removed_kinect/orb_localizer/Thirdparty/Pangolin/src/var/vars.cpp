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

#include <pangolin/var/varextra.h>
#include <pangolin/var/varstate.h>
#include <pangolin/utils/file_utils.h>
#include <pangolin/utils/picojson.h>

#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;

namespace pangolin
{

VarState& VarState::I() {
    static VarState singleton;
    return singleton;
}

VarState::~VarState() {
    Clear();
}

void VarState::Clear() {
    for(VarStoreContainer::iterator i = vars.begin(); i != vars.end(); ++i) {
        delete i->second;
    }
    vars.clear();
    var_adds.clear();
}

void ProcessHistoricCallbacks(NewVarCallbackFn callback, void* data, const std::string& filter)
{
    for (VarState::VarStoreAdditions::const_iterator i = VarState::I().var_adds.begin(); i != VarState::I().var_adds.end(); ++i)
    {
        const std::string& name = *i;
        if (StartsWith(name, filter)) {
            callback(data, name, *VarState::I()[name], false);
        }
    }
}

void RegisterNewVarCallback(NewVarCallbackFn callback, void* data, const std::string& filter)
{
    VarState::I().new_var_callbacks.push_back(NewVarCallback(filter,callback,data));
}

void RegisterGuiVarChangedCallback(GuiVarChangedCallbackFn callback, void* data, const std::string& filter)
{
    VarState::I().gui_var_changed_callbacks.push_back(GuiVarChangedCallback(filter,callback,data));
}

// Find the open brace preceeded by '$'
const char* FirstOpenBrace(const char* str)
{
    bool symbol = false;
    
    for(; *str != '\0'; ++str ) {
        if( *str == '$') {
            symbol = true;
        }else{
            if( symbol ) {
                if( *str == '{' ) {
                    return str;
                } else {
                    symbol = false;
                }
            }
        }
    }
    return 0;
}

// Find the first matching end brace. str includes open brace
const char* MatchingEndBrace(const char* str)
{
    int b = 0;
    for(; *str != '\0'; ++str ) {
        if( *str == '{' ) {
            ++b;
        }else if( *str == '}' ) {
            --b;
            if( b == 0 ) {
                return str;
            }
        }
    }
    return 0;
}

// Recursively expand val
string ProcessVal(const string& val )
{
    string expanded = val;
    
    while(true)
    {
        const char* brace = FirstOpenBrace(expanded.c_str());
        if(brace)
        {
            const char* endbrace = MatchingEndBrace(brace);
            if( endbrace )
            {
                ostringstream oss;
                oss << std::string(expanded.c_str(), brace-1);

                const string inexpand = ProcessVal( std::string(brace+1,endbrace) );
                if( VarState::I().Exists(inexpand) ) {
                    oss << VarState::I()[inexpand]->str->Get();
                }else{
                    pango_print_error("Unabled to expand: [%s].\nMake sure it is defined and terminated with a semi-colon.\n", inexpand.c_str() );
                    oss << "#";
                }

                oss << std::string(endbrace+1, expanded.c_str() + expanded.length() );
                expanded = oss.str();
                continue;
            }
        }
        break;
    }
    
    return expanded;
}

void AddVar(const std::string& name, const string& val )
{
    const std::string full = ProcessVal(val);

    VarValueGeneric*& v = VarState::I()[name];
    if(!v) {
        VarValue<std::string>* nv = new VarValue<std::string>(val);
        InitialiseNewVarMetaGeneric<std::string>(*nv, name);
        v = nv;
    }
    v->str->Set(full);
}

#ifdef ALIAS
void AddAlias(const string& alias, const string& name)
{
    std::map<std::string,_Var*>::iterator vi = vars.find(name);
    
    if( vi != vars.end() )
    {
        _Var * v = vi->second;
        vars[alias].create(v->val,v->val_default,v->type_name);
        vars[alias].Meta().friendly = alias;
        v->generic = false;
        vars[alias].generic = false;
    }else{
        pango_print_error("Variable %s does not exist to alias.\n", name);
    }
}
#endif

void ParseVarsFile(const string& filename)
{
    ifstream f(filename.c_str());
    
    if( f.is_open() )
    {
        while( !f.bad() && !f.eof())
        {
            const int c = f.peek();
            
            if( isspace(c) )
            {
                // ignore leading whitespace
                f.get();
            }else{
                if( c == '#' || c == '%' )
                {
                    // ignore lines starting # or %
                    string comment;
                    getline(f,comment);
                }else{
                    // Otherwise, find name and value, seperated by '=' and ';'
                    string name;
                    string val;
                    getline(f,name,'=');
                    getline(f,val,';');
                    name = Trim(name, " \t\n\r");
                    val = Trim(val, " \t\n\r");
                    
                    if( name.size() >0 && val.size() > 0 )
                    {
                        if( !val.substr(0,1).compare("@") )
                        {
#ifdef ALIAS
                            AddAlias(name,val.substr(1));
#endif
                        }else{
                            AddVar(name,val);
                        }
                    }
                }
            }
        }
        f.close();
    }else{
        cerr << "Unable to open '" << filename << "' for configuration data" << endl;
    }
}

PANGOLIN_EXPORT
void LoadJsonFile(const std::string& filename, const string &prefix)
{
    json::value file_json(json::object_type,true);
    std::ifstream f(filename);
    if(f.is_open()) {
        const std::string err = json::parse(file_json,f);
        if(err.empty()) {
            if(file_json.contains("vars") ) {
                json::value vars = file_json["vars"];
                if(vars.is<json::object>()) {
                    for(json::object::iterator
                        i = vars.get<json::object>().begin();
                        i!= vars.get<json::object>().end();
                        ++i)
                    {
                        const std::string& name = i->first;
                        const std::string& val = i->second.get<std::string>();

                        VarValueGeneric*& v = VarState::I()[name];
                        if(!v) {
                            VarValue<std::string>* nv = new VarValue<std::string>(val);
                            InitialiseNewVarMetaGeneric<std::string>(*nv, name);
                            v = nv;
                        }else{
                            v->str->Set(val);
                        }
                    }
                }
            }
        }else{
            pango_print_error("%s\n", err.c_str());
        }
    }else{
        pango_print_error("Unable to load vars from %s\n", filename.c_str());
    }

}

PANGOLIN_EXPORT
void SaveJsonFile(const std::string& filename, const string &prefix)
{
    json::value vars(json::object_type,true);

    for(VarState::VarStoreAdditions::const_iterator
        i  = VarState::I().var_adds.begin();
        i != VarState::I().var_adds.end();
        ++i)
    {
        const std::string& name = *i;
        if(StartsWith(name,prefix)) {
            try{
                const std::string val = VarState::I()[name]->str->Get();
                vars[name] = val;
            }catch(BadInputException)
            {
                // Ignore things we can't serialise
            }
        }
    }

    json::value file_json(json::object_type,true);
    file_json["pangolin_version"] = PANGOLIN_VERSION_STRING;
    file_json["vars"] = vars;

    std::ofstream f(filename);
    if(f.is_open()) {
        f << file_json.serialize(true);
    }else{
        pango_print_error("Unable to serialise to %s\n", filename.c_str());
    }
}

}
