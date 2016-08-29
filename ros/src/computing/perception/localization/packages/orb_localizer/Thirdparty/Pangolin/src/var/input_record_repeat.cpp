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

#include <pangolin/var/input_record_repeat.h>

#include <limits>

using namespace std;

namespace pangolin
{

InputRecordRepeat::InputRecordRepeat(const std::string& var_record_prefix)
    : record(false), play(false), index(-1)
{
    RegisterGuiVarChangedCallback(&InputRecordRepeat::GuiVarChanged,(void*)this,var_record_prefix);
}

InputRecordRepeat::~InputRecordRepeat()
{

}

void InputRecordRepeat::SetIndex(int id)
{
//    if( id < index )
//        Clear();

    index = id;

    while( !play_queue.empty() && play_queue.front().index < index )
    {
        // 'Play' Frameinput
        FrameInput in = play_queue.front();
        play_queue.pop_front();
        Var<std::string> var(in.var);
        var = in.val;
    }
}

void InputRecordRepeat::Record()
{
    ClearBuffer();
    play = false;
    record = true;
}

void InputRecordRepeat::Stop()
{
    record = false;
    play = false;
}

void InputRecordRepeat::ClearBuffer()
{
    index = -1;
    record_queue.clear();
    play_queue.clear();
}

ostream& operator<<(ostream& os, const FrameInput& fi )
{
    os << fi.index << endl << fi.var << endl << fi.val << endl;
    return os;
}

istream& operator>>(istream& is, FrameInput& fi)
{
    is >> fi.index;
    is.ignore(std::numeric_limits<streamsize>::max(),'\n');
    getline(is,fi.var);
    getline(is,fi.val);
    return is;
}

void InputRecordRepeat::SaveBuffer(const std::string& filename)
{
    ofstream f(filename.c_str());

    for( std::list<FrameInput>::const_iterator i = record_queue.begin(); i!=record_queue.end(); ++i )
    {
        f << *i;
    }
}

void InputRecordRepeat::LoadBuffer(const std::string& filename)
{
    record_queue.clear();

    ifstream f(filename.c_str());
    while(f.good())
    {
        FrameInput fi;
        f >> fi;
        if( f.good() )
            record_queue.push_back(fi);
    }
}

void InputRecordRepeat::PlayBuffer()
{
    play_queue = record_queue;

    record = false;
    play = true;
}

void InputRecordRepeat::PlayBuffer(size_t start, size_t end)
{
    std::list<FrameInput>::iterator s = record_queue.begin();
    std::list<FrameInput>::iterator e = record_queue.begin();

    for(size_t i=0; i<start; i++) s++;
    for(size_t i=0; i<end; i++) e++;

    play_queue.clear();
    play_queue.insert(play_queue.begin(),s,e);

    record = false;
    play = true;
}

size_t InputRecordRepeat::Size()
{
    return record_queue.size();
}

void InputRecordRepeat::UpdateVariable(const std::string& name )
{
    Var<std::string> var(name);

    if( record )
    {
        FrameInput input;
        input.index = index;
        input.var = name;
        input.val = var.Get();
        record_queue.push_back(input);
    }
}

void InputRecordRepeat::GuiVarChanged(void* data, const std::string& name, VarValueGeneric &_var)
{
    InputRecordRepeat* thisptr = (InputRecordRepeat*)data;

    if( thisptr->record )
    {
        Var<std::string> var(_var);

        FrameInput input;
        input.index = thisptr->index;
        input.var = name;
        input.val = var.Get();

        thisptr->record_queue.push_back(input);
    }
}

}
