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

#pragma once

#include <vector>
#include <string>

namespace pangolin
{

enum ConsoleLineType
{
    ConsoleLineTypeCmd,
    ConsoleLineTypeCmdOptions,
    ConsoleLineTypeStdout,
    ConsoleLineTypeStderr,
    ConsoleLineTypeOutput,
    ConsoleLineTypeHelp,
};

class ConsoleLine
{
public:
    inline ConsoleLine()
        : linetype(ConsoleLineTypeCmd)
    {
    }

    inline ConsoleLine(std::string text, ConsoleLineType linetype = ConsoleLineTypeOutput)
        : text(text), linetype(linetype)
    {
    }

    std::string text;
    ConsoleLineType linetype;
};

class ConsoleInterpreter
{
public:
    inline virtual ~ConsoleInterpreter()
    {
    }

    virtual void PushCommand(const std::string& cmd) = 0;

    virtual bool PullLine(ConsoleLine& line) = 0;

    virtual std::vector<std::string> Complete(
        const std::string& cmd, int max_options = 20
    ) = 0;

};

}
