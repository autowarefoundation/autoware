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

#ifndef PANGOLIN_TIMER_H
#define PANGOLIN_TIMER_H

#include <pangolin/platform.h>

#ifdef _UNIX_
#include <unistd.h>
#include <sys/time.h>
#endif

#ifdef _WIN_
// TODO: Put this into CPP file so that it doesn't have consequences
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

#include <stdint.h>

namespace pangolin
{

#ifdef _UNIX_
typedef timeval basetime;

inline basetime TimeNow()
{
    basetime t;
    gettimeofday(&t,NULL);
    return t;
}

inline double Time_s(basetime t)
{
    return (double)t.tv_sec + 1E-6 * (double)t.tv_usec;
}

inline int64_t Time_us(basetime t)
{
    return 1000000 * t.tv_sec + t.tv_usec;
}

inline double TimeDiff_s(basetime start, basetime end)
{
    return (double)(end.tv_sec - start.tv_sec) + (double)(end.tv_usec - start.tv_usec) * 1E-6;
}

inline int64_t TimeDiff_us(basetime start, basetime end)
{
    return 1000000*(end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec);
}

inline basetime TimeFromSeconds(double seconds)
{
    basetime t;
    t.tv_sec = (time_t)seconds;
    t.tv_usec = (useconds_t)((seconds - t.tv_sec) * 1E6);
    return t;
}

inline basetime TimeAdd(basetime t1, basetime t2)
{
    basetime t;
    t.tv_sec = t1.tv_sec + t2.tv_sec;
    t.tv_usec = t1.tv_usec + t2.tv_usec;
    if(t.tv_usec >= 1000000 )
    {
        t.tv_usec -= 1000000;
        t.tv_sec += 1;
    }

    return t;
}
#endif

#ifdef _WIN_
typedef LARGE_INTEGER basetime;

inline basetime TimeNow()
{
    basetime t;
    QueryPerformanceCounter(&t);
    return t;
}

inline double Time_s(basetime t)
{
    LARGE_INTEGER f;
    QueryPerformanceFrequency(&f);
    return (double)t.QuadPart / (double)f.QuadPart;
}

inline int64_t Time_us(basetime t)
{
    LARGE_INTEGER f;
    QueryPerformanceFrequency(&f);
    return (1000000*t.QuadPart) / f.QuadPart;
}

inline double TimeDiff_s(basetime start, basetime end)
{
    LARGE_INTEGER f;
    QueryPerformanceFrequency(&f);
    return (double)(end.QuadPart - start.QuadPart) / (double)f.QuadPart;
}

inline int64_t TimeDiff_us(basetime start, basetime end)
{
    LARGE_INTEGER f;
    QueryPerformanceFrequency(&f);
    return 1000000*(end.QuadPart - start.QuadPart) / f.QuadPart;
}

inline basetime TimeFromSeconds(double seconds)
{
    LARGE_INTEGER f;
    QueryPerformanceFrequency(&f);
    basetime t;
    t.QuadPart = (LONGLONG)(seconds * f.QuadPart);
    return t;
}

inline basetime TimeAdd(basetime t1, basetime t2)
{
    basetime t;
    t.QuadPart = t1.QuadPart + t2.QuadPart;
    return t;
}
#endif

inline double TimeNow_s()
{
    return Time_s(TimeNow());
}


inline basetime WaitUntil(basetime t)
{
    // TODO: use smarter sleep!
    basetime currtime = TimeNow();
    while( TimeDiff_s(currtime,t) > 0 )
        currtime = TimeNow();
    return currtime;
}

struct Timer
{
    Timer() {
        Reset();
    }
    
    void Reset()
    {
        start = TimeNow();
    }
    
    double Elapsed_s()
    {
        basetime currtime = TimeNow();
        return TimeDiff_s(start,currtime);
    }
    
    basetime start;
};

}

#endif //PANGOLIN_TIMER_H
