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

#ifndef PANGOLIN_COMPAT_THREAD_H
#define PANGOLIN_COMPAT_THREAD_H

#include <pangolin/platform.h>

#ifdef CPP11_NO_BOOST
    #include <thread>
#else
#include <boost/thread.hpp>
#include <boost/type_traits.hpp>
#include <unistd.h>

#if BOOST_VERSION >= 104500
#include <boost/chrono/chrono.hpp>

#if BOOST_VERSION < 105000
// Simple implementation of missing sleep_for / sleep_until methods
namespace boost {
    namespace this_thread {
        template <class Rep, class Period>
        void sleep_for(const boost::chrono::duration<Rep, Period>& d)
        {
            boost::chrono::microseconds t = boost::chrono::duration_cast<boost::chrono::microseconds>(d);

            if (t > boost::chrono::microseconds(0)) {
                usleep(t.count());
            }
        }

        template <class Clock, class Duration>
        void sleep_until(const boost::chrono::time_point<Clock, Duration>& t)
        {
            using namespace boost::chrono;
            typedef time_point<Clock, Duration> Time;
            typedef system_clock::time_point SysTime;
            if (t > Clock::now())
            {
                typedef typename boost::common_type<typename Time::duration,
                    typename SysTime::duration>::type D;
                D d = t - Clock::now();
                usleep( duration_cast<microseconds>(d).count() );
            }
        }
    }  // this_thread
} // boost
#endif // BOOST_VERSION < 105000
#endif // BOOST_VERSION >= 104500

#endif // CPP11_NO_BOOST

#include <pangolin/compat/boostd.h>

#endif // PANGOLIN_COMPAT_THREAD_H
