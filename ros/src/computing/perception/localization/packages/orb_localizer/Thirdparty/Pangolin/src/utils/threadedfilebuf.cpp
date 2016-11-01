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

#include <pangolin/utils/threadedfilebuf.h>

#include <cstring>
#include <stdexcept>

using namespace std;

namespace pangolin
{

threadedfilebuf::threadedfilebuf()
    : mem_buffer(0), mem_size(0), mem_max_size(0), mem_start(0), mem_end(0), should_run(false)
{
}

threadedfilebuf::threadedfilebuf( const std::string& filename, unsigned int buffer_size_bytes )
    : mem_buffer(0), mem_size(0), mem_max_size(0), mem_start(0), mem_end(0), should_run(false)
{
    open(filename, buffer_size_bytes);
}

void threadedfilebuf::open(const std::string& filename, unsigned int buffer_size_bytes)
{
    if (file.is_open()) {
        close();
    }

    file.open(filename.c_str(), ios::out | ios::binary);
    if(!file.is_open()) {
        throw std::runtime_error("Unable to open '" + filename + "' for writing.");
    }

    mem_buffer = 0;
    mem_size = 0;
    mem_start = 0;
    mem_end = 0;
    mem_max_size = buffer_size_bytes;
    mem_buffer = new char[(size_t)mem_max_size];

    should_run = true;
    write_thread = boostd::thread(boostd::ref(*this));
}

void threadedfilebuf::close()
{
    should_run = false;
    cond_queued.notify_all();

    write_thread.join();

    if (mem_buffer) delete mem_buffer;
    file.close();
}

threadedfilebuf::~threadedfilebuf()
{
    close();
}

std::streamsize threadedfilebuf::xsputn(const char* data, std::streamsize num_bytes)
{
    if( num_bytes > mem_max_size ) {
        boostd::unique_lock<boostd::mutex> lock(update_mutex);
        // Wait until queue is empty
        while( mem_size > 0 ) {
            cond_dequeued.wait(lock);
        }

        // Allocate bigger buffer
        delete mem_buffer;
        mem_start = 0;
        mem_end = 0;
        mem_max_size = num_bytes * 4;
        mem_buffer = new char[(size_t)mem_max_size];
    }

    {
        boostd::unique_lock<boostd::mutex> lock(update_mutex);
        
        // wait until there is space to write into buffer
        while( mem_size + num_bytes > mem_max_size ) {
            cond_dequeued.wait(lock);
        }
        
        // add image to end of mem_buffer
        const std::streamsize array_a_size =
                (mem_start <= mem_end) ? (mem_max_size - mem_end) : (mem_start - mem_end);
        
        if( num_bytes <= array_a_size )
        {
            // copy in one
            memcpy(mem_buffer + mem_end, data, (size_t)num_bytes);
            mem_end += num_bytes;
            mem_size += num_bytes;
        }else{
            const std::streamsize array_b_size = num_bytes - array_a_size;
            memcpy(mem_buffer + mem_end, data, (size_t)array_a_size);
            memcpy(mem_buffer, data+array_a_size, (size_t)array_b_size);
            mem_end = array_b_size;
            mem_size += num_bytes;
        }
        
        if(mem_end == mem_max_size)
            mem_end = 0;
    }
    
    cond_queued.notify_one();
    
    return num_bytes;
}

int threadedfilebuf::overflow(int c)
{
    const std::streamsize num_bytes = 1;

    {
        boostd::unique_lock<boostd::mutex> lock(update_mutex);

        // wait until there is space to write into buffer
        while( mem_size + num_bytes > mem_max_size ) {
            cond_dequeued.wait(lock);
        }

        // add image to end of mem_buffer
        mem_buffer[mem_end] = c;
        mem_end += num_bytes;
        mem_size += num_bytes;

        if(mem_end == mem_max_size)
            mem_end = 0;
    }

    cond_queued.notify_one();

    return num_bytes;
}

void threadedfilebuf::operator()()
{
    std::streamsize data_to_write = 0;
    
    while(true)
    {
        {
            boostd::unique_lock<boostd::mutex> lock(update_mutex);
            
            while( mem_size == 0 ) {
                if(!should_run) return;
                cond_queued.wait(lock);
            }

            data_to_write =
                    (mem_start < mem_end) ?
                        mem_end - mem_start :
                        mem_max_size - mem_start;
        }
        
        std::streamsize bytes_written =
                file.sputn(mem_buffer + mem_start, data_to_write );
        
        {
            boostd::unique_lock<boostd::mutex> lock(update_mutex);
            
            mem_size -= bytes_written;
            mem_start += bytes_written;
            
            if(mem_start == mem_max_size)
                mem_start = 0;
        }
        
        cond_dequeued.notify_all();
    }
}

}
