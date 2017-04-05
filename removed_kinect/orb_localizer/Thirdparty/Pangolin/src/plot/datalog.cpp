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

#include <pangolin/plot/datalog.h>

#include <limits>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <algorithm>
#include <iostream>

namespace pangolin
{

void DataLogBlock::AddSamples(size_t num_samples, size_t dimensions, const float* data_dim_major )
{
    if(nextBlock) {
        // If next block exists, add to it instead
        nextBlock->AddSamples(num_samples, dimensions, data_dim_major);
    }else{
        if(dimensions > dim) {
            // If dimensions is too high for this block, start a new bigger one
            nextBlock = new DataLogBlock(dimensions, max_samples, start_id + samples);
        }else{
            // Try to copy samples to this block
            const size_t samples_to_copy = std::min(num_samples, SampleSpaceLeft());

            if(dimensions == dim) {
                // Copy entire block all together
                std::copy(data_dim_major, data_dim_major + samples_to_copy*dim, sample_buffer+samples*dim);
                samples += samples_to_copy;
                data_dim_major += samples_to_copy*dim;
            }else{
                // Copy sample at a time, filling with NaN's where needed.
                float* dst = sample_buffer;
                for(size_t i=0; i< samples_to_copy; ++i) {
                    std::copy(data_dim_major, data_dim_major + dimensions, dst);
                    for(size_t ii = dimensions; ii < dim; ++ii) {
                        dst[ii] = std::numeric_limits<float>::quiet_NaN();
                    }
                    dst += dimensions;
                    data_dim_major += dimensions;
                }
                samples += samples_to_copy;
            }

            // Copy remaining data to next block (this one is full)
            if(samples_to_copy < num_samples) {
                nextBlock = new DataLogBlock(dim, max_samples, start_id + Samples());
                nextBlock->AddSamples(num_samples-samples_to_copy, dimensions, data_dim_major);
            }
        }
    }
}

DataLog::DataLog(unsigned int buffer_size)
    : block_samples_alloc(buffer_size), block0(0), blockn(0), record_stats(true)
{
}

DataLog::~DataLog()
{
    Clear();
}

void DataLog::SetLabels(const std::vector<std::string> & new_labels)
{
    // Create new labels if needed
    for( unsigned int i= labels.size(); i < new_labels.size(); ++i )
        labels.push_back( std::string() );

    // Add data to existing plots
    for( unsigned int i=0; i<labels.size(); ++i )
        labels[i] = new_labels[i];
}

const std::vector<std::string>& DataLog::Labels() const
{
    return labels;
}

void DataLog::Log(unsigned int dimension, const float* vals, unsigned int samples )
{
    if(!block0) {
        // Create first block
        block0 = new DataLogBlock(dimension, block_samples_alloc, 0);
        blockn = block0;
    }

    if(record_stats) {
        while(stats.size() < dimension) {
            stats.push_back( DimensionStats() );
        }
        for(unsigned int d=0; d<dimension; ++d) {
            DimensionStats& ds = stats[d];
            for(unsigned int s=0; s<samples; ++s) {
                const float v = vals[s*dimension+d];
                ds.isMonotonic = ds.isMonotonic && (v >= ds.max);
                ds.sum += v;
                ds.sum_sq += v*v;
                ds.min = std::min(ds.min, v);
                ds.max = std::max(ds.max, v);
            }
        }
    }

    blockn->AddSamples(samples,dimension,vals);

    // Update pointer to most recent block.
    while(blockn->NextBlock()) {
        blockn = blockn->NextBlock();
    }
}

void DataLog::Log(float v)
{
    const float vs[] = {v};
    Log(1,vs);
}

void DataLog::Log(float v1, float v2)
{
    const float vs[] = {v1,v2};
    Log(2,vs);
}

void DataLog::Log(float v1, float v2, float v3)
{
    const float vs[] = {v1,v2,v3};
    Log(3,vs);
}
void DataLog::Log(float v1, float v2, float v3, float v4)
{
    const float vs[] = {v1,v2,v3,v4};
    Log(4,vs);
}
void DataLog::Log(float v1, float v2, float v3, float v4, float v5)
{
    const float vs[] = {v1,v2,v3,v4,v5};
    Log(5,vs);
}
void DataLog::Log(float v1, float v2, float v3, float v4, float v5, float v6)
{
    const float vs[] = {v1,v2,v3,v4,v5,v6};
    Log(6,vs);
}

void DataLog::Log(float v1, float v2, float v3, float v4, float v5, float v6, float v7)
{
    const float vs[] = {v1,v2,v3,v4,v5,v6,v7};
    Log(7,vs);
}

void DataLog::Log(float v1, float v2, float v3, float v4, float v5, float v6, float v7, float v8)
{
    const float vs[] = {v1,v2,v3,v4,v5,v6,v7,v8};
    Log(8,vs);
}

void DataLog::Log(float v1, float v2, float v3, float v4, float v5, float v6, float v7, float v8, float v9)
{
    const float vs[] = {v1,v2,v3,v4,v5,v6,v7,v8,v9};
    Log(9,vs);
}

void DataLog::Log(float v1, float v2, float v3, float v4, float v5, float v6, float v7, float v8, float v9, float v10)
{
    const float vs[] = {v1,v2,v3,v4,v5,v6,v7,v8,v9,v10};
    Log(10,vs);
}

void DataLog::Log(const std::vector<float> & vals)
{
    Log(vals.size(), &vals[0]);
}

void DataLog::Clear()
{
    if(block0) {
        block0->ClearLinked();
        blockn = block0;
    }
    stats.clear();
}

void DataLog::Save(std::string filename)
{
    // TODO: Implement
    throw std::runtime_error("Method not implemented");
}

const DataLogBlock* DataLog::FirstBlock() const
{
    return block0;
}

const DataLogBlock* DataLog::LastBlock() const
{
    return blockn;
}

const DimensionStats& DataLog::Stats(size_t dim) const
{
    return stats[dim];
}

unsigned int DataLog::Samples() const
{
    if(blockn) {
        return blockn->StartId() + blockn->Samples();
    }
    return 0;
}

const float* DataLog::Sample(int n) const
{
    if(block0) {
        return block0->Sample(n);
    }else{
        return 0;
    }
}

}
