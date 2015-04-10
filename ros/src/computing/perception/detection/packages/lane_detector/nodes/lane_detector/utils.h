/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

class ExpMovingAverage {
private:
        double alpha; // [0;1] less = more stable, more = less stable
	double oldValue;
        bool unset;
public:
	ExpMovingAverage()
	    : alpha(0.2), unset(true) {
	}

        void clear() {
                unset = true;
        }

	void add(double value) {
		if (unset) {
			oldValue = value;
                        unset = false;
		}
		double newValue = oldValue + alpha * (value - oldValue);
		oldValue = newValue;
	}

        double get() {
                return oldValue;
        }
};

static inline CvPoint2D32f sub(CvPoint2D32f b, CvPoint2D32f a)
{
	return cvPoint2D32f(b.x-a.x, b.y-a.y);
}

static inline CvPoint2D32f mul(CvPoint2D32f b, CvPoint2D32f a)
{
	return cvPoint2D32f(b.x*a.x, b.y*a.y);
}

static inline CvPoint2D32f add(CvPoint2D32f b, CvPoint2D32f a)
{
	return cvPoint2D32f(b.x+a.x, b.y+a.y);
}

static inline CvPoint2D32f mul(CvPoint2D32f b, float t)
{
	return cvPoint2D32f(b.x*t, b.y*t);
}

static inline float dot(CvPoint2D32f a, CvPoint2D32f b)
{
	return (b.x*a.x + b.y*a.y);
}

static inline float dist(CvPoint2D32f v)
{
	return sqrtf(v.x*v.x + v.y*v.y);
}

static inline CvPoint2D32f point_on_segment(CvPoint2D32f line0, CvPoint2D32f line1,
					    CvPoint2D32f pt)
{
        CvPoint2D32f v = sub(pt, line0);
        CvPoint2D32f dir = sub(line1, line0);
        float len = dist(dir);
        float inv = 1.0f/(len+1e-6f);
        dir.x *= inv;
        dir.y *= inv;

        float t = dot(dir, v);
        if(t >= len) return line1;
        else if(t <= 0) return line0;

        return add(line0, mul(dir,t));
}

static inline float dist2line(CvPoint2D32f line0, CvPoint2D32f line1, CvPoint2D32f pt)
{
        return dist(sub(point_on_segment(line0, line1, pt), pt));
}
