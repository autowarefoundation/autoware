/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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
