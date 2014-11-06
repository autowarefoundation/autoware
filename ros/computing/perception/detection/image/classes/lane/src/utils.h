#pragma once

#include <list>

class ExpMovingAverage {
private:
        double alpha; // [0;1] less = more stable, more = less stable
    double oldValue;
        bool unset;
public:
    ExpMovingAverage() {
        this->alpha = 0.2;
                unset = true;
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

CvPoint2D32f sub(CvPoint2D32f b, CvPoint2D32f a) { return cvPoint2D32f(b.x-a.x, b.y-a.y); }
CvPoint2D32f mul(CvPoint2D32f b, CvPoint2D32f a) { return cvPoint2D32f(b.x*a.x, b.y*a.y); }
CvPoint2D32f add(CvPoint2D32f b, CvPoint2D32f a) { return cvPoint2D32f(b.x+a.x, b.y+a.y); }
CvPoint2D32f mul(CvPoint2D32f b, float t) { return cvPoint2D32f(b.x*t, b.y*t); }
float dot(CvPoint2D32f a, CvPoint2D32f b) { return (b.x*a.x + b.y*a.y); }
float dist(CvPoint2D32f v) { return sqrtf(v.x*v.x + v.y*v.y); }

CvPoint2D32f point_on_segment(CvPoint2D32f line0, CvPoint2D32f line1, CvPoint2D32f pt){
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

float dist2line(CvPoint2D32f line0, CvPoint2D32f line1, CvPoint2D32f pt){
        return dist(sub(point_on_segment(line0, line1, pt), pt));
}
