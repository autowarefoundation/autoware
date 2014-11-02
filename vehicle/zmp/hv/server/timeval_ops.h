#ifndef __TIMEVAL_OPS_H__
#define __TIMEVAL_OPS_H__

#define USEC_1SEC        1000000
#define USEC_1MSEC        1000
#define MSEC_1SEC        1000

// t = 0
static inline void timeval_clear(struct timeval *t)
{
    t->tv_sec = t->tv_usec = 0;
}

// x = y
static inline void timeval_copy(struct timeval *x, struct timeval *y)
{
    x->tv_sec = y->tv_sec;
    x->tv_usec = y->tv_usec = 0;
}

// transform from struct gdev_time to microseconds
static inline unsigned long timeval_to_us(struct timeval *p)
{
        return (p->tv_sec * USEC_1SEC + p->tv_usec);
}

// ret = x + y (x and y must be positive).
static inline void __timeval_add_pos(struct timeval *ret, struct timeval *x, struct timeval *y)
{
    ret->tv_sec = x->tv_sec + y->tv_sec;
    ret->tv_usec = x->tv_usec + y->tv_usec;
    if (ret->tv_usec >= USEC_1SEC) {
        ret->tv_sec++;
        ret->tv_usec -= USEC_1SEC;
    }
}

// ret = x - y (x and y must be positive)
static inline void __timeval_sub_pos(struct timeval *ret, struct timeval *x, struct timeval *y)
{
    ret->tv_sec = x->tv_sec - y->tv_sec;
    ret->tv_usec = x->tv_usec - y->tv_usec;
    if (ret->tv_usec < 0) {
        ret->tv_sec--;
        ret->tv_usec += USEC_1SEC;
    }
}

// ret = x + y
static inline void timeval_add(struct timeval *ret, struct timeval *x, struct timeval *y)
{
    if (ret != x && ret != y)
        timeval_clear(ret);
    
    __timeval_add_pos(ret, x, y);
}

// ret = x - y
static inline void timeval_sub(struct timeval *ret, struct timeval *x, struct timeval *y)
{
    if (ret != x && ret != y)
        timeval_clear(ret);
    
    __timeval_sub_pos(ret, x, y);
}

#endif
