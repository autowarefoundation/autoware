#include "amathutils_lib/xorshift128.hpp"

namespace amathutils
{
XorShift128::XorShift128() : x_(123456789u), y_(362436069u), z_(521288629u), w_(88675123u) {}
XorShift128::XorShift128(unsigned int seed) : x_(123456789u), y_(362436069u), z_(521288629u), w_(seed) {}

unsigned int XorShift128::random()
{
    unsigned int t;
    t = x_ ^ (x_ << 11);
    x_ = y_;
    y_ = z_;
    z_ = w_;
    return w_ = (w_ ^ (w_ >> 19)) ^ (t ^ (t >> 8));
}
unsigned int XorShift128::operator()() { return random(); }
}