#pragma once
#include <climits>
#include <random>

namespace amathutils
{
class XorShift128
{
  public:
    using result_type = unsigned int;
    static constexpr unsigned int min() { return 0u; }
    static constexpr unsigned int max() { return UINT_MAX; }
    unsigned int operator()();
    XorShift128();
    XorShift128(unsigned int seed);

  private:
    unsigned int x_;
    unsigned int y_;
    unsigned int z_;
    unsigned int w_;
    result_type random();
};
} // namespace amathutils