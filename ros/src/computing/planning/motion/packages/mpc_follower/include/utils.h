#include <vector>

void fill_increse(std::vector<double>::iterator first,
                  std::vector<double>::iterator last, double init,
                  double diff) {
  double value = init;
  while (first != last) {
    *first++ = value;
    value += diff;
  }
}
