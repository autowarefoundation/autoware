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

bool interp1d(const std::vector<double> &index,
              const std::vector<double> &values, const double &ref,
              double &obj) {

  obj = 0.01;

  if (!(index.size() == values.size())) {
    printf("size of index and values should be same size\n");
    return false;
  }
  if (ref < index.front()) {
    obj = index.front();
    printf("ref point is out of index\n");
    return false;
  }
  if (index.back() < ref) {
    obj = index.back();
    printf("ref point is out of index\n");
    return false;
  }

  uint i = 1;
  while (ref > index[i]) {
    ++i;
  }

  const double a = ref - index[i - 1];
  const double d_index = index[i] - index[i- 1];
  obj = ((d_index - a) * values[i - 1] + a * values[i]) / d_index;

  // printf("i = %d, ref = %f, index.front() = %f, index.back() = %f, a = %f, d_index = %f, obj = %f\n",i,ref, index.front(), index.back(), a, d_index, obj);
  return true;
}
