#include "euclidean_space.hpp"

using namespace euclidean_space;

int main(void)
{
  double distance = 0;

  point *a = new point(5.0, 5.0, 1.0);
  point *b = new point(0.0, 0.0, 0.0);

  distance = EuclideanSpace::find_distance(a, b);
  std::cout << "distance = " << distance << std::endl;

  double angle = 0.0;

  angle = EuclideanSpace::find_angle(b, a);
  std::cout << "angle = " << angle << std::endl;
}
