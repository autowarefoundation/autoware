#ifndef NMS_HPP_
#define NMS_HPP_

#include <vector>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <boost/assert.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/algorithms/area.hpp>
#include <boost/assign/std/vector.hpp>

enum PointInRectangle {X_FrontLeft, Y_FrontLeft,
						X_BackLeft, Y_BackLeft,
						X_BackRight, Y_BackRight,
						X_FrontRight, Y_FrontRight};

typedef boost::geometry::model::d2::point_xy<double> boost_point_xy;
typedef boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double> > boost_polygon;

std::vector< std::vector<float> > Nms(const std::vector<std::vector<float> > &,
                               const float &);

std::vector<float> get_point_from_vector(const std::vector<std::vector<float> > &,
                                         const PointInRectangle &);

std::vector<float> get_polygon_area(const std::vector<float> &x1,
                                    const std::vector<float> &y1,
                                    const std::vector<float> &x2,
                                    const std::vector<float> &y2,
                                    const std::vector<float> &x3,
                                    const std::vector<float> &y3,
                                    const std::vector<float> &x4,
                                    const std::vector<float> &y4);

std::vector<float> get_largest_y(const std::vector<float> &y1,
                                 const std::vector<float> &y2,
                                 const std::vector<float> &y3,
                                 const std::vector<float> &y4);

std::vector<float> euclidean_distance(const std::vector<float> &a,
                                      const std::vector<float> &b);


std::vector<size_t> argsort(std::vector<float> & v);

std::vector<float> set_elements_to_min_value(const float &,
                                             const std::vector<float> &);

std::vector<float> set_elements_to_max_value(const float &,
                                             const std::vector<float> &);

std::vector<float> copy_by_indices(const std::vector<float> &,
                                   const std::vector<size_t> &);

std::vector<size_t> remove_last_element(const std::vector<size_t> &);

std::vector<float> subtract_element_wise(const std::vector<float> &,
                                         const std::vector<float> &);

std::vector<float> multiply_element_wise(const std::vector<float> &,
                                         const std::vector<float> &);

std::vector<float> divide_element_wise(const std::vector<float> &,
                                       const std::vector<float> &);

std::vector<size_t> keep_by_threshold(const std::vector<float> &,
                                   const float &);

std::vector<size_t> remove_elements_by_index(const std::vector<size_t> &,
                                          const std::vector<size_t> &);

std::vector< std::vector<float> > BoxesToRectangles(const std::vector< std::vector<float> > &);

template <typename T>
std::vector<T> keep_only_indices(const std::vector<T> &,
                                 const std::vector<size_t> &);

#endif // NMS_HPP_
