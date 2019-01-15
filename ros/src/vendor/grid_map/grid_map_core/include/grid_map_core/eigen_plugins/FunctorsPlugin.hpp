#include <math.h>

template<typename Scalar> struct scalar_sum_of_finites_op {
  EIGEN_EMPTY_STRUCT_CTOR(scalar_sum_of_finites_op)
  EIGEN_STRONG_INLINE const Scalar operator() (const Scalar& a, const Scalar& b) const {
    using std::isfinite;
    if (isfinite(a) && isfinite(b)) return a + b;
    if (isfinite(a)) return a;
    if (isfinite(b)) return b;
    return a + b;
  }
};
template<typename Scalar>
struct functor_traits<scalar_sum_of_finites_op<Scalar> > {
  enum {
    Cost = 2 * NumTraits<Scalar>::ReadCost + NumTraits<Scalar>::AddCost,
    PacketAccess = false
  };
};

template<typename Scalar>
struct scalar_min_of_finites_op {
  EIGEN_EMPTY_STRUCT_CTOR(scalar_min_of_finites_op)
  EIGEN_STRONG_INLINE const Scalar operator() (const Scalar& a, const Scalar& b) const {
    using std::min;
    using std::isfinite;
    if (isfinite(a) && isfinite(b)) return (min)(a, b);
    if (isfinite(a)) return a;
    if (isfinite(b)) return b;
    return (min)(a, b);
  }
};
template<typename Scalar>
struct functor_traits<scalar_min_of_finites_op<Scalar> > {
  enum {
    Cost = NumTraits<Scalar>::AddCost,
    PacketAccess = false
  };
};

template<typename Scalar>
struct scalar_max_of_finites_op {
  EIGEN_EMPTY_STRUCT_CTOR(scalar_max_of_finites_op)
  EIGEN_STRONG_INLINE const Scalar operator() (const Scalar& a, const Scalar& b) const {
    using std::max;
    using std::isfinite;
    if (isfinite(a) && isfinite(b)) return (max)(a, b);
    if (isfinite(a)) return a;
    if (isfinite(b)) return b;
    return (max)(a, b);
  }
};
template<typename Scalar>
struct functor_traits<scalar_max_of_finites_op<Scalar> > {
  enum {
    Cost = NumTraits<Scalar>::AddCost,
    PacketAccess = false
  };
};
