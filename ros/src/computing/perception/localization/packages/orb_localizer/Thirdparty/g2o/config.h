#ifndef G2O_CONFIG_H
#define G2O_CONFIG_H

/* #undef G2O_OPENMP */
/* #undef G2O_SHARED_LIBS */

// give a warning if Eigen defaults to row-major matrices.
// We internally assume column-major matrices throughout the code.
#ifdef EIGEN_DEFAULT_TO_ROW_MAJOR
#  error "g2o requires column major Eigen matrices (see http://eigen.tuxfamily.org/bz/show_bug.cgi?id=422)"
#endif

#endif
