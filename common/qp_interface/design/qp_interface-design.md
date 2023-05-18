# Interface for QP solvers

This is the design document for the `qp_interface` package.

## Purpose / Use cases

This packages provides a C++ interface for QP solvers.
Currently, supported QP solvers are

- [OSQP library](https://osqp.org/docs/solver/index.html)

## Design

The class `QPInterface` takes a problem formulation as Eigen matrices and vectors, converts these objects into
C-style Compressed-Column-Sparse matrices and dynamic arrays, loads the data into the QP workspace dataholder, and runs the optimizer.

## Inputs / Outputs / API

The interface can be used in several ways:

1. Initialize the interface, and load the problem formulation at the optimization call.

   ```cpp
       QPInterface qp_interface;
       qp_interface.optimize(P, A, q, l, u);
   ```

2. WARM START OPTIMIZATION by modifying the problem formulation between optimization runs.

   ```cpp
       QPInterface qp_interface(true);
       qp_interface.optimize(P, A, q, l, u);
       qp_interface.optimize(P_new, A_new, q_new, l_new, u_new);
   ```

   The optimization results are returned as a vector by the optimization function.

   ```cpp
   const auto solution = qp_interface.optimize();
   double x_0 = solution[0];
   double x_1 = solution[1];
   ```

## References / External links

- OSQP library: <https://osqp.org/>

## Related issues
