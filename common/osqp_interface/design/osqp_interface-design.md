# Interface for the OSQP library

This is the design document for the `osqp_interface` package.

## Purpose / Use cases

<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->

This packages provides a C++ interface for the [OSQP library](https://osqp.org/docs/solver/index.html).

## Design

<!-- Required -->
<!-- Things to consider:
    - How does it work? -->

The class `OSQPInterface` takes a problem formulation as Eigen matrices and vectors, converts these objects into
C-style Compressed-Column-Sparse matrices and dynamic arrays, loads the data into the OSQP workspace dataholder, and runs the optimizer.

## Inputs / Outputs / API

<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

The interface can be used in several ways:

1. Initialize the interface WITHOUT data. Load the problem formulation at the optimization call.

   ```cpp
       osqp_interface = OSQPInterface();
       osqp_interface.optimize(P, A, q, l, u);
   ```

2. Initialize the interface WITH data.

   ```cpp
       osqp_interface = OSQPInterface(P, A, q, l, u);
       osqp_interface.optimize();
   ```

3. WARM START OPTIMIZATION by modifying the problem formulation between optimization runs.

   ```cpp
       osqp_interface = OSQPInterface(P, A, q, l, u);
       osqp_interface.optimize();
       osqp.initializeProblem(P_new, A_new, q_new, l_new, u_new);
       osqp_interface.optimize();
   ```

   The optimization results are returned as a vector by the optimization function.

   ```cpp
   std::tuple<std::vector<double>, std::vector<double>> result = osqp_interface.optimize();
   std::vector<double> param = std::get<0>(result);
   double x_0 = param[0];
   double x_1 = param[1];
   ```

## References / External links

<!-- Optional -->

- OSQP library: <https://osqp.org/>

## Related issues

<!-- Required -->
