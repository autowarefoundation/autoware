# Interpolation package

This package supplies linear and spline interpolation functions.

## Linear Interpolation

`lerp(src_val, dst_val, ratio)` (for scalar interpolation) interpolates `src_val` and `dst_val` with `ratio`.
This will be replaced with `std::lerp(src_val, dst_val, ratio)` in `C++20`.

`lerp(base_keys, base_values, query_keys)` (for vector interpolation) applies linear regression to each two continuous points whose x values are`base_keys` and whose y values are `base_values`.
Then it calculates interpolated values on y-axis for `query_keys` on x-axis.

## Spline Interpolation

`spline(base_keys, base_values, query_keys)` (for vector interpolation) applies spline regression to each two continuous points whose x values are`base_keys` and whose y values are `base_values`.
Then it calculates interpolated values on y-axis for `query_keys` on x-axis.

### Evaluation of calculation cost

We evaluated calculation cost of spline interpolation for 100 points, and adopted the best one which is tridiagonal matrix algorithm.
Methods except for tridiagonal matrix algorithm exists in `spline_interpolation` package, which has been removed from Autoware.

| Method                            | Calculation time |
| --------------------------------- | ---------------- |
| Tridiagonal Matrix Algorithm      | 0.007 [ms]       |
| Preconditioned Conjugate Gradient | 0.024 [ms]       |
| Successive Over-Relaxation        | 0.074 [ms]       |

### Spline Interpolation Algorithm

Assuming that the size of `base_keys` ($x_i$) and `base_values` ($y_i$) are $N + 1$, we aim to calculate spline interpolation with the following equation to interpolate between $y_i$ and $y_{i+1}$.

$$
Y_i(x) = a_i (x - x_i)^3 + b_i (x - x_i)^2 + c_i (x - x_i) + d_i \ \ \ (i = 0, \dots, N-1)
$$

Constraints on spline interpolation are as follows.
The number of constraints is $4N$, which is equal to the number of variables of spline interpolation.

$$
\begin{align}
Y_i (x_i) & = y_i \ \ \ (i = 0, \dots, N-1) \\
Y_i (x_{i+1}) & = y_{i+1} \ \ \ (i = 0, \dots, N-1) \\
Y'_i (x_{i+1}) & = Y'_{i+1} (x_{i+1}) \ \ \ (i = 0, \dots, N-2) \\
Y''_i (x_{i+1}) & = Y''_{i+1} (x_{i+1}) \ \ \ (i = 0, \dots, N-2) \\
Y''_0 (x_0) & = 0 \\
Y''_{N-1} (x_N) & = 0
\end{align}
$$

According to [this article](https://www.mk-mode.com/rails/docs/INTERPOLATION_SPLINE.pdf), spline interpolation is formulated as the following linear equation.

$$
\begin{align}
 \begin{pmatrix}
    2(h_0 + h_1) & h_1 \\
    h_0 & 2 (h_1 + h_2) & h_2 & & O \\
        &     &     & \ddots \\
    O &     &     &       & h_{N-2} & 2 (h_{N-2} + h_{N-1})
 \end{pmatrix}
 \begin{pmatrix}
    v_1 \\ v_2 \\ v_3 \\ \vdots \\ v_{N-1}
 \end{pmatrix}=
 \begin{pmatrix}
    w_1 \\ w_2 \\ w_3 \\ \vdots \\ w_{N-1}
 \end{pmatrix}
\end{align}
$$

where

$$
\begin{align}
h_i & = x_{i+1} - x_i \ \ \ (i = 0, \dots, N-1) \\
w_i & = 6 \left(\frac{y_{i+1} - y_{i+1}}{h_i} - \frac{y_i - y_{i-1}}{h_{i-1}}\right) \ \ \ (i = 1, \dots, N-1)
\end{align}
$$

The coefficient matrix of this linear equation is tridiagonal matrix. Therefore, it can be solve with tridiagonal matrix algorithm, which can solve linear equations without gradient descent methods.

Solving this linear equation with tridiagonal matrix algorithm, we can calculate coefficients of spline interpolation as follows.

$$
\begin{align}
a_i & = \frac{v_{i+1} - v_i}{6 (x_{i+1} - x_i)} \ \ \ (i = 0, \dots, N-1) \\
b_i & = \frac{v_i}{2} \ \ \ (i = 0, \dots, N-1) \\
c_i & = \frac{y_{i+1} - y_i}{x_{i+1} - x_i} - \frac{1}{6}(x_{i+1} - x_i)(2 v_i + v_{i+1}) \ \ \ (i = 0, \dots, N-1) \\
d_i & = y_i \ \ \ (i = 0, \dots, N-1)
\end{align}
$$

### Tridiagonal Matrix Algorithm

We solve tridiagonal linear equation according to [this article](https://www.iist.ac.in/sites/default/files/people/tdma.pdf) where variables of linear equation are expressed as follows in the implementation.

$$
\begin{align}
 \begin{pmatrix}
    b_0 & c_0 &     & \\
    a_0 & b_1 & c_2 & O \\
        &     & \ddots \\
    O &     & a_{N-2} &  b_{N-1}
 \end{pmatrix}
x =
\begin{pmatrix}
    d_0 \\ d_2 \\ d_3 \\ \vdots \\ d_{N-1}
 \end{pmatrix}
\end{align}
$$
