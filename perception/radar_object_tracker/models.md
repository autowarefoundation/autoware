# models

Tracking models can be chosen from the ros parameter `~tracking_model`:

Each model has its own parameters, which can be set in the ros parameter server.

- model name
  - parameter name for general
  - override parameter name for each tracking object class

## linear constant acceleration model

- prediction

$$
\begin{bmatrix}
x_{k+1} \\
y_{k+1} \\
v_{x_{k+1}} \\
v_{y_{k+1}} \\
a_{x_{k+1}} \\
a_{y_{k+1}}
\end{bmatrix} =
\begin{bmatrix}
1 & 0 & dt & 0 & \frac{1}{2}dt^2 & 0 \\
0 & 1 & 0 & dt & 0 & \frac{1}{2}dt^2 \\
0 & 0 & 1 & 0 & dt & 0 \\
0 & 0 & 0 & 1 & 0 & dt \\
0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 1 \\
\end{bmatrix}
\begin{bmatrix}
x_k \\
y_k \\
v_{x_k} \\
v_{y_k} \\
a_{x_k} \\
a_{y_k}
\end{bmatrix} + noise
$$

- noise model

  - random walk in acc: 2 parameters(currently disabled)
  - random state noise: 6 parameters
    $$
    \begin{align}
    noise &= \begin{bmatrix}
    \frac{dt^{3}}{6} \\ 0\\ \frac{dt^{2}}{2} \\ 0 \\ dt \\0
    \end{bmatrix} \nu_{ax_k} + \begin{bmatrix}
    0 \\ \frac{dt^{3}}{6} \\ 0 \\ \frac{dt^{2}}{2} \\ 0 \\ dt
    \end{bmatrix} \nu_{ay_k}  &  \text{(random walk in acc)} \\
    noise &= \begin{bmatrix}
    \nu_{x_k} \\ \nu_{y_k} \\ \nu_{v_{x_k}} \\ \nu_{v_{y_k}} \\ \nu_{a_{x_k}} \\ \nu_{a_{y_k}}
    \end{bmatrix} &
     \text{(random state noise)} &
    \end{align}
    $$

- observation
  - observation: x,y,vx,vy
  - observation noise: 4 parameters

## constant turn rate and velocity model

Just idea, not implemented yet.

$$
\begin{align}
x_{k+1} &= x_k + \frac{v_k}{\omega_k} (sin(\theta_k + \omega_k dt) - sin(\theta_k)) \\
y_{k+1} &= y_k + \frac{v_k}{\omega_k} (cos(\theta_k) - cos(\theta_k + \omega_k dt)) \\
v_{k+1} &= v_k \\
\theta_{k+1} &= \theta_k + \omega_k dt \\
\omega_{k+1} &= \omega_k
\end{align}
$$
