## Note on implementation of informed RRT\*

### Preliminary knowledge on informed-RRT\*

Let us define $f(x)$ as minimum cost of the path when path is constrained to pass through $x$ (so path will be $x_{\mathrm{start}} \to \mathrm{x} \to \mathrm{x_{\mathrm{goal}}}$). Also, let us define $c_{\mathrm{best}}$ as the current minimum cost of the feasible paths. Let us define a set $ X(f) = \left\{ x \in X | f(x) < c*{\mathrm{best}} \right\} $. If we could sample a new point from $X_f$ instead of $X$ as in vanilla RRT\*, chance that $c*{\mathrm{best}}$ is updated is increased, thus the convergence rate is improved.

In most case, $f(x)$ is unknown, thus it is straightforward to approximiate the function $f$ by a heuristic function $\hat{f}$. A heuristic function is [admissible](https://en.wikipedia.org/wiki/Admissible_heuristic) if $\forall x \in X, \hat{f}(x) < f(x)$, which is sufficient condition of conversion to optimal path. The good heuristic function $\hat{f}$ has two properties: 1) it is an admissible tight lower bound of $f$ and 2) sampling from $X(\hat{f})$ is easy.

According to Gammell et al [1], a good heursitic function when path is always straight is $\hat{f}(x) = ||x_{\mathrm{start}} - x|| + ||x - x_{\mathrm{goal}}||$. If we don't assume any obstacle information the heursitic is tightest. Also, $X(\hat{f})$ is hyper-ellipsoid, and hence sampling from it can be done analitically.

### Modification to fit reeds-sheep path case

In the vehicle case, state is $x = (x_{1}, x_{2}, \theta)$. Unlike normal informed-RRT\* where we can connect path by a straight line, here we connect the vehicle path by a reeds-sheep path. So, we need some modification of the original algorithm a bit. To this end, one might first consider a heuristic function $\hat{f}_{\mathrm{RS}}(x) = \mathrm{RS}(x_{\mathrm{start}}, x) + \mathrm{RS}(x, x_{\mathrm{goal}}) < f(x)$ where $\mathrm{RS}$ computes reeds-sheep distance. Though it is good in the sense of tightness, however, sampling from $X(\hat{f}_{RS})$ is really difficult. Therefore, we use $\hat{f}_{euc} = ||\mathrm{pos}(x_{\mathrm{start}}) - \mathrm{pos}(x)|| + ||\mathrm{pos}(x)- \mathrm{pos}(x_{\mathrm{goal}})||$, which is admissible because $\forall x \in X, \hat{f}_{euc}(x) < \hat{f}_{\mathrm{RS}}(x) < f(x)$. Here, $\mathrm{pos}$ function returns position $(x_{1}, x_{2})$ of the vehicle.

Sampling from $X(\hat{f}_{\mathrm{euc}})$ is easy because $X(\hat{f}_{\mathrm{euc}}) = \mathrm{Ellipse} \times (-\pi, \pi]$. Here $\mathrm{Ellipse}$'s focal points are $x_{\mathrm{start}}$ and $x_{\mathrm{goal}}$ and conjugate diameters is $\sqrt{c^{2}_{\mathrm{best}} - ||\mathrm{pos}(x_{\mathrm{start}}) - \mathrm{pos}(x_{\mathrm{goal}}))|| } $ (similar to normal informed-rrtstar's ellipsoid). Please notice that $\theta$ can be arbitrary because $\hat{f}_{\mathrm{euc}}$ is independent of $\theta$.

[1] Gammell et al., "Informed RRT\*: Optimal sampling-based path planning focused via direct sampling of an admissible ellipsoidal heuristic." IROS (2014)
