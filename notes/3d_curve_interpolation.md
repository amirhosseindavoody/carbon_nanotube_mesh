## Interpolate 3D curve
We used B-spline functions to interpolate rough meshes generated by BulletPhysics library.

### Spline functions
**Order**: A spline function of order n is a piecewise polynomial function of degree n-1 in a variable x.

**Knots**: The places where the pieces meet are known as knots.

### Implementation

Use `scipy.interpolate` package to calculate spline parameters ans later one evaluate spline curves using calculated parameters.

- `splprep` is used to calculated spline parameters:

    - The normal output is a 3-tuple, `(t, c, k)`, containing the knot-points, `t`, the coefficients `c` and the order `k` of the spline.
    - The output `u` is the parameter array, which would produce the equivalent of input points for calculating the spline curves.
    - `x_test, y_test, z_test = interpolate.splev(u, tck)` would produce such points in 3d space.

- `splev(x, tck, der=0, ext=0)` <br> Evaluate a B-spline or its derivatives. Given the knots and coefficients of a B-spline representatio, evaluate the value of the smoothing polynomial and its derivatives.

Look here for more: https://stackoverflow.com/questions/18962175/spline-interpolation-coefficients-of-a-line-curve-in-3d-space