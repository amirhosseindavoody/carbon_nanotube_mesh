# Create random points with arbitrary distribution

Assume we want to create a set of random numbers with distribution function $f(x)$ where

$$
\int_{x_0}^{x_1} f(x) dx = 1
$$

We use a random number generator with uniform distribution $g(y)=1$.

This problem a like a _mapping_ problem
> Think about why this is a mapping problem and why this equation is the one we need to solve

$$
\int_{0}^{y} g(y') dy' = \int_{x_0}^{x}  f(x') dx'
$$

Therefore, we get

$$
y = \int_{x_0}^{x}  f(x') dx' = F(x) - F(x_0),
$$

where

$$
F(x) = \int f(x) dx.
$$

## Create random points on a sphere

The probability distribution function is proportional to the area of the sphere at polar angle ($\theta$) and azimuthal angle ($\phi$)

$$
f(\theta, \phi) ~ d\theta ~ d\phi \propto A(\theta, \phi) = \sin(\theta) ~ d\theta ~ d\phi
$$

$$
f(\theta, \phi) = f_1(\theta) f_2(\phi) \quad , \quad f_1(\theta) = \sin(\theta) \quad , \quad f_2(\phi) = 1.
$$

Therefore, following the procedure above we do get the following

$$
y_1 = \frac{\int_0^\theta \sin(\theta') ~ d\theta'}{\int_0^\pi \sin(\theta') ~ d\theta'} = \frac{1-\cos(\theta)}{2} \quad \Rightarrow \quad \theta = \cos^{-1}(1-2y_1)
$$

$$
y_2 = \frac{\int_0^\phi d\phi'}{\int_0^{2\pi}  d\phi'} = \frac{\phi}{2\pi} \quad \Rightarrow \quad \phi = 2\pi y_2
$$