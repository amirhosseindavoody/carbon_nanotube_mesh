# Working with quaternions for rations

Rotation is three dimension as a rotation by around axis $\vec{b}$ by angle $\alpha$. We can specify the rotation axis via [direction cosines](https://en.wikipedia.org/wiki/Direction_cosine)

$$ \cos(\beta_x) = \frac{\vec{b}.\hat{x}}{|b|} \quad,\quad \cos(\beta_y) = \frac{\vec{b}.\hat{y}}{|b|} \quad,\quad \cos(\beta_z) = \frac{\vec{b}.\hat{z}}{|b|} $$

Therefore, the quaternion elements, $\bm{q}=[q_0, q_1, q_2, q_3]^T$, are given by 

$$
\begin{aligned}
q_0 = & ~\cos(\alpha/2) \\
q_1 = & ~\sin(\alpha/2) \cos(\beta_x) \\
q_2 = & ~\sin(\alpha/2) \cos(\beta_y) \\
q_3 = & ~\sin(\alpha/2) \cos(\beta_z) \\
\end{aligned}
$$

The rotation matrix for normalized quaternion is
$$
R = \begin{bmatrix}
q_0^2 + q_1^2 - q_2^2 - q_3^2 &  2(q_1 q_2 - q_0 q_3) &  2(q_0 q_2 + q_1 q_3) \\
2(q_1 q_2 + q_0 q_3) & q_0^2 - q_1^2 + q_2^2 - q_3^2 &  2(q_2 q_3 - q_0 q_1) \\
2(q_1 q_3 - q_0 q_2) & 2( q_0 q_1 + q_2 q_3) & q_0^2 - q_1^2 - q_2^2 + q_3^2 
\end{bmatrix}
$$


#### References:
- https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles