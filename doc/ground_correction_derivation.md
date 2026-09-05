# Ground-normal alignment used by the dynamic-object filter

The segmentation stage rotates the estimated ground plane into a level frame
before it labels ground and foreground points. This note defines the transform
implemented by `ComputeVectorAlignmentRotation()` and records its invariants.

## 1. Problem definition

Let the estimated ground plane in the LiDAR frame be

\[
    n^T(p-p_0)=0,
\]

where `n` is a non-zero ground normal and `p0` is a point on the plane. We seek
a **proper rotation** `R` that maps the normalized ground normal to the target
up direction `t = [0, 0, 1]^T`:

\[
    R\hat n=t, \qquad R^TR=I, \qquad \det(R)=1,
\]

with

\[
    \hat n=\frac{n}{\lVert n\rVert}.
\]

## 2. General case

For normalized source and target directions, define

\[
    c=\hat n^Tt=\cos\theta,\qquad
    v=\hat n\times t,\qquad
    s=\lVert v\rVert=\sin\theta.
\]

If `s > epsilon`, the unit rotation axis is

\[
    k=\frac{v}{s}.
\]

Its skew-symmetric cross-product matrix is

\[
    [k]_\times=
    \begin{bmatrix}
      0&-k_z&k_y\\
      k_z&0&-k_x\\
      -k_y&k_x&0
    \end{bmatrix}.
\]

Rodrigues' formula gives the shortest proper rotation from `n_hat` to `t`:

\[
    \boxed{R=cI+(1-c)kk^T+s[k]_\times}.
\]

Because `k` is perpendicular to `n_hat`, `kk^T n_hat = 0`. The vector triple
product also gives

\[
    k\times\hat n
    =\frac{(\hat n\times t)\times\hat n}{s}
    =\frac{t-c\hat n}{s}.
\]

Therefore

\[
    R\hat n=c\hat n+s(k\times\hat n)=t.
\]

Rodrigues' matrix is the exponential of a skew-symmetric matrix, so it belongs
to `SO(3)` and consequently satisfies `R^T R = I` and `det(R) = 1`.

## 3. Singular cases

- Same direction (`s <= epsilon`, `c >= 0`): `R = I`.
- Opposite direction (`s <= epsilon`, `c < 0`): the rotation axis is not
  unique. A deterministic unit axis `k` perpendicular to `n_hat` is selected,
  and the exact 180-degree rotation is

\[
    R=2kk^T-I.
\]

- A null, non-finite, or zero-length direction is rejected. The caller skips
  dynamic filtering for that scan instead of applying a corrupt transform.

## 4. Plane leveling and height removal

After `p' = Rp`, the plane normal is

\[
    n'=R\hat n=t=e_z.
\]

The rotated plane therefore has constant height

\[
    h=e_z^TRp_0.
\]

The corrected point is

\[
    \boxed{p''=Rp-he_z},
\]

and every ideal ground point satisfies `p''_z = 0`.

The legacy point code indexes `RTM` as a column-major 3-by-3 matrix. The
implementation preserves that storage convention; `RTM[2]`, `RTM[5]`, and
`RTM[8]` are the third row used to compute `h`.

## 5. Runtime checks

Before the matrix is exposed to the point-cloud loop, the implementation checks

\[
    \lVert R^TR-I\rVert_F\le 10^{-6},\qquad
    |\det(R)-1|\le 10^{-6},\qquad
    \lVert R\hat n-t\rVert_2\le 10^{-6}.
\]

These checks prevent a bad ground estimate or floating-point failure from
silently deforming a complete scan.
