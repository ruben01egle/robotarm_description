# robotarm_kinematics

`kinematics_interface` plugin for the 6-DOF robot arm. `KinematicsCore` parses the URDF into
Denavit-Hartenberg (DH) parameters and is the base for forward kinematics and Jacobians.
`Kinematics` derives from it and will add the geometric inverse kinematics (IK).

Notation: `Rz(q)` / `Rx(q)` rotate about z / x, `Tz(d)` / `Tx(a)` translate along z / x.
`X_in_Y` is the pose of frame X expressed in frame Y. Row `k` is joint `k` (`joints_[k-1]`).

## Parameters (read once in `KinematicsCore::initialize`)

| parameter                  | default | meaning                                                                      |
|----------------------------|---------|------------------------------------------------------------------------------|
| `<param_namespace>.lambda` | `0.01`  | damping of the damped least squares in `calculate_jacobian_inverse`, `>= 0`  |

`<param_namespace>` is the argument of `initialize`; if it is empty the parameter is just `lambda`.
A value that is negative, not finite or not a double makes `initialize` return `false`.

## Caller contract of the kinematics functions

- **Inputs are validated**: `joint_pos` (and `delta_theta`) need one entry per joint, and
  `joint_pos`, `delta_x` and `delta_theta` must be finite. Otherwise the call logs a throttled
  error, returns `false` and leaves the output untouched.
- **Outputs are not size-checked**: dynamic-size outputs (`jacobian` 6xN, `jacobian_inverse` Nx6,
  `delta_theta` N) are resized by Eigen if they do not fit. That allocates, so real-time callers
  should pass pre-sized objects to keep the call allocation free.
- The `std::vector` overloads of `kinematics_interface` copy their inputs into fresh Eigen objects
  on every call and therefore allocate regardless. Real-time callers should use the `Eigen`
  overloads with preallocated buffers.

## URDF constraints (checked in `KinematicsCore::initialize`)

- Serial chain: every link has at most one child, and there are exactly 7 joints.
- Joints 1-6 are `revolute` with valid limits (`lower < upper`, `velocity > 0`, `effort > 0`).
  The 7th (last) joint is `fixed` and its child is the tcp link. The tcp link may be empty
  (`<link name="tcp"/>`); it is only a frame.
- Every revolute joint has `<axis xyz="0 0 1"/>`.
- The origin of the first joint is identity, so the root link frame is DH frame 0.
- Each joint origin (except the first) is DH-conform, see below.
- If a link has an `<inertial>`, its mass must be positive.

## Additional constraints for the IK

The closed-form IK needs a spherical wrist and a fixed arrangement of the other axes. They are
conditions on the DH parameters (only the structure, the link lengths stay free):

| condition                                    | meaning                                  |
|----------------------------------------------|------------------------------------------|
| `alpha_1 = ±90°`                             | axis 1 perpendicular to axis 2           |
| `alpha_2 = 0`                                | axes 2 and 3 parallel                    |
| `alpha_3 = ±90°`                             | axis 3 perpendicular to axis 4           |
| `a_4 = 0`, `alpha_4 = ±90°`                  | axes 4 and 5 intersect at a right angle  |
| `a_5 = 0`, `alpha_5 = ±90°`                  | axes 5 and 6 intersect at a right angle  |
| `d_5 = 0`                                    | axes 4, 5 and 6 meet in one point        |

Axis 1 is the z axis of the root link (first origin is identity, axis is `0 0 1`). Whether that
is vertical depends on how the base is mounted. To be enforced in `Kinematics::initialize`.

## How the DH parameters are derived from the URDF

In a URDF the child link frame is the joint frame, and the joint rotates about its z axis by `q`:

```
URDF_link_k = O_1 Rz(q_1) · O_2 Rz(q_2) · ... · O_k Rz(q_k)
```

`O_j` is the origin of joint `j`, expressed in the frame of link `j-1` (`O_1 = I`).
Standard DH describes the same chain as

```
A_k(q) = Rz(q + theta_0) · Tz(d) · Tx(a) · Rx(alpha)      DH_k = A_1(q_1) · ... · A_k(q_k)
```

In DH, frame `k` has its z axis along joint `k+1`, while the URDF frame of link `k` has its z axis
along joint `k`. So the fixed part of row `k` comes from the origin of the **next** joint, `O_{k+1}`
(the last row, joint 6, comes from the tcp joint). `O_{k+1}` is a fixed transform from the URDF
frame of link `k` (call it `P`, z along joint `k`) to DH frame `k` (call it `C`, z along joint `k+1`).
The task is to find `theta_0, d, a, alpha` such that `O_{k+1} = A_k(0)`.

### 1. Write the DH transform as translation · rotation

Moving the rotation to the right of the translations with `Rz(t) · Trans(v) = Trans(Rz(t) · v) · Rz(t)`:

```
A_k(0) = Rz(theta_0) · Tz(d) · Tx(a) · Rx(alpha)
       = Trans( a·cos(theta_0), a·sin(theta_0), d ) · Rz(theta_0) · Rx(alpha)
```

### 2. Compare with the URDF origin

A URDF origin with `xyz = p = (x, y, z)` and `rpy = (roll, pitch, yaw)` is

```
O = Trans(p) · R,      R = Rz(yaw) · Ry(pitch) · Rx(roll)
```

Rotation and translation of `O_{k+1} = A_k(0)` must match separately:

```
rotation:     Rz(yaw) · Ry(pitch) · Rx(roll)  =  Rz(theta_0) · Rx(alpha)
translation:  (x, y, z)                       =  (a·cos(theta_0), a·sin(theta_0), d)
```

DH has 4 parameters, a general pose has 6, so an origin only fits if two conditions hold. Both are
the usual DH rules for placing frame `C` relative to `P`, and both are checked in `initialize`:

1. **No Ry factor: `pitch = 0`** ("invalid rotation"). The rotation side then reads
   `Rz(yaw) · Rx(roll) = Rz(theta_0) · Rx(alpha)`, so `theta_0 = yaw` and `alpha = roll`.
   Geometrically `x_C ⟂ z_P`: the x axis of `C` is `R · x_hat = Rz(yaw) · (cos p, 0, -sin p)`,
   whose z component is `-sin(pitch)`.
2. **The y component of `Rz(-yaw) · p` is zero** ("invalid translation"). This is the
   translation side: `(x, y)` must point along `(cos theta_0, sin theta_0)`. Geometrically the
   x axis of `C` intersects the z axis of `P`.

### 3. Read off the parameters

```
theta_0 = yaw
d       = z
a       = p · (R · x_hat)              = x·cos(yaw) + y·sin(yaw)
alpha   = atan2( (R^T z_hat)_y , (R^T z_hat)_z )                    (= roll)
```

- `a`: `R · x_hat = (cos theta_0, sin theta_0, 0)`, so the projection of `p` onto it gives back
  exactly `a`. Any leftover perpendicular part would be the y component from condition 2.
- `alpha`: `R^T · z_hat = Rx(-alpha) · z_hat = (0, sin alpha, cos alpha)`, so `atan2` of its y and z
  components returns `alpha`. This is the angle between `z_P` and `z_C` seen in the y-z plane of `C`.
  It equals `roll` once `pitch = 0`, but measuring it this way only needs the rotation matrix `R`.

### 4. Why the yaw becomes a joint-angle offset

Joint `k` rotates about the z axis of its own frame, and the leading `Rz(theta_0)` of `O_{k+1}` is a
rotation about that same axis, right after the joint rotation. They combine to
`Rz(q_k) · Rz(theta_0) = Rz(q_k + theta_0)`, which is why the DH joint variable is
`theta_k = q_k + theta_0` and `A_k(q) = Rz(q) · A_k(0)`. At `q = 0` the row reproduces `O_{k+1}` exactly.

### Example: `axis3` (row 3, `joints_[2]`)

The next joint, `axis4`, has `xyz = (0, -0.015, 0)` and `rpy = (-pi/2, 0, -pi/2)`.

```
pitch = 0                                                                  -> condition 1 ok
Rz(-yaw) · p = Rz(pi/2) · (0, -0.015, 0) = (0.015, 0, 0),  y = 0           -> condition 2 ok

theta_0 = yaw = -pi/2
d       = z   = 0
a       = p · (cos(-pi/2), sin(-pi/2), 0) = (0, -0.015, 0) · (0, -1, 0) = 0.015
alpha   = roll = -pi/2
```

This matches the row printed by `print_joints()` (`a = 0.015`, `alpha = -90 deg`, `d = 0`, `theta_0 = -90 deg`).

## Why the inverse transform is needed to get the URDF frames

`calculate_link_transform` must return the URDF frame of the requested link, expressed in the
root link frame. The DH chain does not give that directly: DH frame `k` sits at the origin of the
next joint, so

```
DH_k = URDF_link_k · A_k(0)         =>         URDF_link_k = DH_k · A_k(0)^-1
```

`A_k(0)^-1` is stored per joint as `child_urdf_frame_in_child_dh_frame_` (computed once in
`initialize`). Both frames are fixed to the child link of joint `k`, which is why it is constant and
does not depend on the joint angles. Equivalent form: `URDF_link_k = DH_{k-1} · Rz(q_k)`.

| link_name                        | result                                                          |
|----------------------------------|-----------------------------------------------------------------|
| root link                        | identity                                                        |
| child link of joint `k` (1..6)   | `DH_k · child_urdf_frame_in_child_dh_frame_` of joint `k`       |
| tcp                              | `DH_6` (the tcp joint origin is exactly the last DH offset)     |

Correctness relies on the first origin being identity: it makes DH frame 0 equal to the root link frame.

## Jacobian and its damped pseudo-inverse

`calculate_jacobian` returns the geometric Jacobian `J` (6xN) of a link in the root link frame:
column `i` is `(z_i × (p_end − p_i), z_i)`, with the joint axis `z_i` and position `p_i` of joint `i`.
It maps joint velocities to a twist, `x_dot = J · q_dot` (rows 0-2 linear in m/s, rows 3-5 angular in rad/s).
Joints behind the requested link get a zero column.

`J` cannot simply be inverted: it is singular at kinematic singularities, where `J⁻¹` blows up and a
tiny cartesian step would demand huge joint velocities. `calculate_jacobian_inverse` therefore returns
the damped least squares pseudo-inverse `J⁺` (Nx6), which is used by
`convert_cartesian_deltas_to_joint_deltas` as `delta_theta = J⁺ · delta_x`.

`J⁺` comes from trading tracking error against joint motion:

```
min over q_dot:   ‖J q_dot − x_dot‖² + λ² ‖q_dot‖²
```

Setting the gradient to zero gives `(JᵀJ + λ²I) q_dot = Jᵀ x_dot`, so

```
J⁺ = (JᵀJ + λ²I)⁻¹ Jᵀ          <=>          (JᵀJ + λ²I) · J⁺ = Jᵀ
```

For `λ > 0` the matrix `M = JᵀJ + λ²I` (NxN) is symmetric positive definite, hence always invertible.
It is not inverted explicitly: `M` is factored once with LDLT and the system `M · X = Jᵀ` is solved
for `X = J⁺`. Equivalent form: `J⁺ = Jᵀ (J Jᵀ + λ²I)⁻¹`, which only needs a 6x6 matrix.

With the SVD `J = U Σ Vᵀ` this is `J⁺ = Σ σᵢ / (σᵢ² + λ²) · vᵢ uᵢᵀ`. Directions with `σᵢ ≫ λ` are
inverted as usual (`≈ 1/σᵢ`), directions with `σᵢ ≪ λ` are suppressed (`≈ σᵢ/λ²`), and the gain never
exceeds `1/(2λ)`, which bounds the joint velocities near a singularity.

- `λ` is the parameter `lambda` (default `0.01`). Larger is safer near singularities but less accurate:
  `J · J⁺ ≠ I`, so `J · delta_theta` differs slightly from `delta_x`. In an iterative IK loop this
  residual is corrected by the next iteration.
- The damping is constant. Since the rows of `J` mix metres and radians, the same `λ` weighs the
  position and orientation parts differently.
