# robotarm_rbd

Rigid body math for the 6-DOF robot arm, taken straight from the URDF (joint origins, axes, limits and
link inertias): forward kinematics, geometric Jacobian, its damped least squares inverse and the inverse
dynamics (recursive Newton-Euler, RNEA). It works for any serial chain of revolute joints, not only for
6 of them.

The package builds two libraries:

| target              | class / header                  | depends on                                   | content |
|---------------------|---------------------------------|----------------------------------------------|---------|
| `robotarm_rbd_core` | `RobotarmRbd` (`RobotarmRbd.hpp`) | Eigen, urdfdom, **no ROS**                   | URDF parsing and all calculations. Errors are reported as `false` + `last_error()`, no logging, no allocation on the rt path |
| `robotarm_rbd`      | `Kinematics` (`Kinematics.hpp`)   | `robotarm_rbd_core`, rclcpp, pluginlib, `kinematics_interface` | the `kinematics_interface` plugin: a thin wrapper around `RobotarmRbd` that reads the parameters and logs the errors (throttled on the rt path) |

The inverse dynamics (`RobotarmRbd::recursive_newton_euler`) is only available on `RobotarmRbd`,
`kinematics_interface` has no dynamics. `RobotarmRbd` calls the joint vector `q` (`dq`, `ddq`,
`delta_q`), the wrapper keeps the argument names of `kinematics_interface` (`joint_pos`, `delta_theta`).

```
include/robotarm_rbd/RobotarmRbd.hpp, src/RobotarmRbd.cpp   ROS free model (robotarm_rbd_core)
include/robotarm_rbd/Kinematics.hpp,  src/Kinematics.cpp    kinematics_interface plugin (robotarm_rbd)
kinematics_plugins.xml                                      pluginlib export
test/                                                       gtests, see "Tests"
tools/cartesian_jog/                                        jogging tool, not part of the library
```

Notation: `R(a, q)` rotates by `q` about the unit axis `a`, `O_k` is the origin of joint `k`.
`X_in_Y` is the pose of frame X expressed in frame Y. N is the number of revolute joints. The kinematics
sections count joints from 1 (joint `k` is `joints_[k-1]`); the code and the RNEA section use the 0-based
index `i`, which `q[i]`, `joints_[i]`, `data_.*[i]` and Jacobian column `i` all share: joint `i` and the
link it moves.

## Parameters (read once in `Kinematics::initialize`)

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

## Robot model getters

`Kinematics` hands out what it parsed from the URDF, so callers do not parse it again. All of them
return `false` and leave the output untouched before `initialize` has run through. They allocate, call
them during setup and not from the rt loop.

| function                              | result                                                              |
|---------------------------------------|---------------------------------------------------------------------|
| `get_joint_names(std::vector<std::string> &)` | the N revolute joints in chain order                        |
| `get_joint_limits(std::vector<Limits> &)`     | `min`, `max` [rad], `velocity` [rad/s], `effort` per joint, same order |
| `get_link_names(std::vector<std::string> &)`  | the N+1 links of the moving chain: the root link and the child link of every joint |
| `get_tcp_link_name(std::string &)`    | the child link of the last (fixed) joint                            |

The order is the order of `joint_pos`: entry `i` is joint `i`, which connects link `i` and link `i+1`
(link 0 is the root link). The tcp is neither a joint nor in the link list, but it is a valid `link_name`
for the kinematics functions like any link.

## URDF constraints (checked in `RobotarmRbd::initialize`)

- Serial chain: every link has at most one child. The chain has at least one revolute joint in front of
  the tcp joint; there is no upper limit on the number of joints.
- All joints but the last are `revolute` with valid limits (`lower < upper`, `velocity > 0`, `effort > 0`).
  The last joint is `fixed` and its child is the tcp link. The tcp link may be empty
  (`<link name="tcp"/>`); it is only a frame.
- Every revolute joint has a non-zero `<axis>`. Its length does not matter, it is normalised on parsing.
- Joint origins are arbitrary: any translation and rotation, also for the first joint. (An earlier
  version converted the URDF to DH parameters and therefore required axes along `+z`, an identity first
  origin and DH-conform origins. None of that applies any more.)
- `<inertial>` of the moved links (the child of every revolute joint): **all or none**. Without any, the
  model is kinematics only and the inverse dynamics refuses to work (see "Inverse dynamics"). A model
  where only some links have one is rejected, it would give plausible looking but wrong torques. The
  root link and the tcp link are not moved by any joint, their `<inertial>` is ignored.
- Every given `<inertial>` must be physically plausible: mass `>= 0`, the tensor positive semidefinite,
  the principal moments fulfil the triangle inequality `I_a + I_b >= I_c`, and no inertia without mass
  (tolerance: `1e-6 · trace`). A point mass (zero tensor) and a massless link (zero mass and tensor)
  are fine. Note: urdfdom does not parse `nan`/`inf`, it logs an error and hands out an all-zero
  `<inertial>`, i.e. a massless link.

## Forward kinematics

In a URDF the child link frame of a joint is the joint frame, and the joint rotates about its axis `a_k`
(given in that frame) by `q_k`. Every joint therefore contributes the transform

```
A_k(q_k) = O_k · R(a_k, q_k)            (Joint::transform)
```

`O_k` (`joint_origin_in_parent_`) and the normalised `a_k` (`joint_axis_in_child_`) are read once in
`initialize`. The pose of link `k` and of the tcp in the root link frame are then

```
link_k = A_1(q_1) · A_2(q_2) · ... · A_k(q_k)
tcp    = link_N · O_tcp                 (O_tcp: origin of the fixed tcp joint, tcp_origin_in_parent_)
```

| link_name                       | result of `calculate_link_transform` |
|---------------------------------|--------------------------------------|
| root link                       | identity                             |
| child link of joint `k` (1..N)  | `link_k`                             |
| tcp                             | `link_N · O_tcp`                     |

The rotation `R(a_k, q_k)` is about an axis through the origin of frame `k`, so it does not move that
origin: the position of joint `k` is the translation of `link_k`, whatever `q_k` is.

## Jacobian and its damped pseudo-inverse

`calculate_jacobian` returns the geometric Jacobian `J` (6xN) of a link in the root link frame:
column `i` is `(w_i × (p_end − p_i), w_i)`, with the position `p_i` of joint `i` (translation of `link_i`)
and its axis in the root link frame, `w_i = rot(link_i) · a_i`. Only for a URDF with every axis along
`+z` is that the z column of `rot(link_i)`.
It maps joint velocities to a twist, `x_dot = J · q_dot` (rows 0-2 linear in m/s, rows 3-5 angular in rad/s).
Joints behind the requested link get a zero column.

The chain is multiplied up once while the columns are filled, and `p_end` comes from one call to
`calculate_link_transform` beforehand, so a Jacobian costs two passes over the chain (O(N)) instead of one
transform per column (O(N²)).

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

## Inverse dynamics (recursive Newton-Euler)

```cpp
bool recursive_newton_euler(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const Eigen::VectorXd &ddq,
                            Eigen::VectorXd &tau,
                            const Eigen::Vector3d &F_tcp = 0, const Eigen::Vector3d &M_tcp = 0,
                            const Eigen::Vector3d &gravity = (0, 0, -9.81));
```

Given the motion (q, q̇, q̈) and the wrench the tool exerts on its environment, it returns the joint
torques τ that produce exactly this motion: `τ = M(q) q̈ + C(q, q̇) q̇ + g(q) + Jᵀ(q) w_tcp`. It never
builds `M`, `C` or `J`, it costs O(N) and does not allocate if `tau` is pre-sized.

| use                         | call                                                     |
|-----------------------------|----------------------------------------------------------|
| feedforward torque          | the planned (q, q̇, q̈)                                    |
| gravity compensation        | (q, 0, 0)                                                |
| column j of the mass matrix | `gravity = 0`, q̇ = 0, q̈ = e_j                             |
| static wrench → torque      | `gravity = 0`, q̇ = q̈ = 0, only `F_tcp`/`M_tcp` (= Jᵀ w)   |

On a kinematics only model (no `<inertial>`, see "URDF constraints") every link would be massless and
τ would silently contain nothing but the TCP wrench. The call therefore fails with "Not all links have a
valid inertia", except for the last row of the table: that is the only case where the result does not
depend on the masses.

### 1. The idea

A link cannot know which force its joint has to deliver before it knows how it moves, and it cannot know
the load of its joint before it knows what the links further out push on it. Both are chained:

- **Forward pass, base → tip (motion)**: the base is fixed (ω = ω̇ = 0). The motion of link i follows from
  the motion of link i-1 plus what joint i adds: ω_i, ω̇_i, the acceleration of the frame origin and of
  the centre of mass (CoM).
- **Backward pass, tip → base (loads)**: from its acceleration, Newton (`F = m a`) and Euler
  (`N = I ω̇ + ω × I ω`) give the force and moment link i needs. Joint i has to deliver that plus what the
  next link (or at the tip, the environment) pushes back with. Its component about the joint axis is τ_i.

Each step only needs the neighbour's result, so every link is visited twice: O(N).

### 2. Frames and index convention

URDF convention, shared with the kinematics:

- the child link frame of joint i **is** frame i (the joint frame). All quantities of link i are
  expressed in frame i, so the forward pass works in link frames, never in the base frame.
- `T[i] = joints_[i].transform(q[i]) = O_i · R(a_i, q_i)`: pose of frame i in frame i-1.
  `R_i = T[i].linear()` maps vectors from frame i to frame i-1 (child → parent), `R_iᵀ` from i-1 to i.
  The forward pass uses `Rt = R_iᵀ` (moving the motion outwards), the backward pass `R_next` =
  `R_{i+1}` (moving the loads of link i+1 inwards).
- `p_i = T[i].translation()`: the vector from origin i-1 to origin i, **expressed in frame i-1**. Because the
  joint rotation acts after the origin (in the child frame), p_i does not depend on q_i.
- **Index pitfall**: physically p_i is a piece of link i-1 (it connects its two joints), but the URDF stores
  it in joint i. So the link from origin i to origin i+1 is `T[i+1].translation()`, not `T[i]`. Using
  `T[i-1]` instead of `T[i]` in the forward pass is an easy mistake (and was one here).
- `a_i = joint_axis_in_child_` is the URDF `<axis>`, given in frame i and constant there (normalised on
  parsing, `AngleAxisd` needs a unit axis). Since the joint rotates about a_i itself, `R(a, q) a = a`: the
  axis has the same coordinates in frame i whatever q_i is, so it never has to be recomputed.
- The base itself is not stored: it only provides the start values of the forward pass.

**Why only rotations**: ω, ω̇, accelerations, forces and moments are *free vectors*: they have a
direction and a length, but are not points. Changing the frame they are written in only turns their
components (`v_i = R_iᵀ v_{i-1}`). The full transform `T⁻¹` would also subtract a translation, which only
makes sense for points (positions). That something acts at a *different point* (origin i instead of origin
i-1) is a physical effect, not a change of coordinates, and it is written out explicitly: the `ω × p`
terms of the accelerations and the `p × f` terms of the moments.

### 3. Angular velocity and acceleration: why they are not simply added

```
ω_i  = R_iᵀ ω_{i-1} + q̇_i a_i
ω̇_i  = R_iᵀ ω̇_{i-1} + q̈_i a_i + ω_i × (q̇_i a_i)
```

In a common (inertial) frame angular velocities of a chain *do* add: `ω_i = ω_{i-1} + q̇_i z_i`, with z_i the
joint axis in that frame. The `R_iᵀ` only rewrites ω_{i-1} in frame i.

The acceleration does not follow by just adding q̈_i z_i, because z_i itself moves. Product rule in the
inertial frame:

```
ω̇_i = ω̇_{i-1} + q̈_i z_i + q̇_i ż_i
```

z_i is fixed in link i-1 (the joint housing), and the time derivative of any body-fixed vector is
`ż = ω × z` (see 4.). So `ż_i = ω_{i-1} × z_i` and the extra term is `ω_{i-1} × (q̇_i z_i)`. The code
writes `ω_i × (q̇_i a_i)` instead, which is the same: `ω_i × z = (ω_{i-1} + q̇_i z) × z = ω_{i-1} × z`,
since `z × z = 0` (z is also fixed in link i). This term is the change of direction of the joint's spin
because the whole joint is being turned by the links before it (e.g. a spinning wrist that is swung by
the shoulder).

### 4. Linear acceleration

```
a_org,i = R_iᵀ ( a_org,i-1 + ω̇_{i-1} × p_i + ω_{i-1} × (ω_{i-1} × p_i) )
a_com,i = a_org,i + ω̇_i × r_com,i + ω_i × (ω_i × r_com,i)
```

In the inertial frame `o_i = o_{i-1} + p_i`. p_i is fixed in link i-1, so it only changes its direction
by the rotation of link i-1.

**Derivative of a body-fixed vector**. In 2D, a vector of length L at angle φ is `p = L (cos φ, sin φ)`, so
`ṗ = L φ̇ (−sin φ, cos φ)`: perpendicular to p, with length `L φ̇`. That is exactly `(0, 0, φ̇) × p`. In 3D,
`p = R p_body` with a constant p_body, `ṗ = Ṙ p_body` and `Ṙ = [ω]× R` (the skew matrix of ω), so

```
ṗ = ω × p
```

**Second derivative**, product rule for the cross product (the order must be kept, `a × b = −b × a`):

```
p̈ = ω̇ × p + ω × ṗ = ω̇ × p + ω × (ω × p)
```

- `ω̇ × p`, the **tangential** acceleration: perpendicular to p, from speeding up the rotation.
- `ω × (ω × p)`, the **centripetal** acceleration: with the identity `a × (b × c) = b (a·c) − c (a·b)`
  it is `ω (ω·p) − |ω|² p = −|ω|² p_⊥`, where p_⊥ is the part of p perpendicular to ω. It points from the
  point towards the rotation axis, with the familiar magnitude `|ω|² r`.

The part of p along ω does not rotate and drops out. Everything in the bracket is written in frame i-1,
then turned into frame i by R_iᵀ. Joint i itself (q̇_i, q̈_i) does not appear in a_org,i: origin i lies on
axis i, the joint does not move it. The CoM of link i is fixed in link i at r_com,i (frame i), so the same
argument inside link i, with ω_i and ω̇_i, gives a_com,i.

### 5. Inertia: reference point and axes

Newton and Euler separate cleanly only about the centre of mass: `F = m a_com` and
`N_com = I_com ω̇ + ω × (I_com ω)`. About any other point, extra coupling terms appear. So each link stores
(`InertialParams`):

- `mass`, `com` = r_com in frame i,
- `com_inertia_in_joint`: the inertia tensor **about the CoM**, with the **axes of frame i**. Axes fixed to
  the link make the tensor constant; in the inertial frame it would change with every motion.

The URDF `<inertial>` gives `<origin xyz>` = the CoM in the link frame and `<origin rpy>` = the orientation
R_com of the *inertial frame* in which `<inertia>` is written (often the principal axes). Off-diagonal
entries (`ixy`, `ixz`, `iyz`) are the matrix entries themselves, no sign flip. The tensor is rotated once
on parsing:

```
I' = R_com I R_comᵀ
```

Derivation: in inertial frame axes `L_c = I ω_c`. A vector turns as `ω_c = R_comᵀ ω'`, `L' = R_com L_c`, so
`L' = R_com I R_comᵀ ω'`. Only the axes turn, the reference point stays the CoM, so no Steiner (parallel
axis) term: the offset of the CoM from the joint origin enters the moment balance explicitly (8.).

### 6. Gravity: the base accelerates upwards

A link at rest in gravity still needs a joint force: `F_joint = m (a − g)`. Instead of adding the weight to
every link, the forward pass starts with

```
a_org,base = −g          (= (0, 0, +9.81) for the default g = (0, 0, −9.81))
```

as if the base were an elevator accelerating upwards with 9.81 m/s² in free space. By the equivalence
principle a body cannot tell that from standing still in gravity. The acceleration of the base is passed on
unchanged (only rotated) through every a_org and into every a_com, so all of them contain −g, expressed in
each link's own frame for free, and `m a_com` already includes the weight. The backward pass has no gravity
term.

- **Pitfall**: adding −g to a_org of *every* link does not work: a_org,i is passed on to link i+1, so link k
  would end up with k times gravity.
- **Alternative**: keep a_org purely kinematic and add the gravity to a_com only (it is not passed on),
  `F_dyn = m (a_com − R_{0,i}ᵀ g)`. That needs the orientation R_{0,i} of every link in the base frame,
  which the trick provides implicitly.
- **Consequence**: `data_.a_org` and `data_.a_com` are *not* the kinematic accelerations; they are shifted
  by −g.

### 7. Backward pass: free body diagram

```
F_dyn,i = m_i a_com,i
f_i     = F_dyn,i + R_next f_next
M_i     = I_com,i ω̇_i + ω_i × (I_com,i ω_i) + r_com,i × F_dyn,i + R_next M_next + p_next × (R_next f_next)
τ_i     = M_i · a_i
R_next, p_next ← T[i];   f_next, M_next ← f_i, M_i
```

Sign convention (Craig): f_i, M_i are the force and moment exerted **on link i by link i-1**, at origin i,
in frame i. Cut link i free. What acts on it:

- f_i, M_i from link i-1 (at origin i),
- −f_{i+1}, −M_{i+1} from link i+1, at origin i+1: link i pushes on link i+1 with f_{i+1}, so link i+1 pushes
  back with −f_{i+1} (actio = reactio). f_{i+1} lives in frame i+1, `R_next = R_{i+1}` turns it into frame i,
- gravity, already inside a_com (6.).

Newton for link i: `f_i − R_next f_next = m_i a_com,i`, which gives the force line.

### 8. Moment balance about the joint origin

Euler `I ω̇ + ω × (I ω)` holds only about the CoM. Two rules for moving loads around:

- a **pure moment** (couple) is a free vector: it has the same effect about every point, and can be moved
  without extra terms,
- a **force** has a line of action: about another point it adds the moment `lever × force`.

**About the CoM** (lever arms from the CoM: −r_com,i to origin i, p_next − r_com,i to origin i+1):

```
M_i − R M_next + (−r_com) × f_i + (p_next − r_com) × (−R f_next) = I ω̇ + ω × (I ω)
```

Inserting `f_i = F_dyn + R f_next`, the r_com × R f_next terms cancel and what remains is the moment line
of 7. In this form f_i appears with the lever −r_com.

**About origin i** (what the code writes): f_i acts at origin i and has no lever, it drops out. On the
left side the inertia force `F_dyn` of the CoM, which has no moment about the CoM, now has the lever r_com:

```
M_i = I ω̇ + ω × (I ω)     + r_com × F_dyn     + R M_next    + p_next × (R f_next)
      ─────   ──────────     ──────────────      ─────────     ───────────────────
      rot.    gyroscopic     lever moment of     reaction      lever moment of the
      inertia moment         the inertia force   moment from   successor's reaction
                             (incl. gravity)     the successor force
```

M_i itself is the joint moment link i gets from its predecessor. Both forms are the same balance; what
matters is to use one reference point consistently. The d'Alembert view: treat `−F_dyn` and
`−(I ω̇ + ω × I ω)` as extra loads (inertia force at the CoM, inertia moment), then the link is in static
equilibrium, and the moment balance about origin i gives the same line.

**Example**: a horizontal beam of length L and mass m, hinged at one end, at rest. ω = ω̇ = 0, so the Euler
part is `N = 0`, yet the hinge obviously has to hold `m g L/2`. That is the lever term: a_com = −g (6.),
`r_com × F_dyn = (L/2, 0, 0) × (0, 0, m g)` has magnitude `m g L/2`.

### 9. Motor torque: τ = M · a

```cpp
tau[i] = data_.M[i].dot(ax);
```

A revolute joint can only drive about its own axis. The component of M_i along a_i is the motor torque.
The two other moment components and the whole force f_i are carried by the bearings and the structure; the
RNEA computes them anyway (`data_.F`, `data_.M`), useful for sizing bearings.

### 10. Wrench at the TCP

`F_tcp`, `M_tcp` are the force and moment the robot exerts **on** the environment, at the tcp origin, in
the tcp frame. A force/torque sensor at the flange measures the reaction, the opposite sign. The tcp acts
like one more link without mass after the last one, so the backward pass starts with

```
R_next = rot(O_tcp),  p_next = trans(O_tcp),  f_next = F_tcp,  M_next = M_tcp
```

and the first iteration adds to the last link `f = R F_tcp` and `M = R M_tcp + p × (R F_tcp)`: the force
turned into its frame, and the moment turned plus the lever moment of the force.

### 11. Robustness and feasibility

- The RNEA inverts nothing: there is no matrix to become singular, and kinematic singularities do not
  bother it. Any (q, q̇, q̈) gives finite torques. Every joint trajectory is kinematically possible, only the
  joint limits (position, velocity, torque) restrict it.
- Huge torques come from the inputs: Cartesian trajectories through or near a singularity (where `J⁻¹` turns
  small Cartesian speeds into huge joint speeds and accelerations), or jumps in q̇ (infinite q̈).
- At a singularity `Jᵀ` has a null space: some TCP wrenches (e.g. pushing along a fully stretched arm)
  need no motor torque at all, the structure carries them.
- Checking `|τ_i| <= effort_i` over a trajectory tells whether it is feasible. (The URDF efforts are still
  placeholders.)

### 12. Implementation notes

- `data_` (`RobotarmData`) is a private scratch buffer sized once in `initialize()`: T, ω, ω̇, a_org, a_com, f,
  M per link. The inputs are arguments, not stored. With a pre-sized `tau` the call does not allocate
  (checked by `kinematics_malloc_test`). Not thread safe.
- Eigen pitfalls: `a.dot(b)` is the scalar product; `a.transpose() * b` is a 1x1 matrix; `a * b` for two
  `Vector3d` does not compile (element-wise is `cwiseProduct`). `a.cross(b)` keeps the order of the
  formula, `b.cross(a)` flips the sign.
- The backward loop runs from N-1 down to 0 with a signed `int`: with `size_t` the condition `i >= 0` is
  always true and the loop never ends.

| formula (Craig style)       | code                                              |
|-----------------------------|---------------------------------------------------|
| ⁱ⁻¹Rᵢ, ⁱRᵢ₋₁               | `data_.T[i].linear()`, `Rt` (forward); `R_next` (backward) |
| p_i = ⁱ⁻¹Pᵢ                 | `r = data_.T[i].translation()`; `r_next`           |
| ẑ_i                         | `ax = joints_[i].joint_axis_in_child_`            |
| ω_i, ω̇_i                    | `data_.w[i]`, `data_.dot_w[i]`                    |
| v̇_i (+ gravity)            | `data_.a_org[i]`                                  |
| v̇_Ci (+ gravity)           | `data_.a_com[i]`                                  |
| m_i, P_Ci, ᶜI_i             | `inertia_.mass`, `inertia_.com`, `inertia_.com_inertia_in_joint` |
| F_i = m v̇_C                 | `F_dyn`                                           |
| f_i, n_i                    | `data_.F[i]`, `data_.M[i]`                        |
| f_{N+1}, n_{N+1}            | `F_tcp`, `M_tcp`                                  |
| τ_i = n_iᵀ ẑ_i              | `tau[i] = data_.M[i].dot(ax)`                     |

## Cartesian jogging (`cartesian_jog`), a tool

Everything above is the library (`include/`, `src/`, `test/`). Everything about jogging lives in
`tools/cartesian_jog/` and is not part of the library: `cartesian_jog_node.cpp`, the joint limit policy
`JointLimiter.hpp` (header only, only used by the node, not installed) with its test
`joint_limiter_test.cpp`, the slider GUI `cartesian_jog_gui.py` and `cartesian_jog.launch.py`.

A node to try Cartesian speed jogging in RViz, without ros2_control. It takes the place of
`joint_state_publisher_gui` in the display setup, and a second small program with six sliders
takes the place of its sliders:

```
cartesian_jog_gui (6 sliders)  ->  cmd_vel (Twist)  ->  cartesian_jog  ->  /joint_states
                                                       ->  robot_state_publisher  ->  TF  ->  RViz
```

```
ros2 launch robotarm_rbd cartesian_jog.launch.py                 # sliders + RViz
ros2 launch robotarm_rbd cartesian_jog.launch.py gui:=false      # without the sliders
ros2 launch robotarm_rbd cartesian_jog.launch.py rviz:=false     # without RViz
ros2 launch robotarm_rbd cartesian_jog.launch.py lambda:=0.01    # more accurate, see below

# without the sliders: move the tool along base +x at 5 cm/s (publish continuously,
# the node stops 0.2 s after the last message)
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}}"
```

The launch file starts `robot_state_publisher`, the node, the slider GUI and RViz with the config of
`robotarm_description` (fixed frame `Base_1`). Do not run `joint_state_publisher_gui` next to it, both
would publish `/joint_states`.

- **Slider GUI** (`cartesian_jog_gui`, Qt like `joint_state_publisher_gui`): x, y, z in m/s and the rotation
  rates wx, wy, wz about the base axes in rad/s, range `max_linear` (0.1 m/s) and `max_angular` (0.5 rad/s)
  (parameters). A slider **springs back to zero when released**, so letting go stops the tool at once.
  "Keep values on release" leaves the sliders where they are, "Zero all" resets them. It only publishes
  while a slider is away from zero (plus one final zero), so it does not fight a `ros2 topic pub`.
  `joint_state_publisher_gui` itself cannot be used for this: it builds one slider per joint of a URDF and
  publishes `JointState`, and its sliders stay where they are left.

- **Input**: `cmd_vel` (`geometry_msgs/Twist`), the velocity of the tcp in the **base frame**: linear in
  m/s, angular in rad/s (rotation about the base axes). For jogging along the tool axes the twist would first
  have to be rotated by the tcp rotation from `calculate_link_transform`.
- **What it does**: every cycle `dx = twist · dt` goes through `convert_cartesian_deltas_to_joint_deltas`
  of the plugin, which is loaded through pluginlib (`kinematics_plugin`), like a ros2_control controller would.
  Then the joint step is limited (below), added to the joint positions and published. Purely kinematic:
  no controller, no dynamics.
- **Parameters**: `robot_description`, `kinematics_plugin` (`robotarm_rbd/Kinematics`),
  `rate` (100 Hz), `twist_timeout` (0.2 s), `initial_joint_positions`, `lambda` (launch argument, default 0.05).
- **Watchdog**: the last twist is applied until it is `twist_timeout` old. Nothing older is used, and the
  tool coasts for at most that long after the last message (at 0.05 m/s: up to 1 cm).
- **Robot model**: joint names, joint limits and the tcp link come from the plugin (robot model getters
  above), the node does not parse the URDF itself. That is why it needs a `Kinematics` and links the
  library.
- **Joint limits** (`JointLimiter.hpp`), applied to the whole joint step: it is scaled by
  ONE factor, so the joints keep their ratio and the tool moves along the commanded direction, only slower.
  The tightest joint decides: a joint at its velocity limit slows everything down, and a joint reaching
  its position range ends the step exactly at the limit. A joint at a limit that would move further out
  stops the whole motion; moving back in is always possible. The node logs which joint limits the motion
  (`Velocity limit of axis3: motion scaled to 2 %`).
- **Start pose**: parameter `initial_joint_positions` (must be inside the joint limits). Avoid wrist
  singularities such as `q5 = 0`, where some twists cannot be followed.
- **Near the reach boundary** (arm almost stretched) the elbow needs very large speeds, so the velocity limit
  scales the whole motion down towards zero: the tool creeps instead of stopping. That is the damped
  inverse and the limit doing their job, not a hang.
- **`lambda`** trades accuracy against safety near singularities. At `q = (0, -1.3, 1.5, 0, 1.0, 0)` the tool moves at 93-95 %
  of the commanded speed with `lambda = 0.05`, at 99.7 % with `0.01` (rotation: 99.5 % / 100 %).

## Tests

```
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robotarm_description robotarm_rbd
source install/setup.bash
colcon test --packages-select robotarm_rbd --event-handlers console_direct+
colcon test-result --test-result-base build/robotarm_rbd --verbose
```

The workspace must be sourced and `robotarm_description` built: the real URDF is processed with
`xacro` (a missing package or `xacro` fails the tests, it is not skipped). Tests need
[Pinocchio](https://github.com/stack-of-tasks/pinocchio) (`ros-jazzy-pinocchio`, a `test_depend`, also
in `.devcontainer/Dockerfile`); it is only looked for with `BUILD_TESTING`, users of the library do not
need it (`colcon build --cmake-args -DBUILD_TESTING=OFF` builds without it). Five gtest executables, also
runnable directly from `build/robotarm_rbd/` (with `--gtest_filter=...`):

| executable                | links rclcpp | covers                                                                                  |
|---------------------------|--------------|-----------------------------------------------------------------------------------------|
| `kinematics_test`         | yes | URDF parsing (origins, normalised axes, limits, chain length), link transforms against an independent reference FK built from the URDF joint origins, Jacobian against finite differences, geometry the old DH parser refused being accepted and computed correctly, `initialize()` rejecting every invalid URDF (one case per rule), delta conversions and input validation, round-trip accuracy of the damped inverse |
| `robotarm_rbd_test`       | no  | lifecycle and error messages of the ROS free `RobotarmRbd` (fails to link if it pulls in ROS), `<inertial>` parsing and rotation `R I Rᵀ`, the inertia rules (all or none, implausible tensors rejected), inverse dynamics solved by hand: pendulum `τ = (I + mL²) q̈ − m g L cos q`, TCP wrench in a rotated tcp frame, vertical first axis, refusal without inertia |
| `rnea_pinocchio_test`     | no  | the real robot against Pinocchio: same joints and masses, static gravity (also against `−Σ m J_comᵀ g` from our FK), mass matrix from unit accelerations (symmetric, positive definite, = `crba`), 500 random (q, q̇, q̈) = `rnea`, TCP wrench = `rnea` with `fext` (sign flipped, moved to the joint 6 frame) and = `Jᵀ w`, FK and Jacobian of every link = Pinocchio |
| `kinematics_malloc_test`  | yes | the rt path (kinematics and inverse dynamics) does not allocate with pre-sized outputs (Eigen's malloc guard), with negative controls so the guard cannot be silently off |
| `joint_limiter_test`      | no  | the joint limit policy of the jog tool (`tools/cartesian_jog/JointLimiter.hpp`): velocity and position limits, uniform scaling, joints at / beyond a limit, invalid input |

The round-trip test asserts the exact bound of the damped inverse, `|J⁺J dq − dq| ≤ λ² / (σ_min² + λ²) · |dq|`
(same for `J J⁺`), not an arbitrary tolerance. With fewer than 6 joints only the `dq` round trip is
bounded, with more than 6 only the `dx` round trip. The kinematics tests run on the real robot and on
synthetic URDFs (`test/test_utils.hpp`): two generated from a DH table (the one of the robot, and one with
arbitrary twists, offsets and negative lengths) and random chains of 1, 3, 6 and 7 joints with arbitrary
origins and oblique axes of any length. The synthetic URDFs have no `<inertial>` (kinematics only).
`test/real_urdf.hpp` loads the real URDF without rclcpp.

The Pinocchio comparison agrees to about 1e-14 Nm (torques up to ~30 Nm); the tests allow 1e-9. Pinocchio
merges the inertia of links on fixed joints into their parent: the tcp link has none, the root link goes
to the (fixed) universe, so both models describe the same bodies.

`kinematics_malloc_test` compiles `src/Kinematics.cpp` and `src/RobotarmRbd.cpp` itself instead of
linking the libraries, because Eigen's malloc guard has to be compiled into the code under test.
`ament_uncrustify` and `ament_cpplint` are disabled in `CMakeLists.txt`; `ament_cppcheck` skips itself
on this cppcheck version.
