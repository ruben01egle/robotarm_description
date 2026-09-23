# workspace_envelope

Visualizes the robot's reachable workspace envelope (side view + top view),
in the style of a manufacturer's working-envelope diagram: samples joint
angles within the URDF's limits, computes forward kinematics for a chosen
reference link (default: `tcp`), and plots the swept-out region. Optionally
overlays the robot's STL meshes at the home pose for scale/context.

Two rendering styles (`--style`):
- `ours` (default): fills in every point the reference link reaches over all
  joints that affect its position -- a dense, Monte Carlo-sampled silhouette.
- `kuka`: mimics a manufacturer manual's diagram -- outlines, not filled.
  The side view only sweeps the joints that pitch within the arm's own
  vertical plane (auto-detected: whichever revolute joints' rotation axis is
  parallel to world Y at the zero pose -- axis2/3/5 on this robot), holding
  the base yaw and any roll joints at zero, since panning the whole arm
  around its yaw axis doesn't change that 2D profile. Its outline is not
  traced from scattered samples (a contour of a sampled point cloud picks up
  every by-chance empty bin as a hole, and FK stretches joint space so
  unevenly that no single sample count/bin size works everywhere). Instead
  those joints are swept on a regular grid (`--samples` total nodes), every
  2D face of that joint-space grid is split into triangles, and the
  triangles' FK images are filled into a raster (`--outline-resolution`
  pixels). FK is continuous, so the union of those small triangles covers
  the reachable region without gaps however much FK stretches it, and the
  traced contour is clean -- while still picking up real concave detail and
  holes, like the pocket near the shoulder the arm can't reach.
  The top view is just the outline swept by the base yaw joint (axis1) at
  the max radius found by the side sweep, i.e. a circle with a wedge missing
  where the joint's limit cuts it off, matching e.g. a KUKA manual's
  top-view drawing.
- `both`: renders both, stacked in one figure.

FK is computed independently in this script (standard URDF joint
composition via numpy) rather than through `robotarm_kinematics`'s
`KinematicsCore`, since that C++ class requires a live `rclcpp` node to
initialize and has no Python bindings.

## Dependencies

`numpy`, `xacro`, and `ament_index_python` come from the ROS2 Jazzy
environment (source the workspace before running). `matplotlib` and
`trimesh` are installed system-wide via `.devcontainer/Dockerfile`
(`python3-matplotlib` from apt; `trimesh` via
`pip3 install --break-system-packages trimesh`, since it has no apt
package). If running in a container built before these were added, rebuild
it, or install both directly with the same two commands.

## Usage

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash   # from the workspace root; needed so xacro/ament_index_python can find robotarm_description

python3 robotarm_description/tools/workspace_envelope/workspace_envelope.py
# -> workspace_envelope.png (TCP envelope, "ours" style)

# KUKA-manual style instead, wrist center instead of TCP, no mesh:
python3 robotarm_description/tools/workspace_envelope/workspace_envelope.py \
    --style kuka --reference-link Stage5_1 --no-mesh --output kuka_envelope.png -v

# Both styles stacked in one figure:
python3 robotarm_description/tools/workspace_envelope/workspace_envelope.py --style both
```

Run with `--help` for the full list of options (reference link, sample
count/strategy, bin resolution, output path, etc).
