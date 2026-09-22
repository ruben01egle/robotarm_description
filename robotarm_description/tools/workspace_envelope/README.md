# workspace_envelope

Visualizes the robot's reachable workspace envelope (side view + top view),
in the style of a manufacturer's working-envelope diagram: samples joint
angles within the URDF's limits, computes forward kinematics for a chosen
reference link (default: the wrist center at the intersection of axes 4/5),
and plots the swept-out region. Optionally overlays the robot's STL meshes
at the home pose for scale/context.

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
# -> workspace_envelope.png

# TCP envelope instead of the wrist center, more samples, no mesh:
python3 robotarm_description/tools/workspace_envelope/workspace_envelope.py \
    --reference-link tcp --samples 500000 --no-mesh --output tcp_envelope.png -v
```

Run with `--help` for the full list of options (reference link, sample
count/strategy, bin resolution, output path, etc).
