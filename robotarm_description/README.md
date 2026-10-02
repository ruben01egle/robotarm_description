# URDF export

`robotarm_urdf.py` regenerates the URDF/xacro from the Onshape CAD assembly using
[onshape-robotics-toolkit](https://github.com/ruben01egle/onshape-robotics-toolkit) (this fork,
not the upstream package).

## Setup

From this directory:

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

Create a `.env` file here with your Onshape API keys (generate them at
https://dev-portal.onshape.com/keys):

```
ONSHAPE_ACCESS_KEY=...
ONSHAPE_SECRET_KEY=...
```

## Run

```bash
.venv/bin/python robotarm_urdf.py
```

Output goes to `./output/` (URDF, meshes, xacro), relative to wherever you run it from.

## Tools on the flange

The arm export ends in a fixed `flange_joint` → `flange` link (`FLANGE_XYZ` / `FLANGE_RPY` in
`robotarm_urdf.py`). Tools are mounted there and chosen with the xacro arg `tool`:

```bash
xacro urdf/robotarm.urdf.xacro tool:=<name>          # default: none
ros2 launch robotarm_description display_test.launch.py tool:=<name>
```

`tool` is a file name in `urdf/tools/` without `.xacro`. Every tool file defines the same macro,
`<xacro:macro name="tool" params="parent">`, which hangs the tool on `${parent}` and contains the `tcp`
link. `tools/none.xacro` puts the tcp directly on the flange.

To add a tool exported from its own Onshape assembly with `convert_tool_urdf_to_xacro`:

1. copy the xacro to `urdf/tools/<name>.xacro`
2. copy the meshes to `meshes/tools/<name>/` (the export already writes them in that layout)
3. rebuild: `colcon build --packages-select robotarm_description`

`robotarm_rbd` reads the tool from the URDF: the tcp is wherever the tool puts it, and the tool's mass is
folded into the last moving link (see its README, "Tool behind the flange").

## Updating to newer fork commits

```bash
.venv/bin/pip install --force-reinstall --no-deps git+https://github.com/ruben01egle/onshape-robotics-toolkit.git@main
```
