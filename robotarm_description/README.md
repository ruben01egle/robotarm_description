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

## Updating to newer fork commits

```bash
.venv/bin/pip install --force-reinstall --no-deps git+https://github.com/ruben01egle/onshape-robotics-toolkit.git@main
```
