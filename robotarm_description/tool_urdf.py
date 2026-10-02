"""Export one end-effector tool (gripper etc.) from its own Onshape assembly as urdf/tools/<TOOL_NAME>.xacro.

Same flow as robotarm_urdf.py, only the last step differs: convert_tool_urdf_to_xacro writes the
<xacro:macro name="tool" params="parent"> that robotarm.urdf.xacro hangs on the flange (tool:=<TOOL_NAME>).
Calls the Onshape API, which costs tokens: only run it when the tool's CAD changed.

After the run (see README, "Tools on the flange"):
  cp output_<TOOL_NAME>/<TOOL_NAME>.xacro urdf/tools/
  cp -r output_<TOOL_NAME>/meshes/tools meshes/
  colcon build --packages-select robotarm_description
"""

from pathlib import Path

from onshape_robotics_toolkit import ORTConfig
from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.formats.urdf import URDFSerializer
from onshape_robotics_toolkit.formats.xacro import convert_tool_urdf_to_xacro
from onshape_robotics_toolkit.graph import KinematicGraph
from onshape_robotics_toolkit.mesh import MeshOptions
from onshape_robotics_toolkit.parse import CAD
from onshape_robotics_toolkit.robot import Robot
from onshape_robotics_toolkit.utilities import setup_default_logging

# The tool's own Onshape assembly (not the arm's).
DOCUMENT_URL = "https://cad.onshape.com/documents/<document>/w/<workspace>/e/<element>"

# File name in urdf/tools/ (without .xacro), the value of the xacro arg tool:=..., the prefix of all
# its link/joint names and its mesh folder meshes/tools/<TOOL_NAME>/.
TOOL_NAME = "gripper_v1"

# Name of the FASTENED mate between the assembly's Origin and the part that mounts on the flange,
# like ROOT_MATE_NAME in robotarm_urdf.py.
ROOT_MATE_NAME = "root_mate"

# ROS package the tool lives in - used for package://<PACKAGE_NAME>/meshes/tools/... paths.
PACKAGE_NAME = "robotarm_description"

# Pose of the tool's root link in the flange frame (meters / radians). Zero if the mounting part's
# origin sits on the mounting face with z pointing away from the flange; check in RViz after the
# first export.
ATTACH_XYZ = (0.0, 0.0, 0.0)
ATTACH_RPY = (0.0, 0.0, 0.0)

# Link the tcp attaches to (unprefixed name as in the exported URDF, None = the root link) and the
# tcp pose in that link's frame (meters / radians).
TCP_PARENT_LINK = None
TCP_XYZ = (0.0, 0.0, 0.0)
TCP_RPY = (0.0, 0.0, 0.0)

# Everything goes to its own folder, so the arm export in output/ is not overwritten. The mesh folder
# must be called "meshes": convert_tool_urdf_to_xacro finds the subfolders (collision/) by that name.
OUTPUT_DIR = Path(f"output_{TOOL_NAME}")
URDF_PATH = str(OUTPUT_DIR / f"{TOOL_NAME}.urdf")
MESH_DIR = str(OUTPUT_DIR / "meshes")
XACRO_PATH = str(OUTPUT_DIR / f"{TOOL_NAME}.xacro")

setup_default_logging(file_path=f"{TOOL_NAME}.log", console_level="INFO")

# Optional per-tool overrides (names, limits of the jaw mates), like ORT.yaml for the arm.
ORT_CONFIG_PATH = Path(f"ORT_{TOOL_NAME}.yaml")
if ORT_CONFIG_PATH.exists():
    ORTConfig.load(ORT_CONFIG_PATH)

client = Client(env=".env")
cad = CAD.from_url(DOCUMENT_URL, client=client, max_depth=0)
graph = KinematicGraph.from_cad(cad, root_mate_name=ROOT_MATE_NAME)
robot = Robot.from_graph(kinematic_graph=graph, client=client, name=TOOL_NAME, fetch_mass_properties=True)

URDFSerializer().save(robot,
                      URDF_PATH,
                      download_assets=True,
                      mesh_dir=MESH_DIR,
                      mesh_options=MeshOptions(
                        visual_max_faces=100_000,                # decimate visuals; None = keep full resolution
                        collision_mode="convex_decomposition",   # "convex_hull" | "convex_decomposition" | "mesh"
                        collision_max_hulls=16,                  # convex pieces per link (decomposition only)
                        collision_hull_max_vertices=64,          # vertices per convex piece
                        collision_concavity=0.05,                # lower = tighter fit, more pieces
                      ))

convert_tool_urdf_to_xacro(
    URDF_PATH,
    XACRO_PATH,
    package_name=PACKAGE_NAME,
    mesh_dir=MESH_DIR,
    tool_name=TOOL_NAME,
    attach_xyz=ATTACH_XYZ,
    attach_rpy=ATTACH_RPY,
    tcp_parent_link=TCP_PARENT_LINK,
    tcp_xyz=TCP_XYZ,
    tcp_rpy=TCP_RPY,
)
