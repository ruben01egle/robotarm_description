from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.formats.urdf import URDFSerializer
from onshape_robotics_toolkit.formats.xacro import convert_urdf_to_xacro
from onshape_robotics_toolkit.graph import KinematicGraph
from onshape_robotics_toolkit.parse import CAD
from onshape_robotics_toolkit.robot import Robot
from onshape_robotics_toolkit.utilities import setup_default_logging

DOCUMENT_URL = "https://cad.onshape.com/documents/cd1ea15e5c158db2ad62771b/w/ca3c8c5dbd7650aa0ac06273/e/69821d80e21d0a495c75f831"

# Name of the top-level FASTENED mate between the assembly's own Origin and the Base
# subassembly (see README: "Root selection via a named mate-to-origin"). Replaces the old
# workflow of adding a dummy fixed part just to anchor the kinematic root.
ROOT_MATE_NAME = "root_mate"

# ROS package this URDF/xacro will live in - used for package://<PACKAGE_NAME>/meshes/... paths.
PACKAGE_NAME = "robotarm_description"

# Legacy escape hatch: only needed if the root link is still a throwaway dummy part (the
# old workflow, before ROOT_MATE_NAME). Leave False once the dummy part is deleted from
# Onshape and Base is the real root link - stripping it would delete real robot geometry.
STRIP_ROOT_LINK = False

# Link the tcp frame attaches to, and its offset (meters) along that link's own Z axis.
TIP_LINK = "Stage6_1"
TCP_OFFSET_Z = 0.148

URDF_PATH = "output/robotarm.urdf"
MESH_DIR = "output/meshes"
XACRO_PATH = "output/robotarm.urdf.xacro"

setup_default_logging(file_path="robotarm.log", console_level="INFO")

client = Client(env=".env")
cad = CAD.from_url(DOCUMENT_URL, client=client, max_depth=0)
graph = KinematicGraph.from_cad(cad, root_mate_name=ROOT_MATE_NAME)
robot = Robot.from_graph(kinematic_graph=graph, client=client, name="robotarm", fetch_mass_properties=True)

URDFSerializer().save(robot, URDF_PATH, download_assets=True, mesh_dir=MESH_DIR)

convert_urdf_to_xacro(
    URDF_PATH,
    XACRO_PATH,
    package_name=PACKAGE_NAME,
    mesh_dir=MESH_DIR,
    strip_root_link=STRIP_ROOT_LINK,
    tip_link=TIP_LINK,
    tcp_offset_z=TCP_OFFSET_Z,
)
