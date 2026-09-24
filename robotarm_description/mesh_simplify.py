"""Post-process the meshes of an already exported URDF/xacro. Fully offline: no Onshape client, no API calls."""

from onshape_robotics_toolkit.formats import process_urdf_meshes
from onshape_robotics_toolkit.mesh import MeshOptions
from onshape_robotics_toolkit.utilities import setup_default_logging

URDF_PATH = "urdf/robotarm_geometry.xacro"  # a .xacro with package:// mesh paths works too
MESH_DIR = "meshes"

if __name__ == "__main__":
    setup_default_logging(file_path="postprocess.log", console_level="INFO")
    process_urdf_meshes(
        URDF_PATH,
        mesh_dir=MESH_DIR,
        mesh_options=MeshOptions(visual_max_faces=100_000, collision_mode="convex_decomposition"),
    )
