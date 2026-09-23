#!/usr/bin/env python3
"""Visualize the robot's reachable workspace envelope (side + top view).

Resolves the robot's xacro to a URDF, parses the joint chain, sweeps the
reachable joint space, and computes forward kinematics for a chosen
reference link (default: tcp). Optionally renders the robot's STL meshes
for scale/context.

Two styles (--style): "ours" fills in every point the reference link
reaches over all joints that affect its position. "kuka" mimics a
manufacturer manual's diagram: the side view only sweeps the joints that
pitch in the arm's own vertical plane, the top view is just the outline
swept by the base yaw joint at the resulting max reach.
"""

import argparse
import sys
from dataclasses import dataclass, field
from xml.etree import ElementTree as ET

import numpy as np


@dataclass
class Joint:
    name: str
    joint_type: str
    parent: str
    child: str
    xyz: np.ndarray = field(default_factory=lambda: np.zeros(3))
    rpy: np.ndarray = field(default_factory=lambda: np.zeros(3))
    axis: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0]))
    lower: float = 0.0
    upper: float = 0.0


def default_urdf_path() -> str:
    from ament_index_python.packages import get_package_share_directory
    return f"{get_package_share_directory('robotarm_description')}/urdf/robotarm.urdf.xacro"


def load_urdf_root(xacro_path: str) -> ET.Element:
    import xacro
    doc = xacro.process_file(xacro_path)
    return ET.fromstring(doc.toxml())


def _vec3(s, default=(0.0, 0.0, 0.0)):
    return np.array([float(v) for v in s.split()]) if s else np.array(default)


def parse_joints_and_meshes(root):
    joints = {}
    for j in root.findall('joint'):
        origin = j.find('origin')
        axis_el = j.find('axis')
        limit = j.find('limit')
        joints[j.get('name')] = Joint(
            name=j.get('name'),
            joint_type=j.get('type'),
            parent=j.find('parent').get('link'),
            child=j.find('child').get('link'),
            xyz=_vec3(origin.get('xyz') if origin is not None else None),
            rpy=_vec3(origin.get('rpy') if origin is not None else None),
            axis=_vec3(axis_el.get('xyz') if axis_el is not None else None, (1.0, 0.0, 0.0)),
            lower=float(limit.get('lower')) if limit is not None and 'lower' in limit.attrib else 0.0,
            upper=float(limit.get('upper')) if limit is not None and 'upper' in limit.attrib else 0.0,
        )
    meshes = {}
    for link in root.findall('link'):
        mesh_el = link.find('./visual/geometry/mesh')
        meshes[link.get('name')] = mesh_el.get('filename') if mesh_el is not None else None
    return joints, meshes


def build_chain(joints):
    """Order joints base -> tip, following the (assumed strictly serial) chain."""
    children = {j.child for j in joints.values()}
    parents = {j.parent for j in joints.values()}
    root_link = next(link for link in parents if link not in children)
    by_parent = {j.parent: j for j in joints.values()}
    chain = []
    link = root_link
    while link in by_parent:
        j = by_parent[link]
        chain.append(j)
        link = j.child
    return chain


def joints_up_to_link(chain, reference_link):
    """Prefix of `chain` from the base up to and including the joint whose child
    is `reference_link`. Including that last joint is harmless: a joint's own
    rotation never moves its own child link's origin."""
    idx = next(i for i, j in enumerate(chain) if j.child == reference_link)
    return chain[:idx + 1]


def active_joints(prefix):
    """Revolute joints in `prefix` whose angle can move the final link's position
    (i.e. every revolute joint except the prefix's own last entry, which is
    inert for its own child's position)."""
    return [j for j in prefix[:-1] if j.joint_type == 'revolute']


def rpy_to_R(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return Rz @ Ry @ Rx


def origin_matrix(j):
    T = np.eye(4)
    T[:3, :3] = rpy_to_R(j.rpy)
    T[:3, 3] = j.xyz
    return T


def axis_rotation_batch(axis, q):
    """Rodrigues formula, vectorized: (N,4,4) rotation matrices about `axis`
    (unit 3-vector) for each angle in q (N,)."""
    axis = axis / np.linalg.norm(axis)
    n = q.shape[0]
    c, s, t = np.cos(q), np.sin(q), 1 - np.cos(q)
    x, y, z = axis
    R = np.empty((n, 3, 3))
    R[:, 0, 0] = t * x * x + c
    R[:, 0, 1] = t * x * y - s * z
    R[:, 0, 2] = t * x * z + s * y
    R[:, 1, 0] = t * x * y + s * z
    R[:, 1, 1] = t * y * y + c
    R[:, 1, 2] = t * y * z - s * x
    R[:, 2, 0] = t * x * z - s * y
    R[:, 2, 1] = t * y * z + s * x
    R[:, 2, 2] = t * z * z + c
    T = np.zeros((n, 4, 4))
    T[:, 3, 3] = 1.0
    T[:, :3, :3] = R
    return T


def fk_positions_batch(chain, reference_link, varying_joints, angles):
    """Position of `reference_link` for N joint configurations. `varying_joints`
    is the list of joint names that differ per sample, taken from `angles`
    columns in that same order; every other joint up to `reference_link`
    (including its own parent joint, which is inert for its own position
    anyway) is held at zero. Returns (N,3) world positions."""
    idx = next(i for i, j in enumerate(chain) if j.child == reference_link)
    sub_chain = chain[:idx + 1]
    n = angles.shape[0]
    T = np.broadcast_to(np.eye(4), (n, 4, 4)).copy()
    col = {name: i for i, name in enumerate(varying_joints)}
    for j in sub_chain:
        O = origin_matrix(j)
        if j.joint_type == 'revolute' and j.name in col:
            R = axis_rotation_batch(j.axis, angles[:, col[j.name]])
        else:
            R = np.broadcast_to(np.eye(4), (n, 4, 4))
        T = T @ (O[None, :, :] @ R)
    return T[:, :3, 3]


def fk_pose_single(chain, angle_by_joint):
    """Full pose (4x4) of every link in `chain`, for one joint configuration
    (dict joint_name -> angle, radians; missing/fixed joints default to 0)."""
    poses = {}
    T = np.eye(4)
    for j in chain:
        O = origin_matrix(j)
        if j.joint_type == 'revolute':
            R = axis_rotation_batch(j.axis, np.array([angle_by_joint.get(j.name, 0.0)]))[0]
        else:
            R = np.eye(4)
        T = T @ O @ R
        poses[j.child] = T
    return poses


def axis_world_direction(chain, joint_name):
    """World-frame direction of a joint's own rotation axis, evaluated at the
    all-zero configuration (i.e. composing only the fixed origin rotations of
    every joint up to and including this one, not any joint's own rotation --
    at q=0 that rotation is the identity anyway, so this is exactly the axis
    direction a full FK at the zero pose would give)."""
    R = np.eye(3)
    for j in chain:
        R = R @ rpy_to_R(j.rpy)
        if j.name == joint_name:
            return R @ j.axis
    raise ValueError(f'unknown joint: {joint_name}')


def classify_joints_by_axis(chain, world_direction, tol=0.01):
    """Revolute joints (in chain order) whose world-frame axis (at the zero
    pose) is parallel to `world_direction` (either sign)."""
    d = np.asarray(world_direction) / np.linalg.norm(world_direction)
    out = []
    for j in chain:
        if j.joint_type != 'revolute':
            continue
        a = axis_world_direction(chain, j.name)
        a = a / np.linalg.norm(a)
        if abs(abs(np.dot(a, d)) - 1.0) < tol:
            out.append(j)
    return out


def sample_angles(joints, n, seed, mode):
    rng = np.random.default_rng(seed)
    if mode == 'random':
        return np.stack([rng.uniform(j.lower, j.upper, n) for j in joints], axis=1)
    steps = max(2, round(n ** (1.0 / max(1, len(joints)))))
    axes = [np.linspace(j.lower, j.upper, steps) for j in joints]
    mesh = np.meshgrid(*axes, indexing='ij')
    return np.stack([m.ravel() for m in mesh], axis=1)


def resolve_package_uri(uri):
    from ament_index_python.packages import get_package_share_directory
    assert uri.startswith('package://'), f'unsupported mesh URI: {uri}'
    pkg, rel = uri[len('package://'):].split('/', 1)
    return f'{get_package_share_directory(pkg)}/{rel}'


def load_mesh_world(mesh_uri, T_link):
    """Returns (vertices_world (M,3), faces (F,3) triangle vertex indices)."""
    import trimesh
    path = resolve_package_uri(mesh_uri)
    mesh = trimesh.load(path, force='mesh', process=False)
    v = np.asarray(mesh.vertices)
    verts_world = (T_link[:3, :3] @ v.T).T + T_link[:3, 3]
    return verts_world, np.asarray(mesh.faces)


def plot_filled(ax, points2d, bins, color, alpha):
    from matplotlib.colors import ListedColormap
    H, xedges, yedges = np.histogram2d(points2d[:, 0], points2d[:, 1], bins=bins)
    mask = np.where(H.T > 0, 1.0, np.nan)
    ax.pcolormesh(xedges, yedges, mask, cmap=ListedColormap([color]), alpha=alpha, shading='flat')


def joint_grid_triangles(grid_points2d):
    """Triangulate every 2D face of a regular joint-space grid, in image space.

    `grid_points2d` has shape (s1, ..., sk, 2): the projected FK position for
    each node of a k-dimensional joint-angle grid (k >= 2). For each pair of
    joint axes, every grid face spanned by those two axes (the other joints
    fixed at a grid value) is a quad in joint space, and is split into two
    triangles; returns their images, (T,3,2).

    FK is continuous, so each small joint-space triangle maps to (roughly) the
    small image triangle spanned by its corners, and the union of all of them
    covers the reachable region without gaps -- unlike scattered point
    samples, whose coverage depends on how much FK stretches joint space
    locally. Faces along every axis pair (not just the grid's outer boundary)
    are needed because a boundary of the reachable region can also come from
    a singular configuration inside the joint box (e.g. a fully stretched
    elbow), not only from joint limits."""
    k = grid_points2d.ndim - 1
    tris = []
    for a in range(k):
        for b in range(a + 1, k):
            P = np.moveaxis(grid_points2d, (a, b), (0, 1))
            v00, v10 = P[:-1, :-1], P[1:, :-1]
            v01, v11 = P[:-1, 1:], P[1:, 1:]
            tris.append(np.stack([v00, v10, v11], axis=-2).reshape(-1, 3, 2))
            tris.append(np.stack([v00, v11, v01], axis=-2).reshape(-1, 3, 2))
    return np.concatenate(tris)


def rasterize_triangles(tris, resolution, pad_px=3):
    """Occupancy mask of the union of `tris` (T,3,2) on a square-pixel grid
    whose longer side has `resolution` pixels, with `pad_px` empty pixels of
    margin all around (so a contour of the mask always closes). Returns
    (mask (H,W) bool, row 0 = lowest y; x pixel centers (W,); y pixel centers
    (H,))."""
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.collections import PolyCollection
    from matplotlib.figure import Figure

    lo = tris.reshape(-1, 2).min(axis=0)
    hi = tris.reshape(-1, 2).max(axis=0)
    px = float(np.max(hi - lo)) / resolution
    lo = lo - pad_px * px
    hi = hi + pad_px * px
    w, h = (int(np.ceil(v)) for v in (hi - lo) / px)
    hi = lo + px * np.array([w, h])

    dpi = 100
    fig = Figure(figsize=(w / dpi, h / dpi), dpi=dpi)
    canvas = FigureCanvasAgg(fig)
    ax = fig.add_axes((0, 0, 1, 1))
    ax.set_axis_off()
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    # A thin edge on each triangle so sliver triangles (near singularities,
    # where FK folds the grid flat) still mark the pixels they cross.
    ax.add_collection(PolyCollection(tris, facecolor='k', edgecolor='k',
                                      linewidth=0.5 * 72 / dpi, antialiased=False))
    canvas.draw()
    img = np.asarray(canvas.buffer_rgba())[:, :, 0]
    mask = np.flipud(img < 128)
    xc = lo[0] + px * (np.arange(mask.shape[1]) + 0.5)
    yc = lo[1] + px * (np.arange(mask.shape[0]) + 0.5)
    return mask, xc, yc


def plot_outline_mask(ax, outline_mask, color, linewidth):
    """Boundary of a precomputed occupancy mask (see `rasterize_triangles`),
    including interior holes (e.g. an unreachable gap near the base)."""
    mask, xc, yc = outline_mask
    ax.contour(xc, yc, mask.astype(float), levels=[0.5], colors=color, linewidths=linewidth)


def plot_outline_line(ax, outline_xy, color, linewidth):
    ax.plot(outline_xy[:, 0], outline_xy[:, 1], color=color, linewidth=linewidth)


def plot_mesh_silhouette(ax, verts2d, faces, color):
    """Fills each mesh triangle's projected footprint, rather than plotting sparse
    vertex points -- CAD meshes tessellate flat panels with very few large
    triangles and curved/detailed regions with many small ones, so a
    vertex-density-based occupancy grid leaves large flat panels looking empty."""
    from matplotlib.collections import PolyCollection
    tris = verts2d[faces]  # (F,3,2)
    ax.add_collection(PolyCollection(tris, facecolor=color, edgecolor='none',
                                      linewidth=0, antialiased=False))


def axis1_sweep_outline(max_r, yaw_joint, n=400):
    """Boundary of the disk sector swept by rotating a point at radius `max_r`
    through `yaw_joint`'s full angular range -- an outline (not filled), like
    the manufacturer-manual top view: the outer arc plus the two straight
    radial edges where the joint's limit cuts the circle off."""
    theta = np.linspace(yaw_joint.lower, yaw_joint.upper, n)
    arc_x = max_r * np.cos(theta)
    arc_y = max_r * np.sin(theta)
    x = np.concatenate([[0.0], arc_x, [0.0]])
    y = np.concatenate([[0.0], arc_y, [0.0]])
    return np.column_stack([x, y])


def render_panel(kind, ax_side, ax_top, data, mesh_data_by_link, bins):
    if kind == 'ours':
        plot_filled(ax_side, data['side_xyz'][:, [0, 2]], bins, 'C0', 0.45)
        plot_filled(ax_top, data['top_xyz'][:, :2], bins, 'C0', 0.45)
    else:
        plot_outline_mask(ax_side, data['side_mask'], 'C0', 1.5)
        plot_outline_line(ax_top, data['top_outline'], 'C0', 1.5)

    for verts, faces in mesh_data_by_link.values():
        plot_mesh_silhouette(ax_side, verts[:, [0, 2]], faces, '0.3')
        plot_mesh_silhouette(ax_top, verts[:, :2], faces, '0.3')

    for ax, ylabel, title in ((ax_side, 'z [m]', 'Side view (x-z)'), (ax_top, 'y [m]', 'Top view (x-y)')):
        ax.set_xlabel('x [m]')
        ax.set_ylabel(ylabel)
        # adjustable='datalim' (not the 'equal' default of 'box'): keep every
        # subplot's box the same size as its grid cell and pad the data
        # limits instead, so side/top panels render at the same height even
        # though their data spans different x:y ratios.
        ax.set_aspect('equal', adjustable='datalim')
        ax.set_title(title)


def make_figure(style, ours, kuka, mesh_data_by_link, bins, output, dpi, show):
    """`ours`: dict with 'side_xyz'/'top_xyz' point clouds, rendered filled.
    `kuka`: dict with 'side_mask' (occupancy mask from `rasterize_triangles`)
    and 'top_outline' (precomputed polyline), rendered as outlines. Either
    may be None if that style wasn't requested."""
    import matplotlib
    if not show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    rows = [r for r in (('ours', ours), ('kuka', kuka)) if style in (r[0], 'both')]
    fig, axes = plt.subplots(len(rows), 2, figsize=(14, 7 * len(rows)), squeeze=False)

    for (kind, data), (ax_side, ax_top) in zip(rows, axes):
        render_panel(kind, ax_side, ax_top, data, mesh_data_by_link, bins)
        suffix = 'all axes' if kind == 'ours' else 'axis2/3/5 side, axis1 sweep, outlines (KUKA style)'
        ax_side.set_title(f'{ax_side.get_title()} -- {suffix}')
        ax_top.set_title(f'{ax_top.get_title()} -- {suffix}')

    fig.tight_layout()
    fig.savefig(output, dpi=dpi)
    if show:
        plt.show()


def build_argparser():
    p = argparse.ArgumentParser(description='Reachable-workspace envelope: side/top silhouettes via FK sweep.')
    p.add_argument('--urdf', default=None,
                    help='path to robotarm.urdf.xacro (default: resolved via ament_index_python)')
    p.add_argument('--reference-link', default='tcp',
                    help='link whose origin is swept (default: tcp). Use "Stage5_1" for the '
                         'wrist center (intersection of axes 4/5) instead.')
    p.add_argument('--style', choices=['ours', 'kuka', 'both'], default='ours',
                    help='"ours": fill in every reachable point of the reference link over all '
                         'relevant joints. "kuka": manufacturer-manual style -- side view swept '
                         'over only the axes that pitch in the arm\'s own vertical plane (here '
                         'axis2/3/5), top view is just the outline swept by the base yaw joint '
                         '(axis1) at the resulting max reach. "both": render both, stacked.')
    p.add_argument('--samples', type=int, default=200_000)
    p.add_argument('--sampling', choices=['random', 'grid'], default='random')
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--bins', type=int, default=300, help='occupancy-grid resolution per subplot')
    p.add_argument('--outline-resolution', type=int, default=1200,
                    help='kuka side view: pixels along the longer side of the raster the outline '
                         'is traced from')
    p.add_argument('--no-mesh',action='store_true', help='skip mesh loading (no trimesh needed)')
    p.add_argument('--output', default='workspace_envelope.png')
    p.add_argument('--dpi', type=int, default=150)
    p.add_argument('--show', action='store_true', help='also open an interactive window')
    p.add_argument('-v', '--verbose', action='store_true')
    return p


def main(argv=None):
    args = build_argparser().parse_args(argv)
    urdf_path = args.urdf or default_urdf_path()

    if args.verbose:
        print(f'Loading URDF from {urdf_path}', file=sys.stderr)
    root = load_urdf_root(urdf_path)
    joints, meshes = parse_joints_and_meshes(root)
    chain = build_chain(joints)

    prefix = joints_up_to_link(chain, args.reference_link)
    prefix_names = {j.name for j in prefix}
    if args.verbose:
        print(f'Chain: {[j.name for j in chain]}', file=sys.stderr)
        print(f'Reference link: {args.reference_link}', file=sys.stderr)

    if args.verbose and prefix[-1].joint_type == 'revolute':
        # Sanity check: the reference link's own parent joint must be inert for its position.
        j = prefix[-1]
        rng = np.random.default_rng(0)
        base_pos = fk_pose_single(chain, {})[args.reference_link][:3, 3]
        drift = [np.linalg.norm(fk_pose_single(chain, {j.name: q})[args.reference_link][:3, 3] - base_pos)
                 for q in rng.uniform(j.lower, j.upper, 3)]
        ok = all(d < 1e-9 for d in drift)
        print(f'Own-joint invariance check for {j.name}: max drift={max(drift):.2e} '
              f'({"OK" if ok else "FAILED"})', file=sys.stderr)

    ours = kuka = None

    if args.style in ('ours', 'both'):
        swept = active_joints(prefix)
        if args.verbose:
            print(f'[ours] swept joints: {[j.name for j in swept]} ({len(swept)})', file=sys.stderr)
        angles = sample_angles(swept, args.samples, args.seed, args.sampling)
        xyz = fk_positions_batch(chain, args.reference_link, [j.name for j in swept], angles)
        ours = {'side_xyz': xyz, 'top_xyz': xyz}

    if args.style in ('kuka', 'both'):
        pitch_joints = [j for j in classify_joints_by_axis(chain, (0.0, 1.0, 0.0)) if j.name in prefix_names]
        yaw_joints = [j for j in classify_joints_by_axis(chain, (0.0, 0.0, 1.0)) if j.name in prefix_names]
        if not yaw_joints:
            sys.exit(f'--style kuka needs a base yaw joint (axis parallel to world Z) upstream of '
                      f'{args.reference_link!r}, found none')
        yaw_joint = yaw_joints[0]
        if args.verbose:
            print(f'[kuka] side-plane (pitch) joints: {[j.name for j in pitch_joints]}', file=sys.stderr)
            print(f'[kuka] yaw joint: {yaw_joint.name}', file=sys.stderr)
        if len(pitch_joints) < 2:
            sys.exit(f'--style kuka needs at least two side-plane (pitch) joints upstream of '
                      f'{args.reference_link!r} to sweep an area, found {len(pitch_joints)}')
        # Always a regular grid here (not --sampling): the outline is built
        # from the grid's faces, see joint_grid_triangles.
        steps = max(2, round(args.samples ** (1.0 / len(pitch_joints))))
        side_angles = sample_angles(pitch_joints, steps ** len(pitch_joints), args.seed, 'grid')
        side_xyz = fk_positions_batch(chain, args.reference_link, [j.name for j in pitch_joints], side_angles)
        grid_xz = side_xyz[:, [0, 2]].reshape((steps,) * len(pitch_joints) + (2,))
        side_mask = rasterize_triangles(joint_grid_triangles(grid_xz), args.outline_resolution)
        if args.verbose:
            print(f'[kuka] side grid: {steps} steps/joint, mask {side_mask[0].shape}', file=sys.stderr)
        max_r = float(np.max(np.hypot(side_xyz[:, 0], side_xyz[:, 1])))
        kuka = {'side_mask': side_mask, 'top_outline': axis1_sweep_outline(max_r, yaw_joint)}

    mesh_data_by_link = {}
    if not args.no_mesh:
        home_poses = fk_pose_single(chain, {})
        for link, uri in meshes.items():
            if uri is None:
                continue
            T_link = home_poses.get(link, np.eye(4))
            if args.verbose:
                print(f'Loading mesh for {link}: {uri}', file=sys.stderr)
            mesh_data_by_link[link] = load_mesh_world(uri, T_link)

    make_figure(args.style, ours, kuka, mesh_data_by_link, args.bins, args.output, args.dpi, args.show)
    if args.verbose:
        print(f'Wrote {args.output}', file=sys.stderr)


if __name__ == '__main__':
    main()
