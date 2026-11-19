"""Build game-ready GLB files of the Eddie robot for Cyberpunk 2077 modding.

Assembles the meshes with the same kinematics as urdf/eddie_robot.urdf.xacro:
the base (eddie_base.macro.xacro + kelo_drive_unit / kelo_hub_wheel /
sensors/hokuyo, wheel joints at zero), two Kinova Gen3 7-DoF arms with vision
module in the pose from config/initial_positions_arm_*.yaml, and Robotiq 2F-85
grippers. The robot, arm and gripper descriptions come from the packages that
fetch_deps.py clones into ../deps (see README.md). It decimates the meshes, adds
smooth-shaded normals and box-projected UVs, and writes:

  out/eddie_parts.glb      one node per part, wheels and drive units as their own
                           nodes pivoting on their joint axes (for Blender: rigging,
                           animation, or re-exporting a subset)
  out/eddie_static.glb     everything baked into one object per material, named
                           submesh_XX_LOD_1 as WolvenKit's mesh import expects
  out/eddie_drive.glb,     with MOVING_PARTS: one drive housing and the two wheels
  eddie_wheel_r/l.glb      (hub + tyre) in their own joint frames, for the game mod
                           to steer and spin; eddie_static.glb then leaves them out
  out/eddie_arm_*.glb,     with MOVING_ARMS: each arm link and gripper part in its own
  eddie_grip_*.glb         frame (shared by both arms), plus the joint chain and IK-solved
                           poses in eddie_physics.json -> moving_parts.arms
  out/eddie_collision.glb  convex hulls for the physics shape (<= 255 verts each)
  out/eddie_physics.json   mass, centre of mass, inertia, hull and material table

Output orientation: glTF Y-up, metres, origin on the floor under the robot centre,
robot front facing +Y once imported into Blender / WolvenKit (Z-up, the game's
forward axis).

Run:  .venv/Scripts/python build_glb.py   (from this folder)
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import fast_simplification
import numpy as np
import pyfqmr
import trimesh
from scipy.spatial.transform import Rotation
from trimesh.visual.material import PBRMaterial

HERE = Path(__file__).resolve().parent
DEPS = HERE.parent / "deps"  # python fetch_deps.py clones the packages below here
MESHES = DEPS / "eddie_description/meshes"
OUT = HERE / "out"
KORTEX_MESHES = DEPS / "ros2_kortex/kortex_description/arms/gen3/7dof/meshes"
ROBOTIQ_MESHES = DEPS / "ros2_robotiq_gripper/robotiq_description/meshes/visual/2f_85"

INCLUDE_ARMS = True
INCLUDE_GRIPPERS = True  # eddie_robot.urdf.xacro default: robotiq_2f_85
# joint_1..joint_7, radians; defaults from config/initial_positions_arm_*.yaml
ARM_POSE = {
    "left": (-0.70, -2.05, 2.14, -2.44, -1.57, 0.0, 0.0),
    "right": (0.70, -2.05, 0.90, 2.44, 1.57, 0.0, 0.0),
}
GRIPPER_CLOSURE = 0.0  # robotiq_85_left_knuckle_joint, 0 = open .. 0.8 = closed
# drive housings and wheels as their own meshes (out/eddie_drive.glb, eddie_wheel_*.glb)
# instead of baked into eddie_static.glb, so the game mod can steer and spin them
MOVING_PARTS = True
# the same for the arm links and gripper fingers (out/eddie_arm_*.glb, eddie_grip_*.glb),
# so the mod can pose the arms; needs MOVING_PARTS
MOVING_ARMS = True

SMOOTH_ANGLE_DEG = 35.0  # edges sharper than this stay hard
UV_METRES_PER_TILE = 0.5  # box-projection density for tiling game materials
MIN_PART_MM = 10.0  # loose parts with a smaller bounding-box diagonal are dropped
MAX_SUBMESH_VERTS = 65000  # 16-bit index buffers in the game's meshes
DECIMATE_AGGRESSIVENESS = 5  # pyfqmr: lower keeps shape better, may miss the budget
# arm/gripper .dae pieces: keep this fraction of triangles, clamped to [MIN, CAP]
DAE_TRI_RATIO, DAE_TRI_MIN, DAE_TRI_CAP = 0.3, 150, 3000

# ---------------------------------------------------------------------------
# Materials (colours from meshes/colors.json, meshes/torso_colors.json)
# ---------------------------------------------------------------------------
MATERIALS = {
    # name: (rgb, metallic, roughness)
    "casing_grey": ((0.25, 0.25, 0.26), 0.2, 0.55),
    "cover_light": ((0.898, 0.918, 0.929), 0.0, 0.35),
    "lidar_grey": ((0.792, 0.82, 0.933), 0.1, 0.3),
    "mount_grey": ((0.42, 0.42, 0.43), 0.6, 0.4),
    "drive_grey": ((0.35, 0.35, 0.35), 0.5, 0.45),
    "wheel_light": ((0.996, 0.996, 1.0), 0.0, 0.4),
    "tyre_dark": ((0.251, 0.251, 0.251), 0.0, 0.85),
    "sensor_black": ((0.06, 0.06, 0.07), 0.1, 0.5),
    # arms and grippers (URDF/.dae colours)
    "arm_grey": ((0.753, 0.753, 0.753), 0.3, 0.4),
    "arm_accent": ((0.0, 0.0, 1.0), 0.0, 0.4),
    "gripper_grey": ((0.8, 0.8, 0.8), 0.6, 0.35),
    "gripper_black": ((0.03, 0.03, 0.03), 0.0, 0.6),
}

# .dae embedded colour (0-255) -> material
KORTEX_PALETTE = [((255, 255, 255), "arm_grey"), ((0, 0, 255), "arm_accent")]
ROBOTIQ_PALETTE = [((204, 204, 204), "gripper_grey"), ((76, 76, 76), "gripper_grey"),
                   ((8, 8, 8), "gripper_black")]

# triangle budget per source mesh (LOD 1 = full detail in game terms)
TRI_BUDGET = {
    "casing.stl": 9000,
    "panel.stl": 2000,
    "cover.stl": 2500,
    "lidar.stl": 5000,
    "torso.stl": 30000,
    "torso_mounts.stl": 2000,
    "drive.stl": 5000,
    "wheel_a.stl": 1600,
    "wheel_b.stl": 1600,
    "tyre_a.stl": 1100,
    "tyre_b.stl": 1100,
    "hokuyo": 1200,
}


# ---------------------------------------------------------------------------
# Transforms
# ---------------------------------------------------------------------------
def tf(xyz=(0, 0, 0), rpy=(0, 0, 0)) -> np.ndarray:
    """URDF origin -> 4x4 (R = Rz(yaw) Ry(pitch) Rx(roll))."""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = map(float, (np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)))
    m = np.eye(4)
    m[:3, :3] = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    m[:3, 3] = xyz
    return m


def scale(s: float) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] *= s
    return m


HALF_PI = math.pi / 2
MESH_RPY = (HALF_PI, 0, HALF_PI)  # CAD is y-up
MESH_RPY_MIRRORED = (HALF_PI, 0, -HALF_PI)  # same part half a turn about the pivot
MM = scale(0.001)

# URDF (Z-up, +X forward) -> output: robot front to +Y, then Z-up -> glTF Y-up
URDF_TO_GLTF = np.array(
    [[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1]], dtype=float
) @ tf(rpy=(0, 0, HALF_PI))


# ---------------------------------------------------------------------------
# Kinematic tree (all joints at zero)
# ---------------------------------------------------------------------------
@dataclass
class Visual:
    name: str
    source: str  # STL file name, "hokuyo", or a .dae piece key (see dae_visuals)
    material: str
    origin: np.ndarray  # mesh -> link


def inertia(ixx, iyy, izz, ixy=0.0, ixz=0.0, iyz=0.0) -> np.ndarray:
    return np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]], dtype=float)


@dataclass
class Link:
    name: str
    parent: str | None
    origin: np.ndarray  # link -> parent, joint position included
    visuals: list[Visual] = field(default_factory=list)
    mass: float = 0.0
    com: tuple = (0, 0, 0)
    inertia: np.ndarray = field(default_factory=lambda: np.zeros((3, 3)))  # at com
    collision_group: str | None = None  # links sharing a group share one hull
    # moving joint: origin = joint_origin @ rotation about `axis` by scale * value of `var`
    joint_origin: np.ndarray | None = None
    joint: tuple | None = None  # (var, axis, scale)


def rot_z(q: float) -> np.ndarray:
    return tf(rpy=(0, 0, q))


def rot_y(q: float) -> np.ndarray:
    return tf(rpy=(0, q, 0))


_dae_pieces: dict[str, trimesh.Trimesh] = {}


def dae_visuals(name: str, path: Path, palette, origin=None) -> list[Visual]:
    """One Visual per geometry in a .dae, material picked from its embedded colour."""
    if not path.exists():
        raise FileNotFoundError(f"{path} missing: run fetch_deps.py first")
    scene = trimesh.load(path, force="scene")
    out = []
    for i, node in enumerate(sorted(scene.graph.nodes_geometry)):
        transform, geom = scene.graph[node]
        mesh = scene.geometry[geom]
        key = f"{path.parent.name}/{path.name}#{i}"
        if key not in _dae_pieces:
            _dae_pieces[key] = trimesh.Trimesh(
                trimesh.transform_points(mesh.vertices, transform), mesh.faces, process=False)
        rgb = np.asarray(mesh.visual.material.main_color[:3], dtype=float)
        mat = min(palette, key=lambda p: np.sum((np.array(p[0]) - rgb) ** 2))[1]
        out.append(Visual(f"{name}_geo{i}", key, mat, np.eye(4) if origin is None else origin))
    return out


# Kinova Gen3 7-DoF (kortex_description/arms/gen3/7dof/urdf/gen3_macro.xacro, vision=true):
# (link, mesh, joint origin xyz, joint origin rpy, mass, com, inertia ixx iyy izz ixy ixz iyz)
GEN3_CHAIN = (
    ("shoulder_link", "shoulder_link.dae", (0, 0, 0.15643), (3.1416, 0, 0), 1.3773,
     (-2.3e-05, -0.010364, -0.07336), (0.00457, 0.004831, 0.001409, 1e-06, 2e-06, 0.000448)),
    ("half_arm_1_link", "half_arm_1_link.dae", (0, 0.005375, -0.12838), (1.5708, 0, 0), 1.1636,
     (-4.4e-05, -0.09958, -0.013278), (0.011088, 0.001072, 0.011255, 5e-06, 0, -0.000691)),
    ("half_arm_2_link", "half_arm_2_link.dae", (0, -0.21038, -0.006375), (-1.5708, 0, 0), 1.1636,
     (-4.4e-05, -0.006641, -0.117892), (0.010932, 0.011127, 0.001043, 0, -7e-06, 0.000606)),
    ("forearm_link", "forearm_link.dae", (0, 0.006375, -0.21038), (1.5708, 0, 0), 0.9302,
     (-1.8e-05, -0.075478, -0.015006), (0.008147, 0.000631, 0.008316, -1e-06, 0, -0.0005)),
    ("spherical_wrist_1_link", "spherical_wrist_1_link.dae", (0, -0.20843, -0.006375), (-1.5708, 0, 0),
     0.6781, (1e-06, -0.009432, -0.063883), (0.001596, 0.001607, 0.000399, 0, 0, 0.000256)),
    ("spherical_wrist_2_link", "spherical_wrist_2_link.dae", (0, 0.00017505, -0.10593), (1.5708, 0, 0),
     0.6781, (1e-06, -0.045483, -0.00965), (0.001641, 0.00041, 0.001641, 0, 0, -0.000278)),
    ("bracelet_link", "bracelet_with_vision_link.dae", (0, -0.10593, -0.00017505), (-1.5708, 0, 0),
     0.5, (0.000281, 0.011402, -0.029798), (0.000587, 0.000369, 0.000609, 3e-06, 3e-06, -0.000118)),
)


def build_arm(side: str) -> list[Link]:
    """kinova_gen3.macro.xacro on {side}_arm_attachment_link, joints from ARM_POSE."""
    p = f"{side}_arm_"
    links = []
    base = Link(f"{p}base_link", f"{side}_arm_attachment_link",
                tf((0.01549, 0, 0), (1.5708, -1.5708, 1.5708)), mass=1.697,
                com=(-0.000648, -0.000166, 0.084487),
                inertia=inertia(0.004622, 0.004495, 0.002079, 9e-06, 6e-05, 9e-06))
    base.visuals = dae_visuals(f"{p}base", KORTEX_MESHES / "base_link.dae", KORTEX_PALETTE)
    base.collision_group = base.name
    links.append(base)
    parent = base.name
    for i, (q, (name, mesh, xyz, rpy, mass, com, I)) in enumerate(zip(ARM_POSE[side], GEN3_CHAIN), 1):
        link = Link(f"{p}{name}", parent, tf(xyz, rpy) @ rot_z(q), mass=mass, com=com,
                    inertia=inertia(*I), collision_group=f"{p}{name}",
                    joint_origin=tf(xyz, rpy), joint=(f"{side[0]}{i}", (0.0, 0.0, 1.0), 1.0))
        link.visuals = dae_visuals(f"{p}{name.removesuffix('_link')}", KORTEX_MESHES / mesh,
                                   KORTEX_PALETTE)
        links.append(link)
        parent = link.name
    links.append(Link(f"{p}end_effector_link", parent, tf((0, 0, -0.061525), (math.pi, 0, 0))))
    if INCLUDE_GRIPPERS:
        links += build_gripper(p, f"{p}end_effector_link")
    return links


def build_gripper(p: str, parent: str) -> list[Link]:
    """robotiq_2f_85_macro.urdf.xacro; mimic joints follow GRIPPER_CLOSURE.

    Finger inertias are left out (point masses): ~0.26 kg in total, negligible here.
    """
    c = GRIPPER_CLOSURE
    g = f"{p}gripper"
    r = f"{p}robotiq_85_"
    base = Link(f"{r}base_link", parent, np.eye(4), mass=0.663, com=(0, 2.274e-05, 0.03232288),
                collision_group=g)
    base.visuals = dae_visuals(f"{r}base", ROBOTIQ_MESHES / "robotiq_base.dae", ROBOTIQ_PALETTE)
    links = [base]
    var = f"{p[0]}g"  # the closure, shared by all mimic joints
    # (link, parent, xyz, mimic multiplier (angle = m * closure about (0,-1,0)), mass, com)
    for name, par, xyz, m, mass, com in (
        ("left_knuckle", "base", (0.03060114, 0, 0.05490452), 1, 0.01384773, (0.01213197, 0.0002, -0.00058647)),
        ("right_knuckle", "base", (-0.03060114, 0, 0.05490452), -1, 0.01384773, (-0.01213197, 0.0002, -0.00058647)),
        ("left_finger", "left_knuckle", (0.03152616, 0, -0.00376347), 0, 0.04260377, (0.00346899, -0.00079447, 0.01867121)),
        ("right_finger", "right_knuckle", (-0.03152616, 0, -0.00376347), 0, 0.04260377, (-0.00346899, -5.53e-06, 0.01867121)),
        ("left_inner_knuckle", "base", (0.0127, 0, 0.06142), 1, 0.02969376, (0.01897699, 0.00015001, 0.02247101)),
        ("right_inner_knuckle", "base", (-0.0127, 0, 0.06142), -1, 0.02969376, (-0.01926824, 5.001e-05, 0.02222178)),
        ("left_finger_tip", "left_finger", (0.00563134, 0, 0.04718515), -1, 0.04268589, (-0.01456706, -0.0008, 0.01649701)),
        ("right_finger_tip", "right_finger", (-0.00563134, 0, 0.04718515), 1, 0.04268589, (0.01456706, 5e-05, 0.01649701)),
    ):
        link = Link(f"{r}{name}_link", f"{r}{par}_link", tf(xyz) @ rot_y(-m * c), mass=mass, com=com,
                    collision_group=g)
        if m:
            link.joint_origin, link.joint = tf(xyz), (var, (0.0, 1.0, 0.0), -float(m))
        link.visuals = dae_visuals(f"{r}{name}", ROBOTIQ_MESHES / f"{name}.dae", ROBOTIQ_PALETTE)
        links.append(link)
    return links


def build_tree() -> list[Link]:
    links = [Link("base_footprint", None, np.eye(4))]

    body = Link("base_link", "base_footprint", tf((0, 0, 0.2164)), mass=76.4377,
                com=(0, 0, 0.0833), inertia=inertia(4.128795216, 4.117596854, 3.364397364))
    base = tf(rpy=MESH_RPY) @ MM
    torso = tf((-0.1738, 0.0003, 0.4184), MESH_RPY) @ MM
    body.visuals += [
        Visual("casing", "casing.stl", "casing_grey", base),
        Visual("panel", "panel.stl", "casing_grey", base),
        Visual("cover", "cover.stl", "cover_light", base),
        Visual("lidar", "lidar.stl", "lidar_grey", base),
        Visual("torso", "torso.stl", "cover_light", torso),
        Visual("torso_mounts", "torso_mounts.stl", "mount_grey", torso),
    ]
    links.append(body)

    # laser scanners (mounted upside down)
    for name, xyz, yaw in (("front_right_laser", (0.2193, -0.2391, 0.0682), -0.7854),
                           ("rear_left_laser", (-0.2193, 0.2386, 0.0682), 2.3562)):
        laser = Link(name, "base_link", tf(xyz) @ tf(rpy=(math.pi, 0, yaw)), mass=0.16)
        laser.visuals.append(Visual(f"{name}_scanner", "hokuyo", "sensor_black", tf((0, 0, 0.015))))
        links.append(laser)

    drives = (
        ("front_left", (0.18813, 0.20748, -0.1273), True),
        ("rear_left", (-0.18787, 0.20748, -0.1273), True),
        ("rear_right", (-0.18787, -0.20752, -0.1273), False),
        ("front_right", (0.18813, -0.20752, -0.1273), False),
    )
    for name, xyz, mirrored in drives:
        flip = -1 if mirrored else 1
        mesh_tf = tf(rpy=MESH_RPY_MIRRORED if mirrored else MESH_RPY) @ MM
        unit = Link(f"{name}_drive", "base_link", tf(xyz), mass=1.392, com=(0, flip * 0.0101, 0),
                    inertia=inertia(0.004574163, 0.004344228, 0.004640438))
        unit.visuals.append(Visual(f"{name}_drive_housing", "drive.stl", "drive_grey", mesh_tf))
        links.append(unit)
        for side, wxyz, wheel, tyre in (
            ("r", (flip * -0.0405, flip * 0.010108, -0.036587), "wheel_a.stl", "tyre_a.stl"),
            ("l", (flip * 0.0405, flip * 0.009890, -0.036587), "wheel_b.stl", "tyre_b.stl"),
        ):
            w = Link(f"{name}_drive_{side}_wheel", f"{name}_drive", tf(wxyz), mass=1.563,
                     inertia=inertia(0.001579986, 0.001012837, 0.001013130))
            w.visuals += [
                Visual(f"{name}_{side}_hub", wheel, "wheel_light", mesh_tf),
                Visual(f"{name}_{side}_tyre", tyre, "tyre_dark", mesh_tf),
            ]
            links.append(w)

    if INCLUDE_ARMS:
        # arm attachment frames (eddie_base.macro.xacro)
        for side, dy, yaw in (("left", 0.0676, 1.5708 - 0.7783), ("right", -0.0676, 1.5708 - 2.3632)):
            links.append(Link(f"{side}_arm_attachment_link", "base_link",
                              tf((-0.1738 + 0.0236, 0.0003 + dy, 0.4184 + 0.1160), (0, 0.09, yaw))))
            links += build_arm(side)
    return links


def world_transforms(links: list[Link]) -> dict[str, np.ndarray]:
    by_name = {l.name: l for l in links}
    cache: dict[str, np.ndarray] = {}

    def get(n: str) -> np.ndarray:
        if n not in cache:
            l = by_name[n]
            cache[n] = l.origin if l.parent is None else get(l.parent) @ l.origin
        return cache[n]

    return {l.name: get(l.name) for l in links}


# ---------------------------------------------------------------------------
# Mesh processing
# ---------------------------------------------------------------------------
_source_cache: dict[str, trimesh.Trimesh] = {}


def load_source(source: str) -> trimesh.Trimesh:
    """Load and decimate a source mesh once (in its own units/frame)."""
    if source in _source_cache:
        return _source_cache[source]
    is_stl = source.endswith(".stl")
    if source in _dae_pieces:
        mesh = _dae_pieces[source].copy()
    elif source == "hokuyo":
        mesh = trimesh.load(MESHES / "sensors" / "hokuyo.dae", force="mesh")
    else:
        mesh = trimesh.load(MESHES / source, force="mesh")
    mesh.merge_vertices()
    mesh.update_faces(mesh.nondegenerate_faces())
    mesh.remove_unreferenced_vertices()
    if is_stl:
        # drop screws, washers and tessellation slivers: invisible in game, and the
        # hundreds of tiny open shells are what stalls the decimator
        bodies = mesh.split(only_watertight=False)
        big = [p for p in bodies if np.linalg.norm(p.extents) >= MIN_PART_MM]
        mesh = trimesh.util.concatenate(big)
        mesh.merge_vertices()
    if source in TRI_BUDGET:
        target = TRI_BUDGET[source]
    else:
        target = int(np.clip(len(mesh.faces) * DAE_TRI_RATIO, DAE_TRI_MIN, DAE_TRI_CAP))
    if len(mesh.faces) > target:
        simp = pyfqmr.Simplify()
        simp.setMesh(mesh.vertices.astype(np.float64), mesh.faces.astype(np.int32))
        simp.simplify_mesh(target_count=target, aggressiveness=DECIMATE_AGGRESSIVENESS,
                           preserve_border=True, verbose=False)
        v, f, _ = simp.getMesh()
        mesh = trimesh.Trimesh(v, f, process=True)
        mesh.update_faces(mesh.nondegenerate_faces())
        mesh.remove_unreferenced_vertices()
    _source_cache[source] = mesh
    return mesh


def shade_and_unwrap(vertices: np.ndarray, faces: np.ndarray, uv_frame: np.ndarray):
    """Split vertices at hard edges, compute smooth normals and box-projected UVs.

    uv_frame maps the vertices into the space the box projection is done in (the
    final output space) so the texture density is uniform in metres.
    Returns (vertices, faces, normals, uv) with vertices re-indexed.
    """
    tri = vertices[faces]  # (F,3,3)
    fn = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    area2 = np.linalg.norm(fn, axis=1)
    keep = area2 > 1e-18
    faces, tri, fn, area2 = faces[keep], tri[keep], fn[keep], area2[keep]
    fn_unit = fn / area2[:, None]

    # corner -> vertex grouping
    nf = len(faces)
    corner_v = faces.reshape(-1)
    corner_f = np.repeat(np.arange(nf), 3)
    order = np.argsort(corner_v, kind="stable")
    sv = corner_v[order]
    starts = np.flatnonzero(np.r_[True, sv[1:] != sv[:-1]])
    counts = np.diff(np.r_[starts, len(sv)])
    group_of = np.repeat(np.arange(len(starts)), counts)  # per sorted corner

    # all (corner, neighbour corner) pairs sharing a vertex
    reps = counts[group_of]
    a = np.repeat(np.arange(len(sv)), reps)
    offs = np.arange(len(a)) - np.repeat(np.cumsum(reps) - reps, reps)
    b = starts[group_of[a]] + offs
    fa, fb = corner_f[order[a]], corner_f[order[b]]
    cos_t = math.cos(math.radians(SMOOTH_ANGLE_DEG))
    ok = np.einsum("ij,ij->i", fn_unit[fa], fn_unit[fb]) >= cos_t
    corner_n = np.zeros((len(sv), 3))
    np.add.at(corner_n, a[ok], fn[fb[ok]])  # area-weighted (fn is 2*area*normal)
    corner_n /= np.linalg.norm(corner_n, axis=1, keepdims=True) + 1e-30
    normals = np.empty_like(corner_n)
    normals[order] = corner_n

    # box projection in output space
    pos_out = trimesh.transform_points(vertices, uv_frame)
    n_out = fn_unit @ uv_frame[:3, :3].T
    axis = np.argmax(np.abs(n_out), axis=1)
    sign = np.sign(n_out[np.arange(nf), axis])
    uv_axes = np.array([[2, 1], [0, 2], [0, 1]])  # project along x, y, z
    p = pos_out[faces].reshape(-1, 3)
    ax = np.repeat(axis, 3)
    sg = np.repeat(sign, 3)
    u = p[np.arange(len(p)), uv_axes[ax, 0]] * np.where(np.isin(ax, (0, 2)), -sg, sg)
    v = p[np.arange(len(p)), uv_axes[ax, 1]]
    uv = np.stack([u, v], axis=1) / UV_METRES_PER_TILE

    # weld identical corners
    key = np.concatenate([corner_v[:, None].astype(np.float64),
                          np.round(normals * 1e4), np.round(uv * 1e5)], axis=1)
    _, first, inverse = np.unique(key, axis=0, return_index=True, return_inverse=True)
    new_faces = inverse.reshape(-1, 3)
    return vertices[corner_v[first]], new_faces, normals[first], uv[first]


def make_mesh(vertices, faces, frame: np.ndarray, uv_frame: np.ndarray, material: PBRMaterial):
    v = trimesh.transform_points(vertices, frame)
    if np.linalg.det(frame[:3, :3]) < 0:
        faces = faces[:, ::-1]
    v, f, n, uv = shade_and_unwrap(v, faces, uv_frame)
    mesh = trimesh.Trimesh(v, f, vertex_normals=n, process=False)
    mesh.visual = trimesh.visual.TextureVisuals(uv=uv, material=material)
    return mesh


def pbr(name: str) -> PBRMaterial:
    rgb, metal, rough = MATERIALS[name]
    return PBRMaterial(name=name, baseColorFactor=[*rgb, 1.0], metallicFactor=metal,
                       roughnessFactor=rough, doubleSided=False)


# ---------------------------------------------------------------------------
# Outputs
# ---------------------------------------------------------------------------
def export_parts(links, world) -> trimesh.Scene:
    """Hierarchical scene: one node per link, meshes in link frames."""
    scene = trimesh.Scene()
    mats = {k: pbr(k) for k in MATERIALS}
    scene.graph.update(frame_to="eddie", frame_from=scene.graph.base_frame, matrix=URDF_TO_GLTF)
    for link in links:
        parent = "eddie" if link.parent is None else link.parent
        scene.graph.update(frame_to=link.name, frame_from=parent, matrix=link.origin)
        for vis in link.visuals:
            src = load_source(vis.source)
            # UVs projected in output space so density matches the static mesh
            uv_frame = URDF_TO_GLTF @ world[link.name]
            mesh = make_mesh(src.vertices, src.faces, vis.origin, uv_frame, mats[vis.material])
            scene.add_geometry(mesh, geom_name=vis.name, node_name=vis.name, parent_node_name=link.name)
    return scene


def is_moving(link: Link) -> bool:
    """Drive units (steering about the pivot) and their wheels."""
    return link.name.endswith("_drive") or link.name.endswith("_wheel")


def export_static(links, world, skip=frozenset()):
    """Baked, merged per material, WolvenKit submesh naming. `skip`: links left out."""
    mats = {k: pbr(k) for k in MATERIALS}
    per_mat: dict[str, list[trimesh.Trimesh]] = {}
    for link in links:
        if MOVING_PARTS and is_moving(link) or link.name in skip:
            continue
        for vis in link.visuals:
            src = load_source(vis.source)
            frame = URDF_TO_GLTF @ world[link.name] @ vis.origin
            per_mat.setdefault(vis.material, []).append(
                make_mesh(src.vertices, src.faces, frame, np.eye(4), mats[vis.material]))
    return submesh_scene(per_mat, mats)


def submesh_scene(per_mat: dict[str, list[trimesh.Trimesh]], mats):
    # the game's index buffers are 16 bit: keep every submesh under 65k vertices
    chunks: list[tuple[str, list[trimesh.Trimesh]]] = []
    for mat, parts in per_mat.items():
        current, count = [], 0
        for p in parts:
            if current and count + len(p.vertices) > MAX_SUBMESH_VERTS:
                chunks.append((mat, current))
                current, count = [], 0
            current.append(p)
            count += len(p.vertices)
        chunks.append((mat, current))

    scene = trimesh.Scene()
    table = []
    for i, (mat, parts) in enumerate(chunks):
        merged = trimesh.util.concatenate(parts)
        merged.visual = trimesh.visual.TextureVisuals(
            uv=np.concatenate([p.visual.uv for p in parts]), material=mats[mat])
        assert len(merged.vertices) <= MAX_SUBMESH_VERTS, (mat, len(merged.vertices))
        name = f"submesh_{i:02d}_LOD_1"
        scene.add_geometry(merged, geom_name=name, node_name=name)
        table.append({"submesh": name, "material": mat, "rgb": MATERIALS[mat][0],
                      "triangles": int(len(merged.faces)), "vertices": int(len(merged.vertices))})
    return scene, table


# URDF Z-up -> glTF Y-up, without the turn to "front = +Y": the part meshes keep their
# joint frame's axes, so after import (Y-up -> Z-up again) mesh space == joint frame
ZUP_TO_GLTF = np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1]], dtype=float)
REF_DRIVE = "front_right"  # an unmirrored unit; mirrored ones are this turned 180 deg about the pivot


def export_moving(links, world):
    """One drive housing and the two wheels, each in its own joint frame, plus where
    the four units sit. Returns {glb name: (scene, table)} and the layout for the mod."""
    mats = {k: pbr(k) for k in MATERIALS}
    by_name = {l.name: l for l in links}
    meshes = {}
    for mesh_name, link_name in (("eddie_drive", f"{REF_DRIVE}_drive"),
                                 ("eddie_wheel_r", f"{REF_DRIVE}_drive_r_wheel"),
                                 ("eddie_wheel_l", f"{REF_DRIVE}_drive_l_wheel")):
        per_mat: dict[str, list[trimesh.Trimesh]] = {}
        for vis in by_name[link_name].visuals:
            src = load_source(vis.source)
            per_mat.setdefault(vis.material, []).append(
                make_mesh(src.vertices, src.faces, ZUP_TO_GLTF @ vis.origin, np.eye(4), mats[vis.material]))
        meshes[mesh_name] = submesh_scene(per_mat, mats)

    # wheel radius from the tyre: spin axis is the wheel frame's x
    tyre = next(v for v in by_name[f"{REF_DRIVE}_drive_r_wheel"].visuals if v.name.endswith("_tyre"))
    pts = trimesh.transform_points(load_source(tyre.source).vertices, tyre.origin)
    radius = float(np.max(np.hypot(pts[:, 1], pts[:, 2])))

    ref = world[f"{REF_DRIVE}_drive"]
    offsets = {side: by_name[f"{REF_DRIVE}_drive_{side}_wheel"].origin[:3, 3].round(6).tolist()
               for side in ("r", "l")}
    drives = []
    for link in links:
        if not link.name.endswith("_drive"):
            continue
        T = world[link.name]
        assert np.allclose(T[:3, :3], ref[:3, :3]), link.name  # units only differ by position
        # mirrored units: the reference geometry turned half a turn about the pivot
        housing = link.visuals[0].origin
        turned = rot_z(math.pi) @ by_name[f"{REF_DRIVE}_drive"].visuals[0].origin
        mirrored = not np.allclose(housing, by_name[f"{REF_DRIVE}_drive"].visuals[0].origin, atol=1e-9)
        if mirrored:
            assert np.allclose(housing, turned, atol=1e-9), link.name
            r_wheel = by_name[f"{link.name}_r_wheel"].origin[:3, 3]
            assert np.allclose(r_wheel, rot_z(math.pi)[:3, :3] @ offsets["r"], atol=1e-6), link.name
        drives.append({"name": link.name.removesuffix("_drive"), "pivot": T[:3, 3].round(6).tolist(),
                       "steer0": math.pi if mirrored else 0.0})
    layout = {
        "frame": "URDF base_footprint (x forward, y left, z up); game entity space = this turned +90 deg about z",
        "drives": drives,
        "wheel_offsets": offsets,
        "wheel_spin_axis": "x of the drive frame",
        "wheel_radius": round(radius, 5),
    }
    return meshes, layout


# ---------------------------------------------------------------------------
# Arms as moving parts
# ---------------------------------------------------------------------------
ARM_TOOL = (0.0, 0.0, 0.13)  # between the fingertips, in the gripper base frame
# Gen3 7-DoF position limits (rad) of joints 2, 4 and 6; joints 1, 3, 5, 7 turn endlessly
GEN3_LIMITS = {2: 2.2497, 4: 2.5796, 6: 2.0996}
GRIP_CLOSED = 0.8
# Poses for the mod, as targets for the left gripper (the right arm mirrors y):
# tool point (URDF base frame, m), direction the fingers point, direction the fingers
# open along. "rest" is ARM_POSE. Solved by IK below, avoiding the torso, the chassis
# and V when she rides on top of the torso.
ARM_TARGETS = {
    "reach": {"pos": (0.62, 0.20, 0.66), "approach": (1, 0, 0), "open": (0, 0, 1)},
    "up": {"pos": (-0.12, 0.44, 1.42), "approach": (0, 0.25, 1), "open": (1, 0, 0)},
}
# "wave": the right arm raised to the side, the left one at rest
WAVE_TARGET = {"pos": (0.02, -0.50, 1.22), "approach": (0, -0.15, 1), "open": (1, 0, 0)}
WAVE_AMPLITUDE, WAVE_HZ = 0.45, 1.2  # rad, swings per second
RIDER = ((-0.185, 0.0), 0.95, 2.6, 0.30)  # V on the torso: centre (x, y), z from..to, radius


def arm_moving(links) -> set[str]:
    """Arm links that move: those with a joint, and everything below them."""
    by = {l.name: l for l in links}
    out = set()
    for l in links:
        n = l
        while n is not None:
            if n.joint:
                out.add(l.name)
                break
            n = by.get(n.parent)
    return out


def pose7(T: np.ndarray) -> list[float]:
    """4x4 -> [x, y, z, qi, qj, qk, qr]"""
    q = Rotation.from_matrix(T[:3, :3]).as_quat()
    return [round(float(v), 9) for v in (*T[:3, 3], *q)]


def from_pose7(p) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(p[3:]).as_matrix()
    T[:3, 3] = p[:3]
    return T


def axis_rot(axis, angle: float) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(np.asarray(axis, float) * angle).as_matrix()
    return T


def arm_fk(entries, q: dict) -> list[np.ndarray]:
    """World (URDF base frame) transform of each arm part for joint values q."""
    out = []
    for e in entries:
        T = from_pose7(e["origin"])
        if e["var"]:
            T = T @ axis_rot(e["axis"], e["scale"] * q[e["var"]])
        out.append(T if e["parent"] < 0 else out[e["parent"]] @ T)
    return out


def convex_planes(points: np.ndarray, inflate: float):
    h = trimesh.convex.convex_hull(points)
    n = h.face_normals
    d = np.einsum("ij,ij->i", n, h.triangles[:, 0]) + inflate
    return n, d


def penetration(pts: np.ndarray, obstacles) -> float:
    """Deepest point inside any obstacle (m), 0 if none is."""
    worst = 0.0
    for kind, a, b in obstacles:
        if kind == "hull":
            depth = -np.max(pts @ a.T - b, axis=1)
        else:  # vertical cylinder: a = (cx, cy, r), b = (z0, z1)
            rad = a[2] - np.hypot(pts[:, 0] - a[0], pts[:, 1] - a[1])
            depth = np.minimum(rad, np.minimum(pts[:, 2] - b[0], b[1] - pts[:, 2]))
        worst = max(worst, float(depth.max(initial=0.0)))
    return worst


def export_arms(links, world):
    """One mesh per arm link / gripper part (both arms share them), each in its own
    frame, plus the chain the mod evaluates: parent, fixed origin, joint axis and
    variable, the rest pose, and poses solved by IK."""
    mats = {k: pbr(k) for k in MATERIALS}
    by = {l.name: l for l in links}
    moving = arm_moving(links)
    entries, meshes, index, geometry = [], {}, {}, {}
    for l in links:  # parents come first
        if l.name not in moving or not l.visuals:
            continue
        side, rest = l.name.split("_arm_", 1)
        key = ("grip_" + rest.removeprefix("robotiq_85_") if rest.startswith("robotiq_85_")
               else "arm_" + rest).removesuffix("_link")
        mesh = f"eddie_{key}"
        sig = [(v.source, v.material, v.origin.round(9).tobytes()) for v in l.visuals]
        if mesh in geometry:
            assert geometry[mesh] == sig, mesh  # left and right use the same geometry
        else:
            geometry[mesh] = sig
            per_mat: dict[str, list[trimesh.Trimesh]] = {}
            for vis in l.visuals:
                src = load_source(vis.source)
                per_mat.setdefault(vis.material, []).append(
                    make_mesh(src.vertices, src.faces, ZUP_TO_GLTF @ vis.origin, np.eye(4), mats[vis.material]))
            meshes[mesh] = submesh_scene(per_mat, mats)
        # fixed transform from the parent part (or the base frame) to this link's joint
        origin = l.joint_origin if l.joint else l.origin
        p, parent = by[l.parent], -1
        while p.name not in index:
            if p.name not in moving:
                origin = world[p.name] @ origin
                break
            assert not p.joint and not p.visuals, p.name
            origin = p.origin @ origin
            p = by[p.parent]
        else:
            parent = index[p.name]
        index[l.name] = len(entries)
        var, axis, scale = l.joint if l.joint else (None, None, 0.0)
        entries.append({"name": f"{side[0]}_{key}", "link": l.name, "mesh": mesh, "parent": parent,
                        "origin": pose7(origin), "axis": axis, "var": var, "scale": scale,
                        "rest": pose7(world[l.name])})

    rest_q = {}
    for side in ("left", "right"):
        rest_q.update({f"{side[0]}{i}": float(v) for i, v in enumerate(ARM_POSE[side], 1)})
        rest_q[f"{side[0]}g"] = float(GRIPPER_CLOSURE)
    for e, T in zip(entries, arm_fk(entries, rest_q)):
        assert np.allclose(T, world[e["link"]], atol=1e-6), e["name"]
    poses, wave = arm_poses(links, world, entries, rest_q)
    layout = {"links": entries, "vars": rest_q, "poses": poses, "wave": wave,
              "tool": list(ARM_TOOL), "grip_closed": GRIP_CLOSED}
    return meshes, layout


def arm_obstacles(links, world, rider: bool):
    pts = {"chassis": [], "torso": []}
    for l in links:
        for vis in l.visuals:
            g = ("chassis" if vis.name in ("casing", "panel", "cover", "lidar") else
                 "torso" if vis.name in ("torso", "torso_mounts") else None)
            if g:
                pts[g].append(trimesh.transform_points(load_source(vis.source).vertices, world[l.name] @ vis.origin))
    obs = [("hull", *convex_planes(np.concatenate(p), 0.02)) for p in pts.values()]
    if rider:
        (cx, cy), z0, z1, r = RIDER
        obs.append(("cyl", (cx, cy, r), (z0, z1)))
    return obs


def arm_poses(links, world, entries, rest_q):
    """Solve ARM_TARGETS (and the wave) with IK and check that every pose, and the
    straight joint-space moves between any two of them, stay clear of the body."""
    from scipy.optimize import least_squares

    obstacles = arm_obstacles(links, world, rider=True)
    body = arm_obstacles(links, world, rider=False)
    # sample points of each part (its own frame); the shoulder and first half-arm sit
    # on the torso mount, so they're not checked against it
    samples = {}
    for i, e in enumerate(entries):
        if e["mesh"] in ("eddie_arm_shoulder", "eddie_arm_half_arm_1"):
            continue
        v = np.concatenate([trimesh.transform_points(load_source(vis.source).vertices, vis.origin)
                            for vis in next(l for l in links if l.name == e["link"]).visuals])
        samples[i] = v[:: max(1, len(v) // 40)]
    tool = np.array([*ARM_TOOL, 1.0])

    def clearance(q, obs, side=None):
        Ts = arm_fk(entries, q)
        worst = 0.0
        for i, p in samples.items():
            if side and entries[i]["name"][0] != side:
                continue
            worst = max(worst, penetration(trimesh.transform_points(p, Ts[i]), obs))
        return worst

    def solve(side, target):
        s = side[0]
        mirror = np.array([1, -1 if side == "right" else 1, 1])
        pos = np.array(target["pos"]) * mirror
        app = np.array(target["approach"], float) * mirror
        app /= np.linalg.norm(app)
        opn = np.array(target["open"], float) * mirror
        opn /= np.linalg.norm(opn)
        grip = next(i for i, e in enumerate(entries) if e["name"] == f"{s}_grip_base")
        names = [f"{s}{i}" for i in range(1, 8)]
        seed0 = np.array([rest_q[n] for n in names])

        def pen(x, obs):
            Ts = arm_fk(entries, dict(rest_q, **dict(zip(names, x))))
            return [penetration(trimesh.transform_points(p, Ts[i]), obs)
                    for i, p in samples.items() if entries[i]["name"][0] == s]

        def residual(x):
            T = arm_fk(entries, dict(rest_q, **dict(zip(names, x))))[grip]
            r = [(T @ tool)[:3] - pos, (T[:3, 2] - app) * 0.2, (np.abs(T[:3, 0] @ opn) - 1) * 0.05 * np.ones(1)]
            r.append(np.array(pen(x, obstacles)) * 5)
            # the straight joint-space move from rest has to clear the body too
            for t in (0.2, 0.4, 0.6, 0.8):
                r.append(np.array(pen(seed0 + t * (x - seed0), body)) * 5)
            r.append((x - seed0) * 0.03)
            return np.concatenate(r)

        lo = np.array([-GEN3_LIMITS.get(j, 2 * math.pi) for j in range(1, 8)]) + np.where(
            [j in GEN3_LIMITS for j in range(1, 8)], 0, seed0)
        hi = np.array([GEN3_LIMITS.get(j, 2 * math.pi) for j in range(1, 8)]) + np.where(
            [j in GEN3_LIMITS for j in range(1, 8)], 0, seed0)
        rng = np.random.default_rng(7)
        best = None
        for k in range(6):
            x0 = seed0 if k == 0 else np.clip(seed0 + rng.normal(0, 0.8, 7), lo + 1e-3, hi - 1e-3)
            sol = least_squares(residual, x0, bounds=(lo, hi), diff_step=1e-4, max_nfev=400)
            if best is None or sol.cost < best.cost:
                best = sol
        q = dict(rest_q, **dict(zip(names, best.x)))
        T = arm_fk(entries, q)[grip]
        err = np.linalg.norm((T @ tool)[:3] - pos)
        ang = math.degrees(math.acos(np.clip(T[:3, 2] @ app, -1, 1)))
        return {n: round(float(v), 5) for n, v in zip(names, best.x)}, err, ang

    poses = {"rest": {k: round(v, 5) for k, v in rest_q.items() if not k.endswith("g")}}
    report = []
    for name, target in ARM_TARGETS.items():
        pose = {}
        for side in ("left", "right"):
            q, err, ang = solve(side, target)
            pose.update(q)
            report.append(f"{name}/{side}: tool off by {err * 1000:.0f} mm, {ang:.0f} deg")
        poses[name] = pose
    wq, err, ang = solve("right", {**WAVE_TARGET, "pos": np.array(WAVE_TARGET["pos"]) * [1, -1, 1],
                                   "approach": np.array(WAVE_TARGET["approach"]) * [1, -1, 1],
                                   "open": WAVE_TARGET["open"]})
    report.append(f"wave/right: tool off by {err * 1000:.0f} mm, {ang:.0f} deg")
    poses["wave"] = {**{k: v for k, v in poses["rest"].items() if k[0] == "l"}, **wq}
    # wave with the joint whose axis points most along the robot's x: the hand swings sideways
    Ts = arm_fk(entries, dict(rest_q, **poses["wave"]))
    cand = [(abs(Ts[i][:3, 2] @ [1, 0, 0]), e["var"]) for i, e in enumerate(entries)
            if e["var"] in ("r4", "r5", "r6")]
    wave = {"var": max(cand)[1], "amplitude": WAVE_AMPLITUDE, "hz": WAVE_HZ}

    # every pose and the straight move between any two must clear the body (V only in poses)
    names = list(poses)
    for n in names:
        q = dict(rest_q, **poses[n])
        report.append(f"{n}: {clearance(q, obstacles) * 1000:.0f} mm into body/rider")
        for sgn in (-1, 1):
            qw = dict(q)
            if n == "wave":
                qw[wave["var"]] += sgn * WAVE_AMPLITUDE
                report.append(f"wave swing {sgn:+d}: {clearance(qw, obstacles) * 1000:.0f} mm")
    worst = []
    for a in names:
        for b in names:
            if a < b:
                qa, qb = dict(rest_q, **poses[a]), dict(rest_q, **poses[b])
                d = max(clearance({k: qa[k] + t * (qb[k] - qa[k]) for k in qa}, body)
                        for t in np.linspace(0, 1, 21))
                worst.append(f"{a}<->{b} {d * 1000:.0f} mm")
    report.append("moves into body: " + ", ".join(worst))
    print("  arm poses:\n    " + "\n    ".join(report))
    return poses, wave


def hull(points_urdf: np.ndarray, max_verts=64) -> trimesh.Trimesh:
    h = trimesh.convex.convex_hull(trimesh.transform_points(points_urdf, URDF_TO_GLTF))
    if len(h.vertices) > max_verts:
        v, f = fast_simplification.simplify(h.vertices, h.faces, target_count=max_verts * 2 - 4)
        h = trimesh.convex.convex_hull(v)
    return h


def export_collision(links, world):
    """Convex hulls: chassis, torso, one per drive unit (incl. wheels), lasers."""
    groups: dict[str, list[np.ndarray]] = {}
    for link in links:
        for vis in link.visuals:
            if link.collision_group:
                g = link.collision_group
            elif vis.name in ("casing", "panel", "cover", "lidar"):
                g = "chassis"
            elif vis.name in ("torso", "torso_mounts"):
                g = "torso"
            elif link.name.endswith("_laser"):
                g = link.name
            else:
                g = link.name.split("_drive")[0] + "_drive"
            src = load_source(vis.source)
            groups.setdefault(g, []).append(
                trimesh.transform_points(src.vertices, world[link.name] @ vis.origin))
    scene = trimesh.Scene()
    info = []
    for g, pts in groups.items():
        h = hull(np.concatenate(pts))
        name = f"collision_{g}"
        scene.add_geometry(h, geom_name=name, node_name=name)
        info.append({"name": name, "vertices": int(len(h.vertices)), "faces": int(len(h.faces)),
                     "volume_m3": round(float(h.volume), 5), "box_zup": oriented_box_zup(h.vertices)})
    return scene, info


# glTF (Y-up, front -Z) -> game / WolvenKit entity space (Z-up, front +Y)
GLTF_TO_ZUP = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=float)


def oriented_box_zup(points_gltf: np.ndarray) -> dict:
    """Tightest oriented box around the points, in the game's Z-up entity space,
    as physicsColliderBox wants it (centre, half extents, quaternion i j k r)."""
    pts = points_gltf @ GLTF_TO_ZUP.T
    T, extents = trimesh.bounds.oriented_bounds(pts)  # T moves the points to an axis-aligned box at the origin
    box_to_world = np.linalg.inv(T)
    R = box_to_world[:3, :3]
    if np.linalg.det(R) < 0:  # keep it a proper rotation
        R[:, 0] *= -1
    q = trimesh.transformations.quaternion_from_matrix(np.vstack([np.hstack([R, [[0], [0], [0]]]), [0, 0, 0, 1]]))
    w, x, y, z = q  # trimesh: w first
    return {"center": box_to_world[:3, 3].round(5).tolist(), "half_extents": (extents / 2).round(5).tolist(),
            "quat_ijkr": [round(float(x), 6), round(float(y), 6), round(float(z), 6), round(float(w), 6)]}


def mass_properties(links, world):
    """Total mass, COM and inertia about COM, in output (glTF) coordinates."""
    total, mc = 0.0, np.zeros(3)
    items = []
    for link in links:
        if link.mass <= 0:
            continue
        T = URDF_TO_GLTF @ world[link.name]
        c = trimesh.transform_points(np.array([link.com], float), T)[0]
        R = T[:3, :3]
        items.append((link.mass, c, R @ link.inertia @ R.T))
        total += link.mass
        mc += link.mass * c
    com = mc / total
    I = np.zeros((3, 3))
    for m, c, Ic in items:
        d = c - com
        I += Ic + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return total, com, I


def add_tangents(path: Path) -> None:
    """Add a TANGENT attribute to every primitive (WolvenKit's mesh import requires it)."""
    import pygltflib as G

    gltf = G.GLTF2().load(str(path))
    blob = bytearray(gltf.binary_blob())
    ncomp = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}
    dtype = {5121: np.uint8, 5123: np.uint16, 5125: np.uint32, 5126: np.float32}

    def read(i):
        acc = gltf.accessors[i]
        view = gltf.bufferViews[acc.bufferView]
        n, dt = ncomp[acc.type], np.dtype(dtype[acc.componentType])
        start = (view.byteOffset or 0) + (acc.byteOffset or 0)
        stride = view.byteStride or n * dt.itemsize
        raw = np.frombuffer(bytes(blob), dtype=np.uint8, count=stride * (acc.count - 1) + n * dt.itemsize,
                            offset=start)
        rows = np.lib.stride_tricks.as_strided(raw, (acc.count, n * dt.itemsize), (stride, 1))
        return np.ascontiguousarray(rows).view(dt).reshape(acc.count, n)

    for mesh in gltf.meshes:
        for prim in mesh.primitives:
            pos = read(prim.attributes.POSITION).astype(np.float64)
            nrm = read(prim.attributes.NORMAL).astype(np.float64)
            uv = read(prim.attributes.TEXCOORD_0).astype(np.float64)
            tri = read(prim.indices).reshape(-1, 3).astype(np.int64)
            e1, e2 = pos[tri[:, 1]] - pos[tri[:, 0]], pos[tri[:, 2]] - pos[tri[:, 0]]

            # repair zero-length normals (vertices only touched by sliver triangles)
            nlen = np.linalg.norm(nrm, axis=1)
            if (nlen < 0.5).any():
                acc_n = np.zeros_like(pos)
                fn = np.cross(e1, e2)
                for k in range(3):
                    np.add.at(acc_n, tri[:, k], fn)
                fix = nlen < 0.5
                alen = np.linalg.norm(acc_n[fix], axis=1, keepdims=True)
                nrm[fix] = np.where(alen > 1e-20, acc_n[fix] / np.maximum(alen, 1e-30), [[0.0, 1.0, 0.0]])
                nacc = gltf.accessors[prim.attributes.NORMAL]
                nview = gltf.bufferViews[nacc.bufferView]
                assert (nview.byteStride or 12) == 12 and nacc.componentType == G.FLOAT
                off = (nview.byteOffset or 0) + (nacc.byteOffset or 0)
                blob[off:off + 12 * len(nrm)] = nrm.astype(np.float32).tobytes()
            d1, d2 = uv[tri[:, 1]] - uv[tri[:, 0]], uv[tri[:, 2]] - uv[tri[:, 0]]
            det = d1[:, 0] * d2[:, 1] - d2[:, 0] * d1[:, 1]
            r = np.where(np.abs(det) > 1e-12, 1.0 / np.where(det == 0, 1, det), 0.0)[:, None]
            sdir = (e1 * d2[:, 1:2] - e2 * d1[:, 1:2]) * r
            tdir = (e2 * d1[:, 0:1] - e1 * d2[:, 0:1]) * r
            s_acc, t_acc = np.zeros_like(pos), np.zeros_like(pos)
            for k in range(3):
                np.add.at(s_acc, tri[:, k], sdir)
                np.add.at(t_acc, tri[:, k], tdir)
            t = s_acc - nrm * np.einsum("ij,ij->i", nrm, s_acc)[:, None]
            length = np.linalg.norm(t, axis=1)
            bad = length < 1e-12
            if bad.any():  # no usable UV gradient: any vector perpendicular to the normal
                helper = np.where(np.abs(nrm[bad, 0:1]) < 0.9, [[1.0, 0, 0]], [[0, 1.0, 0]])
                t[bad] = np.cross(nrm[bad], helper)
                length[bad] = np.linalg.norm(t[bad], axis=1)
            t /= length[:, None]
            assert np.isfinite(t).all(), f"non-finite tangents in {mesh.name}"
            w = np.where(np.einsum("ij,ij->i", np.cross(nrm, t), t_acc) < 0, -1.0, 1.0)
            data = np.concatenate([t, w[:, None]], axis=1).astype(np.float32)

            while len(blob) % 4:
                blob.append(0)
            gltf.bufferViews.append(G.BufferView(buffer=0, byteOffset=len(blob), byteLength=data.nbytes,
                                                 target=G.ARRAY_BUFFER))
            blob += data.tobytes()
            gltf.accessors.append(G.Accessor(bufferView=len(gltf.bufferViews) - 1, componentType=G.FLOAT,
                                             count=len(data), type=G.VEC4))
            prim.attributes.TANGENT = len(gltf.accessors) - 1

    gltf.buffers[0].byteLength = len(blob)
    gltf.set_binary_blob(bytes(blob))
    gltf.save(str(path))


def export_textures(used: list[str]) -> list[dict]:
    """Solid 64x64 textures per material, named for WolvenKit's Import tool
    (_d01 base colour, _r01 roughness, _m01 metalness), for engine\\materials\\metal_base.remt."""
    from PIL import Image

    tex_dir = OUT / "textures"
    tex_dir.mkdir(exist_ok=True)
    rows = []
    for mat in used:
        rgb, metal, rough = MATERIALS[mat]
        base = f"eddie_{mat}"
        Image.new("RGB", (64, 64), tuple(round(c * 255) for c in rgb)).save(tex_dir / f"{base}_d01.png")
        Image.new("L", (64, 64), round(rough * 255)).save(tex_dir / f"{base}_r01.png")
        Image.new("L", (64, 64), round(metal * 255)).save(tex_dir / f"{base}_m01.png")
        rows.append({"material": mat, "BaseColor": f"{base}_d01", "Roughness": f"{base}_r01",
                     "Metalness": f"{base}_m01"})
    return rows


def main():
    OUT.mkdir(exist_ok=True)
    links = build_tree()
    world = world_transforms(links)

    parts = export_parts(links, world)
    parts.export(OUT / "eddie_parts.glb", include_normals=True)

    arms = MOVING_PARTS and MOVING_ARMS and INCLUDE_ARMS
    static, table = export_static(links, world, arm_moving(links) if arms else frozenset())
    static.export(OUT / "eddie_static.glb", include_normals=True)
    add_tangents(OUT / "eddie_static.glb")

    coll, coll_info = export_collision(links, world)
    coll.export(OUT / "eddie_collision.glb")

    moving, layout, uses = ({}, None, {})
    for f in [*OUT.glob("eddie_drive.glb"), *OUT.glob("eddie_wheel_*.glb"),
              *OUT.glob("eddie_arm_*.glb"), *OUT.glob("eddie_grip_*.glb")]:
        f.unlink()
    if MOVING_PARTS:
        moving, layout = export_moving(links, world)
        uses = {n: 4 for n in moving}  # 4 drive units
        if arms:
            arm_meshes, layout["arms"] = export_arms(links, world)
            moving.update(arm_meshes)
            uses.update({n: 2 for n in arm_meshes})  # both arms
        for name, (scene, _) in moving.items():
            scene.export(OUT / f"{name}.glb", include_normals=True)
            add_tangents(OUT / f"{name}.glb")

    used = [t["material"] for t in table] + [t["material"] for _, tab in moving.values() for t in tab]
    textures = export_textures(list(dict.fromkeys(used)))

    mass, com, inertia_total = mass_properties(links, world)
    bounds = static.bounds
    joints = "drive pivots and wheels at zero"
    if INCLUDE_ARMS:
        joints += f"; arms (Kinova Gen3) at {ARM_POSE}"
        joints += f"; Robotiq 2F-85 closure {GRIPPER_CLOSURE}" if INCLUDE_GRIPPERS else "; no grippers"
    else:
        joints += "; arms not included"
    meta = {
        "units": "metres",
        "axes": "glTF Y-up; after import (Z-up) the robot front faces +Y",
        "origin": "floor, under the base_link centre",
        "joints": joints,
        "bounds_min": bounds[0].round(4).tolist(),
        "bounds_max": bounds[1].round(4).tolist(),
        "size_m": (bounds[1] - bounds[0]).round(4).tolist(),
        "mass_kg": round(mass, 4),
        "center_of_mass": com.round(4).tolist(),
        "inertia_about_com": inertia_total.round(6).tolist(),
        "triangles_total": int(sum(t["triangles"] for t in table)
                               # each part mesh is used 4 (drives) or 2 (arms) times
                               + sum(uses[n] * t["triangles"] for n, (_, tab) in moving.items() for t in tab)),
        "submeshes": table,
        "moving_parts": None if layout is None else {
            **layout, "meshes": {n: tab for n, (_, tab) in moving.items()}},
        "material_textures": textures,
        "collision_hulls": coll_info,
    }
    (OUT / "eddie_physics.json").write_text(json.dumps(meta, indent=2))
    print(json.dumps({k: meta[k] for k in ("size_m", "mass_kg", "center_of_mass", "triangles_total")}))
    for t in table:
        print(f"  {t['submesh']}  {t['material']:<12} {t['triangles']:>6} tris")
    for n, (_, tab) in moving.items():
        print(f"  {n}: " + ", ".join(f"{t['material']} {t['triangles']} tris" for t in tab))
    if layout:
        print(f"  wheel radius {layout['wheel_radius']} m, drives "
              + ", ".join(f"{d['name']}{' (mirrored)' if d['steer0'] else ''}" for d in layout["drives"]))
    for c in coll_info:
        print(f"  {c['name']:<32} {c['vertices']:>3} verts")


if __name__ == "__main__":
    main()
