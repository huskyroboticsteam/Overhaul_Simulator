#!/usr/bin/env python3
"""Generate a heavily simplified URDF of the dy2026 rover from the Onshape export.

The Onshape export has 213 links / 212 joints, most of which are artifacts of how
Onshape decomposes mates (ball, cylindrical, planar and parallel mates become chains
of dummy links with +-10000 limits, plus *_loop_closure links). This script:

  * keeps only the real degrees of freedom (listed in KEEP below),
  * merges every other link rigidly into its nearest kept ancestor (or an explicit
    group in ASSIGN), dropping massless dummy / loop-closure links,
  * replaces the meshes with one primitive per original part: a cylinder about the
    part's axis if its cross-section is round (tubes, shafts, discs, bearings, motors),
    else a box, link-aligned or rotated with the part when that is clearly tighter
    (angled struts). A bent / L / U-shaped, flanged or stepped part that one shape would
    bridge is cut into up to MAX_PIECES primitives where that removes enough empty space
    (space other parts fill doesn't count). Parts smaller than MIN_PART_SIZE keep their
    mass but get no geometry, aligned neighbouring shapes are merged, shapes mostly inside
    a bigger shape are dropped, and each link keeps only the shapes with the most surface,
  * ignores stray CAD in the root link (loose PCBs etc. placed around the assembly)
    by keeping only the parts connected to the main chassis,
  * combines masses, centres of mass and inertia tensors of merged parts,
  * re-expresses everything in a ROS-style base_link (X forward, Z up) instead of
    Onshape's world frame, with a base_footprint on the ground below it.

Joint zero positions match the original export, so joint values carry over.

Usage (needs numpy, scipy, networkx and trimesh: pip install -r requirements.txt):
    python3 simplify_dy2026.py [--source <onshape model dir>] [--out <urdf path>]
"""

import argparse
import math
import os
import re
import xml.etree.ElementTree as ET
from xml.dom import minidom

import numpy as np
import trimesh
# trimesh only soft-depends on these and silently degrades without them (oriented_bounds
# fails with a misleading error), so import them here to fail loudly instead
import networkx  # noqa: F401  (trimesh hull repair)
from scipy.spatial import ConvexHull, QhullError

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_SOURCE = os.path.join(HERE, '..', '..', 'models', 'dy2026_masterassembly')
DEFAULT_OUT = os.path.join(HERE, '..', '..', 'models', 'dy2026_simple', 'urdf', 'dy2026_simple.urdf.xacro')

# Parts whose bounding box is smaller than this (largest side, metres) get no geometry
MIN_PART_SIZE = 0.01
# A box rotated off the link axes is used only when it is this much smaller than the
# link-aligned one: keeps aligned parts exactly aligned (trimesh's oriented box is only
# accurate to a degree or so) but catches parts tilted by a few degrees (gripper: 7.5)
OBB_PREFERENCE = 0.9
# Cross-section roundness (hull area / enclosing circle area) from which a part is a
# cylinder about that axis: circle ~1, octagon 0.90, hex 0.83 (gearbox stages), NEO 550
# motor with its tabs 0.77, square 0.64. No whole part on this model falls between 0.74
# and 0.77 (a few cut pieces do).
ROUND_SECTION = 0.76
# Bent / L / U-shaped, flanged or tapered parts (at least MIN_SPLIT_VOLUME, m^3) are cut
# into up to MAX_PIECES pieces as long as each extra piece removes empty space worth
# 1 - SPLIT_GAIN of the shape. Space that other parts of the link fill doesn't count as
# removed: a housing or bracket around a motor stays one block. (Gentle tapers gain
# less: the chassis side plates' best 2 boxes are still 0.93 of one.)
MAX_PIECES = 3
SPLIT_GAIN = 0.85
MIN_SPLIT_VOLUME = 1e-5
# Cut positions tried along each axis of a part
CUT_BINS = 48
# Surface samples per mesh: vertices only sit on edges, so a cut through a long flat
# face would otherwise leave nothing on one side of it
SURFACE_SAMPLES = 3000
# A shape this much inside a kept one is dropped
MOSTLY_INSIDE = 0.8
# Two aligned shapes are merged when one shape around both is at most this much bigger
# than the two together (below 1 / SPLIT_GAIN, so a cut that paid off is rarely undone)
MERGE_SLACK = 1.1
# Only the largest shapes of each link are kept (by surface area). Cut parts need a few
# more slots than whole ones (10 lost the steering brackets' top arms); the wheels only
# need the tyre plus hub, so the total stays near 200 shapes.
MAX_SHAPES_PER_LINK = 14
MAX_SHAPES_BASE = 20
WHEEL_SHAPES = 3
# Root-link parts count as touching when their bounding boxes are within this distance
TOUCH_TOLERANCE = 0.005

# --- simplified kinematic tree -------------------------------------------------------
# new link: (parent new link, joint name, joint type, source joint, original head link,
#            colour). The new joint takes its frame/axis/limits from the source joint.
# 'type' / 'limits' / 'flip' override the source for mates Onshape split up.
# 'group' picks the effort/velocity/damping/friction xacro properties of the joint
# (<group>_effort etc., set in dy2026.urdf.xacro; JOINT_GROUP_DEFAULTS is the fallback).
STEER_LIMITS = (-math.pi / 2, math.pi / 2)
# The CAD leaves the elbow and wrist pitch unlimited; placeholders until measured
ELBOW_LIMITS = (-2.6, 2.6)
WRIST_PITCH_LIMITS = (-math.pi / 2, math.pi / 2)

KEEP = {
    'differential_bar': dict(
        group='suspension', parent='base_link', joint='differential_joint', src='revolute_2_5',
        head='60355k291_steel_ball_bearing_40', color='steel'),
    # Rockers pivot on cylindrical mates (the rotation half of each mate)
    'rocker_left': dict(
        group='suspension', parent='base_link', joint='rocker_left_joint', src='cylindrical_2_13',
        head='part_1_109', color='frame'),
    'rocker_right': dict(
        group='suspension', parent='base_link', joint='rocker_right_joint', src='cylindrical_1_7',
        head='part_1_63', color='frame'),
    # Steering: the export locks each wheel module with a "fixed angle" mate, so the
    # pivot comes from the turntable bearing (ttb_0115) mate on the same vertical axis
    'steer_front_left': dict(
        group='steer', parent='rocker_left', joint='steer_front_left_joint', src='revolute_1_12',
        head='part_1_119', color='module', type='revolute', limits=STEER_LIMITS),
    'steer_rear_left': dict(
        group='steer', parent='rocker_left', joint='steer_rear_left_joint', src='revolute_1_17',
        head='part_1_164', color='module', type='revolute', limits=STEER_LIMITS),
    'steer_rear_right': dict(
        group='steer', parent='rocker_right', joint='steer_rear_right_joint', src='revolute_1_27',
        head='part_1_91', color='module', type='revolute', limits=STEER_LIMITS),
    'steer_front_right': dict(
        group='steer', parent='rocker_right', joint='steer_front_right_joint', src='revolute_1_22',
        head='part_1_48', color='module', type='revolute', limits=STEER_LIMITS),
    'wheel_front_left': dict(
        group='wheel', parent='steer_front_left', joint='wheel_front_left_joint', src='revolute_1_10',
        head='part_1_138', color='tire', wheel=True),
    'wheel_rear_left': dict(
        group='wheel', parent='steer_rear_left', joint='wheel_rear_left_joint', src='revolute_1_15',
        head='part_1_169', color='tire', wheel=True),
    'wheel_rear_right': dict(
        group='wheel', parent='steer_rear_right', joint='wheel_rear_right_joint', src='revolute_1_25',
        head='wheel_1', color='tire', wheel=True),
    'wheel_front_right': dict(
        group='wheel', parent='steer_front_right', joint='wheel_front_right_joint', src='revolute_1_20',
        head='wheel', color='tire', wheel=True),
    'mast_lower': dict(
        group='mast', parent='base_link', joint='mast_lower_joint', src='slider_1_1',
        head='part_1_5', color='accent'),
    'mast_upper': dict(
        group='mast', parent='mast_lower', joint='mast_upper_joint', src='slider_2_1',
        head='part_1_8', color='accent'),
    # The carriage rides the lead screw; its travel comes from the nut's loop-closure mate
    # (screw -> nut, so the axis already points the way the carriage moves)
    'arm_carriage': dict(
        group='arm_rail', parent='base_link', joint='arm_rail_joint', src='cylindrical_1_loop_closure_4',
        head='a26_s102_railcrossbar_1', color='arm'),
    'arm_upper': dict(
        group='arm_shoulder', parent='arm_carriage', joint='arm_shoulder_joint', src='revolute_1_3',
        head='a26_s107_shaft', color='arm'),
    # Elbow: the forearm turns on the bicep motor axis (forearm -> motor mate, so flipped)
    'arm_forearm': dict(
        group='arm_elbow', parent='arm_upper', joint='arm_elbow_joint', src='cylindrical_1_loop_closure_1',
        head='a26_e103_forearmtube', color='arm', flip=True,
        type='revolute', limits=ELBOW_LIMITS),
    # Wrist roll: the whole wrist carrier turns about the forearm axis (forearm motor)
    'arm_wrist': dict(
        group='arm_wrist', parent='arm_forearm', joint='arm_wrist_roll_joint', src='revolute_1_1',
        head='a26_e107_driveshaftoutput', color='arm'),
    # Wrist pitch: bevel differential on the carrier. Both pinions turning together pitch
    # the centre block; turning against each other spins the crown gear, which drives the
    # claw lead screws (that motion is the claw joints, so the crown gear rides along here)
    'gripper': dict(
        group='arm_wrist', parent='arm_wrist', joint='arm_wrist_pitch_joint', src='cylindrical_3_3',
        head='a26_w104_centerblock', color='gripper',
        type='revolute', limits=WRIST_PITCH_LIMITS),
    # Claws slide on the gripper's linear rods (positive = open); the right claw mirrors the left
    'claw_left': dict(
        group='claw', parent='gripper', joint='claw_left_joint', src='cylindrical_1',
        head='a26_h102_leftclaw', color='claw', type='prismatic'),
    'claw_right': dict(
        group='claw', parent='gripper', joint='claw_right_joint', src='cylindrical_1',
        head='a26_h103_rightclaw', color='claw', type='prismatic', mimic='claw_left_joint'),
}

# Original links that belong to a group other than their nearest kept ancestor
ASSIGN = {
    # rear-left steering housing is reached through the right rocker in the export
    'top___f_fit___inside_labels_3': 'rocker_left',
    # rocker cranks (to the differential linkage) turn with their rockers
    'fillet8': 'rocker_right',
    'fillet8_2': 'rocker_left',
    # gripper body sits on the crown gear axis and pitches with the centre block
    # (the export routes it through the wrist carrier)
    'a26_h108_shaftcollar': 'gripper',
    'a26_h108_shaftcollar_1': 'gripper',
    'a26_h111_linearrod': 'gripper',
    'a26_h111_linearrod_1': 'gripper',
    'a26_h101_body': 'gripper',
    # claw guide bushings move with the claws
    'a26_h119_linearrodbearing_3': 'claw_left',
    'a26_h119_linearrodbearing_1': 'claw_left',
    'a26_h119_linearrodbearing': 'claw_right',
    'a26_h119_linearrodbearing_2': 'claw_right',
    # the lead-screw nut moves with the arm carriage
    'a26_s105_nutmount': 'arm_carriage',
}

# Differential linkage (a closed loop, so not in the URDF tree): each rocker crank pushes a
# rod that turns the differential bar. Source ball-joint mates at the two ends of each rod:
# rocker: (crank-end ball, bar-end ball). Used to work out the bar/rocker angle ratios.
DIFFERENTIAL = {
    'rocker_left': ('ball_1_loop_closure', 'ball_1_3'),
    'rocker_right': ('ball_1_6', 'ball_1'),
}

# Fallback joint properties per group (effort [N or Nm], velocity [m/s or rad/s],
# damping, friction); dy2026.urdf.xacro sets the real values.
JOINT_GROUP_DEFAULTS = dict(effort=100, velocity=1, damping=0, friction=0)

COLORS = {
    'chassis': (0.45, 0.47, 0.50, 1.0),
    'frame': (0.85, 0.45, 0.65, 1.0),
    'module': (0.80, 0.40, 0.60, 1.0),
    'tire': (0.15, 0.15, 0.15, 1.0),
    'steel': (0.70, 0.72, 0.75, 1.0),
    'accent': (0.85, 0.75, 0.15, 1.0),
    'arm': (0.30, 0.55, 0.80, 1.0),
    'gripper': (0.95, 0.55, 0.15, 1.0),
    'claw': (0.90, 0.30, 0.20, 1.0),
}


# --- math helpers --------------------------------------------------------------------

def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def matrix_rpy(R):
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:  # gimbal lock
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return roll, pitch, yaw


def origin_matrix(elem):
    T = np.eye(4)
    if elem is None:
        return T
    T[:3, :3] = rpy_matrix(*map(float, (elem.get('rpy') or '0 0 0').split()))
    T[:3, 3] = list(map(float, (elem.get('xyz') or '0 0 0').split()))
    return T


def frame_from_z(z, origin, hint=None):
    """Rotation whose Z axis is `z` (used to align cylinders and joint axes)."""
    z = z / np.linalg.norm(z)
    hint = np.array([1.0, 0, 0]) if hint is None or abs(np.dot(hint, z)) > 0.9 else hint
    if abs(np.dot(hint, z)) > 0.9:
        hint = np.array([0, 1.0, 0])
    x = np.cross(hint, z)
    x /= np.linalg.norm(x)
    T = np.eye(4)
    T[:3, :3] = np.column_stack([x, np.cross(z, x), z])
    T[:3, 3] = origin
    return T


def fmt(values):
    return ' '.join(f'{v:.6g}' if abs(v) > 1e-12 else '0' for v in values)


# --- source model ----------------------------------------------------------------------

class Source:

    def __init__(self, model_dir):
        self.model_dir = model_dir
        name = os.path.basename(os.path.normpath(model_dir))
        urdf = os.path.join(model_dir, 'urdf', name + '.urdf')
        if not os.path.exists(urdf):
            found = [f for f in os.listdir(os.path.join(model_dir, 'urdf')) if f.endswith('.urdf')]
            if len(found) != 1:
                raise FileNotFoundError(f'{urdf} not found and no single .urdf to fall back to: {found}')
            urdf = os.path.join(model_dir, 'urdf', found[0])
        root = ET.parse(urdf).getroot()
        self.links = {link.get('name'): link for link in root.findall('link')}
        self.joints = {j.get('name'): j for j in root.findall('joint')}
        self.parent_joint = {j.find('child').get('link'): j for j in self.joints.values()}
        children = set(self.parent_joint)
        roots = [n for n in self.links if n not in children]
        assert len(roots) == 1, roots
        self.root = roots[0]

        # Zero-configuration world pose of every link
        self.world = {self.root: np.eye(4)}
        kids = {}
        for j in self.joints.values():
            kids.setdefault(j.find('parent').get('link'), []).append(j)
        stack = [self.root]
        while stack:
            name = stack.pop()
            for j in kids.get(name, []):
                child = j.find('child').get('link')
                self.world[child] = self.world[name] @ origin_matrix(j.find('origin'))
                stack.append(child)
        self._meshes = {}

    def joint_frame(self, name):
        """World frame of a joint and its world axis."""
        j = self.joints[name]
        Tj = self.world[j.find('parent').get('link')] @ origin_matrix(j.find('origin'))
        axis_elem = j.find('axis')
        axis = np.array(list(map(float, axis_elem.get('xyz').split()))) \
            if axis_elem is not None else np.array([1.0, 0, 0])
        return Tj, Tj[:3, :3] @ axis

    def mesh_points(self, filename):
        """(vertices, surface samples) of a mesh file."""
        path = os.path.join(self.model_dir, 'meshes', os.path.basename(filename))
        if path not in self._meshes:
            scene = trimesh.load(path, force='scene')
            geom = scene.to_geometry() if hasattr(scene, 'to_geometry') else scene.dump(True)
            v = np.asarray(geom.vertices)
            if len(v) > 20000:  # moves no bound by more than 0.3 mm on this model
                v = v[:: len(v) // 20000]
            samples, _ = trimesh.sample.sample_surface(geom, SURFACE_SAMPLES, seed=0)
            self._meshes[path] = v, np.asarray(samples)
        return self._meshes[path]

    def visual_points(self, link_name, samples=False):
        """World-space vertices (plus surface samples) of each visual of a link."""
        out = []
        Wl = self.world[link_name]
        for visual in self.links[link_name].findall('visual'):
            mesh = visual.find('geometry/mesh')
            if mesh is None:
                continue
            v, s = self.mesh_points(mesh.get('filename'))
            if samples:
                v = np.vstack([v, s])
            Tw = Wl @ origin_matrix(visual.find('origin'))
            out.append((Tw[:3, :3] @ v.T).T + Tw[:3, 3])
        return out

    def inertial(self, link_name):
        """(mass, world COM, world inertia about COM) or None."""
        inertial = self.links[link_name].find('inertial')
        if inertial is None:
            return None
        mass = float(inertial.find('mass').get('value'))
        if mass <= 1e-6:
            return None
        Ti = self.world[link_name] @ origin_matrix(inertial.find('origin'))
        i = inertial.find('inertia')
        I_local = np.array([
            [float(i.get('ixx')), float(i.get('ixy')), float(i.get('ixz'))],
            [float(i.get('ixy')), float(i.get('iyy')), float(i.get('iyz'))],
            [float(i.get('ixz')), float(i.get('iyz')), float(i.get('izz'))]])
        R = Ti[:3, :3]
        return mass, Ti[:3, 3], R @ I_local @ R.T


# --- geometry fitting ----------------------------------------------------------------

def to_link(points, T_link_world):
    return points @ T_link_world[:3, :3].T + T_link_world[:3, 3]


def wheel_cylinder(local, axis):
    """Cylinder about a given axis through the link origin, enclosing the points (the tyres)."""
    T = frame_from_z(axis, np.zeros(3))
    p = local @ T[:3, :3]
    lo, hi = p.min(0), p.max(0)
    T[:3, 3] = T[:3, :3] @ np.array([0, 0, (lo[2] + hi[2]) / 2])
    # farthest vertex from the axle, not half the bounding box: a treaded tyre's box
    # depends on where the lugs sit, which made identical wheels differ by 0.3 mm
    radius = np.hypot(p[:, 0], p[:, 1]).max()
    # 'exact': never merged with anything, the rolling contact depends on it
    return dict(kind='cylinder', origin=T, radius=radius, length=hi[2] - lo[2], exact=True)


def cylinder_about(local, axis):
    """(roundness, cylinder) enclosing the points about `axis`.

    Roundness is the area of the section's convex hull over the area of the cylinder's
    circle: ~1 for anything round (tube, disc, gear, motor can), 0.64 for a square.
    """
    axis = axis * np.sign(axis[np.argmax(np.abs(axis))])  # same frame for +-axis
    T = frame_from_z(axis, np.zeros(3))
    p = local @ T[:3, :3]
    lo, hi = p.min(0), p.max(0)
    centre = (lo + hi) / 2  # the circle's centre whatever the in-plane rotation
    radius = np.linalg.norm(p[:, :2] - centre[:2], axis=1).max()
    try:
        area = ConvexHull(p[:, :2]).volume  # (a 2-D hull's "volume" is its area)
    except QhullError:  # section is a line: a sliver cut off a flat part, seen edge-on
        area = 0.0
    T[:3, 3] = T[:3, :3] @ centre
    return area / (math.pi * radius ** 2), dict(kind='cylinder', origin=T, radius=radius,
                                                  length=hi[2] - lo[2])


def fit_primitive(local):
    """Box or cylinder around points (link frame): dict(kind, origin, size | radius, length),
    or None if too small."""
    # Link-aligned box vs. minimum-volume oriented box: CAD parts are usually aligned with
    # the link, but struts, arm tubes and angled brackets are not
    lo, hi = local.min(0), local.max(0)
    aabb_T = np.eye(4)
    aabb_T[:3, 3] = (lo + hi) / 2
    obb_T, obb_ext = trimesh.bounds.oriented_bounds(local)
    obb_T = np.linalg.inv(obb_T)
    if np.linalg.det(obb_T[:3, :3]) < 0:
        # trimesh returns a left-handed frame for flat point sets; a box is symmetric,
        # so flipping one axis keeps the shape and makes the rpy conversion valid
        obb_T[:3, 0] *= -1
    if np.prod(obb_ext + 1e-3) < OBB_PREFERENCE * np.prod(hi - lo + 1e-3):
        T, ext = obb_T, np.asarray(obb_ext)
    else:
        T, ext = aabb_T, hi - lo
    if ext.max() < MIN_PART_SIZE:
        return None

    # Round parts, long or short, are cylinders: the tightest one about an axis with a round
    # section. A round part's axis is an axis of its oriented box, and usually of the link
    # frame too: a link axis that fits about as tightly wins, so aligned cylinders don't
    # come out a hair off-axis
    fits = [cylinder_about(local, axis) for R in (np.eye(3), obb_T[:3, :3]) for axis in R.T]
    round_fits = [c for roundness, c in fits if roundness >= ROUND_SECTION]
    if round_fits:
        tightest = min(map(volume, round_fits))
        return next(c for c in round_fits if volume(c) <= 1.02 * tightest)
    return dict(kind='box', origin=T, size=ext)


def best_cuts(local, R, max_slabs):
    """Best ways to slice the points into 2 (and 3) slabs across an axis of frame R.

    Returns [(total volume of the slabs' boxes / volume of the whole box, slab index per
    point)], boxes aligned with R. Two cuts catch dumbbells (a shaft between two flanges),
    which no single cut improves. Cuts are searched on CUT_BINS bins per axis.
    """
    q = local @ R
    n = CUT_BINS
    best = {}
    for k in range(3):
        edges = np.linspace(q[:, k].min(), q[:, k].max(), n + 1)
        b = np.clip(np.searchsorted(edges, q[:, k], side='right') - 1, 0, n - 1)
        lo, hi = np.full((n, 3), np.inf), np.full((n, 3), -np.inf)
        np.minimum.at(lo, b, q)
        np.maximum.at(hi, b, q)
        # vol[i, j]: box around bins i..j-1, inf if empty or too small to get geometry
        vol = np.full((n + 1, n + 1), np.inf)
        for i in range(n):
            ext = np.maximum.accumulate(hi[i:]) - np.minimum.accumulate(lo[i:])
            ok = np.isfinite(ext).all(1) & (ext.max(1) >= MIN_PART_SIZE)
            vol[i, i + 1:][ok] = np.prod(ext[ok], 1)
        two = vol[0, :] + vol[:, n]                        # cut at c
        three = vol[0, :, None] + vol + vol[None, :, n]    # cuts at a < c
        for slabs, total in ((2, two), (3, three)):
            if slabs > max_slabs:
                continue
            idx = np.unravel_index(np.argmin(total), total.shape)
            if total[idx] < best.get(slabs, (np.inf,))[0]:
                best[slabs] = (total[idx], sum(b >= c for c in idx))
    whole = max(np.prod(q.max(0) - q.min(0)), 1e-12)
    return [(v / whole, labels) for v, labels in best.values() if np.isfinite(v)]


def fit_part(local, first, others=()):
    """Primitives for one part: usually one, but a part that a single shape would bridge
    (bent, L / U-shaped, flanged, stepped, dumbbell) is cut into a few pieces that each fit.

    `first` is the part's single primitive, `others` the single primitives of the link's
    other parts. What a cut would remove only counts where no other part fills it: a
    housing or bracket around a motor stays one shape instead of opening up.
    """
    def splittable(prim):
        return prim is not None and volume(prim) >= MIN_SPLIT_VOLUME

    def filled(prim, fits):
        """Volume of `prim` outside `fits` that other parts' shapes occupy."""
        if not others:
            return 0.0
        pts = sample_inside(prim, 2000)
        gone = pts[~np.any([contains(f, pts, 0) for f in fits], axis=0)]
        if not len(gone):
            return 0.0
        return np.any([contains(q, gone, 0) for q in others], axis=0).sum() / 2000 * volume(prim)

    pieces = [(local, first, splittable(first))]
    while len(pieces) < MAX_PIECES:
        candidates = [i for i, (_, _, ok) in enumerate(pieces) if ok]
        if not candidates:
            break
        # split the biggest piece first; if no cut pays off, that piece stays whole
        i = max(candidates, key=lambda i: volume(pieces[i][1]))
        pts, prim, _ = pieces[i]
        # Cut across the piece's own axes or the link's, judged by the slabs' real fits.
        # Every extra piece has to shrink the total to SPLIT_GAIN of what it was, with
        # space that other parts fill counted as not removed.
        best = (volume(prim), None)
        for R in (prim['origin'][:3, :3], np.eye(3)):
            for ratio, labels in best_cuts(pts, R, MAX_PIECES - len(pieces) + 1):
                if ratio > SPLIT_GAIN:  # not even the rough boxes get small enough
                    continue
                slabs = [pts[labels == j] for j in range(labels.max() + 1)]
                fits = [fit_primitive(s) for s in slabs]
                if not all(fits):
                    continue
                kept = sum(map(volume, fits))
                if kept / SPLIT_GAIN ** (len(fits) - 1) >= best[0]:
                    continue
                score = (kept + filled(prim, fits)) / SPLIT_GAIN ** (len(fits) - 1)
                if score < best[0]:
                    best = (score, list(zip(slabs, fits)))
        if best[1]:
            pieces[i:i + 1] = [(s, f, splittable(f)) for s, f in best[1]]
        else:
            pieces[i] = (pts, prim, False)
    return [prim for _, prim, _ in pieces if prim is not None]


# --- main ----------------------------------------------------------------------------

def kinematics(src):
    """The simplified model without its geometry, as a dict: group (source link -> new
    link), root_keep (which root visuals are chassis, not stray CAD), W_new (world frame of
    each new link), joints, links (combined mass / com / I and source parts per new link),
    color_of, height (base_link above base_footprint), wheels (the tyre cylinder per wheel
    link) and ratios (differential linkage). Shared with simplify_dy2026_meshes.py."""
    # Group membership: explicit, kept head, or inherited from the parent link
    head_of = {spec['head']: name for name, spec in KEEP.items()}
    group = {src.root: 'base_link'}

    def assign(link):
        if link in group:
            return group[link]
        if link in ASSIGN:
            g = ASSIGN[link]
        elif link in head_of:
            g = head_of[link]
        else:
            g = assign(src.parent_joint[link].find('parent').get('link'))
        group[link] = g
        return g

    for link in src.links:
        assign(link)

    root_parts = src.visual_points(src.root)
    other_parts = [p for n in src.links if n != src.root for p in src.visual_points(n)]
    root_keep = attached_parts(root_parts, other_parts)

    # Chassis frame: up = differential pivot axis, lateral = rocker pivot axis
    _, up = src.joint_frame(KEEP['differential_bar']['src'])
    up = -up / np.linalg.norm(up)
    _, lateral = src.joint_frame(KEEP['rocker_left']['src'])
    lateral -= up * np.dot(lateral, up)
    lateral /= np.linalg.norm(lateral)
    forward = np.cross(lateral, up)
    # The arm reaches forward
    chassis_pts = np.vstack([p for p, keep in zip(root_parts, root_keep) if keep])
    centre = (chassis_pts.min(0) + chassis_pts.max(0)) / 2
    if np.dot(src.world[KEEP['gripper']['head']][:3, 3] - centre, forward) < 0:
        forward, lateral = -forward, -lateral
    R_base = np.column_stack([forward, np.cross(up, forward), up])
    # base_link sits at the centre of the chassis box (in chassis-aligned coordinates)
    c_local = chassis_pts @ R_base
    base_origin = R_base @ ((c_local.min(0) + c_local.max(0)) / 2)
    W_new = {'base_link': np.eye(4)}
    W_new['base_link'][:3, :3] = R_base
    W_new['base_link'][:3, 3] = base_origin

    # Joints and new link frames (child frame = joint frame at zero)
    joints = []
    for name, spec in KEEP.items():
        Tj, axis = src.joint_frame(spec['src'])
        if spec.get('flip'):
            axis = -axis
        sj = src.joints[spec['src']]
        jtype = spec.get('type', sj.get('type'))
        limit = sj.find('limit')
        lower = upper = None
        if limit is not None and jtype in ('revolute', 'prismatic'):
            # (with 'flip' the axis is reversed but the limits stay: the source mate moves
            # the screw relative to the nut, i.e. the nut by the same amount the other way)
            lower, upper = float(limit.get('lower')), float(limit.get('upper'))
        if 'limits' in spec:
            lower, upper = spec['limits']
        # Frame: joint position of the source mate, Z along the joint axis
        frame = frame_from_z(axis, Tj[:3, 3], hint=R_base[:, 0])
        if name.startswith('claw_'):
            # put each claw's frame at its own part, sliding along the shared rod axis
            frame[:3, 3] = src.world[spec['head']][:3, 3]
        W_new[name] = frame
        joints.append(dict(name=spec['joint'], type=jtype, parent=spec['parent'], child=name,
                           lower=lower, upper=upper, mimic=spec.get('mimic'),
                           group=spec['group']))

    # Claws: positive travel opens the gripper (moves the claws apart); the right claw
    # mirrors the left
    left = src.world[KEEP['claw_left']['head']][:3, 3]
    right = src.world[KEEP['claw_right']['head']][:3, 3]
    z_left = W_new['claw_left'][:3, 2]
    if np.dot(right - left, z_left) > 0:
        W_new['claw_left'] = frame_from_z(-z_left, W_new['claw_left'][:3, 3], R_base[:, 0])
        for j in joints:
            if j['name'] == 'claw_left_joint':
                j['lower'], j['upper'] = -j['upper'], -j['lower']
    W_new['claw_right'] = frame_from_z(
        -W_new['claw_left'][:3, 2], W_new['claw_right'][:3, 3], R_base[:, 0])
    jl = next(j for j in joints if j['name'] == 'claw_left_joint')
    for j in joints:
        if j['name'] == 'claw_right_joint':
            j['lower'], j['upper'] = jl['lower'], jl['upper']

    # Combined inertia per new link, from every source link (geometry kept or not), and
    # the tyre cylinders
    links = {name: dict(mass=0.0, com=np.zeros(3), I=np.zeros((3, 3)), parts=[])
             for name in W_new}
    color_of = {name: spec['color'] for name, spec in KEEP.items()}
    color_of['base_link'] = 'chassis'
    inertials = {name: [] for name in W_new}
    wheels = {}
    for link_name, g in group.items():
        links[g]['parts'].append(link_name)
        inertial = src.inertial(link_name)
        if inertial is not None:
            inertials[g].append(inertial)
        spec = KEEP.get(g, {})
        if spec.get('wheel') and link_name == spec['head']:
            # the tyre (each wheel's head link): one cylinder about the wheel axis, from
            # the mesh vertices only, so its radius is the CAD's outermost tread
            points = src.visual_points(link_name)
            if points:
                wheels[g] = wheel_cylinder(to_link(np.vstack(points), np.linalg.inv(W_new[g])),
                                           np.array([0, 0, 1.0]))
    for name, L in links.items():
        if inertials[name]:
            L['mass'], L['com'], L['I'] = combine_inertials(inertials[name], W_new[name])

    # base_footprint: on the ground (lowest wheel point) straight below base_link
    wheel_pts = np.vstack([p for n, g in group.items() if KEEP.get(g, {}).get('wheel')
                           for p in src.visual_points(n)])
    height = np.dot(base_origin - wheel_pts[np.argmin(wheel_pts @ up)], up)

    return dict(group=group, root_keep=root_keep, W_new=W_new, joints=joints, links=links,
                color_of=color_of, height=height, wheels=wheels, ratios=differential_ratios(src))


def combine_inertials(inertials, W_link):
    """(mass, com, inertia about the com) in the link frame W_link of several (mass, world
    com, world inertia about own com)."""
    inv = np.linalg.inv(W_link)
    R = inv[:3, :3]
    m = sum(i[0] for i in inertials)
    com_w = sum(i[0] * i[1] for i in inertials) / m
    I = np.zeros((3, 3))
    for mi, ci, Ii in inertials:
        d = ci - com_w
        I += Ii + mi * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return m, R @ com_w + inv[:3, 3], R @ I @ R.T


def fit_shapes(src, kin):
    """Fill each new link's 'visuals' with primitives fitted to its parts' meshes."""
    links, W_new, root_keep = kin['links'], kin['W_new'], kin['root_keep']
    points = {name: [] for name in links}
    for name, L in links.items():
        L['visuals'] = [kin['wheels'][name]] if name in kin['wheels'] else []
    for link_name, g in kin['group'].items():
        spec = KEEP.get(g, {})
        if spec.get('wheel') and link_name == spec['head']:
            continue  # the tyre is the wheel cylinder
        visual_sets = src.visual_points(link_name, samples=True)
        if link_name == src.root:
            visual_sets = [p for p, keep in zip(visual_sets, root_keep) if keep]
        inv = np.linalg.inv(W_new[g])
        points[g].extend(to_link(pts, inv) for pts in visual_sets)

    # One primitive per part (a few for bent parts), fitted once all of a link's parts are
    # known. Then merge aligned neighbours and keep the most visible few.
    for name, L in links.items():
        parts = points[name]
        firsts = [fit_primitive(pts) for pts in parts]
        lo = np.array([p.min(0) for p in parts]).reshape(-1, 3)
        hi = np.array([p.max(0) for p in parts]).reshape(-1, 3)
        for i, (pts, first) in enumerate(zip(parts, firsts)):
            near = np.all((lo <= hi[i]) & (hi >= lo[i]), axis=1)
            near[i] = False
            others = [firsts[j] for j in np.nonzero(near)[0] if firsts[j] is not None]
            L['visuals'].extend(fit_part(pts, first, others))
        cap = MAX_SHAPES_BASE if name == 'base_link' else \
            WHEEL_SHAPES if KEEP[name].get('wheel') else MAX_SHAPES_PER_LINK
        L['visuals'] = keep_visible(merge_shapes(L['visuals']), cap)


def build(src):
    kin = kinematics(src)
    fit_shapes(src, kin)
    return (kin['W_new'], kin['joints'], kin['links'], kin['color_of'], kin['height'],
            kin['ratios'])


def differential_ratios(src):
    """Bar angle per rocker angle for each rocker, around the zero pose.

    The rod between the crank ball and the bar ball keeps its length, so the two ball
    velocities agree along the rod: u.(a_r x r_c) d(rocker) = u.(a_b x r_b) d(bar).
    Angles use the generated joints' axes (same as the source mates' axes).
    """
    T_bar, a_bar = src.joint_frame(KEEP['differential_bar']['src'])
    ratios = {}
    for rocker, (crank_ball, bar_ball) in DIFFERENTIAL.items():
        T_rocker, a_rocker = src.joint_frame(KEEP[rocker]['src'])
        if KEEP[rocker].get('flip'):
            a_rocker = -a_rocker
        p_crank = src.joint_frame(crank_ball)[0][:3, 3]
        p_bar = src.joint_frame(bar_ball)[0][:3, 3]
        u = p_crank - p_bar
        crank = np.dot(u, np.cross(a_rocker, p_crank - T_rocker[:3, 3]))
        bar = np.dot(u, np.cross(a_bar, p_bar - T_bar[:3, 3]))
        ratios[KEEP[rocker]['joint']] = crank / bar
    return ratios


def attached_parts(parts, anchors):
    """Mask of `parts` connected, through touching bounding boxes, to any anchor part.

    The chassis touches the moving links (rocker pivots, arm base, ...); stray CAD
    floating around the assembly does not.
    """
    everything = list(parts) + list(anchors)
    lo = np.array([p.min(0) for p in everything]) - TOUCH_TOLERANCE
    hi = np.array([p.max(0) for p in everything]) + TOUCH_TOLERANCE
    parent = list(range(len(everything)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for i in range(len(everything)):
        touching = np.nonzero(np.all((lo[i] <= hi) & (hi[i] >= lo), axis=1))[0]
        for j in touching:
            parent[find(j)] = find(i)
    anchored = {find(i) for i in range(len(parts), len(everything))}
    return np.array([find(i) in anchored for i in range(len(parts))])


def sample_inside(prim, n):
    """n random (but repeatable) points spread evenly through the shape, in the link frame."""
    u = np.random.default_rng(0).random((n, 3))
    if prim['kind'] == 'box':
        local = (u - 0.5) * prim['size']
    else:
        r, t = prim['radius'] * np.sqrt(u[:, 0]), 2 * math.pi * u[:, 1]
        local = np.column_stack([r * np.cos(t), r * np.sin(t), (u[:, 2] - 0.5) * prim['length']])
    return to_link(local, prim['origin'])


def contains(prim, points, tol=1e-3):
    """Mask of the points (link frame) inside the shape grown by tol."""
    local = to_link(points, np.linalg.inv(prim['origin']))
    if prim['kind'] == 'box':
        return np.all(np.abs(local) <= prim['size'] / 2 + tol, axis=1)
    return ((np.hypot(local[:, 0], local[:, 1]) <= prim['radius'] + tol)
            & (np.abs(local[:, 2]) <= prim['length'] / 2 + tol))


def hull_points(p):
    """Points (in the link frame) whose convex hull is the shape (16-gon for cylinders)."""
    if p['kind'] == 'box':
        e = p['size'] / 2
        c = np.array([[sx * e[0], sy * e[1], sz * e[2]]
                      for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    else:
        t = np.linspace(0, 2 * math.pi, 16, endpoint=False)
        ring = p['radius'] * np.column_stack([np.cos(t), np.sin(t)])
        c = np.vstack([np.column_stack([ring, np.full(16, z)])
                       for z in (-p['length'] / 2, p['length'] / 2)])
    return to_link(c, p['origin'])


def near_pairs(prims):
    """Index pairs (i < j) of shapes whose bounding spheres overlap."""
    centre = np.array([p['origin'][:3, 3] for p in prims]).reshape(-1, 3)
    reach = np.array([np.linalg.norm(extent(p)) / 2 for p in prims])
    dist = np.linalg.norm(centre[:, None] - centre[None], axis=2)
    return zip(*np.nonzero(np.triu(dist <= reach[:, None] + reach[None] + 1e-3, 1)))


def keep_visible(prims, cap):
    """The `cap` shapes with the most surface, skipping any lying (mostly) inside one
    already kept: hidden internals, and bolt heads or hub bits poking a few mm out of
    something big, which would only use up slots. (A thin plate, or one piece of a bent
    one, matters as much as a solid block, hence surface area rather than volume.)"""
    kept = []
    for p in sorted(prims, key=lambda p: -area(p)):
        if len(kept) == cap:
            break
        pts = sample_inside(p, 200)
        if not any(contains(q, pts).mean() >= MOSTLY_INSIDE for q in kept):
            kept.append(p)
    return kept


def union(a, b):
    """Shape of a's kind and orientation around both a and b, or None if b isn't aligned."""
    Ra, Rb = a['origin'][:3, :3], b['origin'][:3, :3]
    if a['kind'] != b['kind'] or (
            not np.all(np.abs(Ra.T @ Rb).max(0) > 0.9998) if a['kind'] == 'box'
            else abs(Ra[:, 2] @ Rb[:, 2]) < 0.9998):
        return None
    pts = to_link(np.vstack([hull_points(a), hull_points(b)]), np.linalg.inv(a['origin']))
    lo, hi = pts.min(0), pts.max(0)
    T = a['origin'].copy()
    if a['kind'] == 'box':
        T[:3, 3] = to_link(((lo + hi) / 2)[None], a['origin'])[0]
        return dict(kind='box', origin=T, size=hi - lo)
    T[:3, 3] += Ra[:, 2] * (lo[2] + hi[2]) / 2  # stays on a's axis
    return dict(kind='cylinder', origin=T, radius=np.hypot(pts[:, 0], pts[:, 1]).max(),
                length=hi[2] - lo[2])


def merge_shapes(prims):
    """Merge aligned shapes of a link that one shape covers with little to spare: a tube
    that came out as several segments, coaxial discs, plates continuing one another."""
    prims = list(prims)
    # judged against the volume of the shapes it replaces, so merges don't creep
    covers = [volume(p) for p in prims]
    while True:
        for i, j in near_pairs(prims):
            if prims[i].get('exact') or prims[j].get('exact'):
                continue
            u = union(prims[i], prims[j])
            if u and volume(u) <= MERGE_SLACK * (covers[i] + covers[j]):
                prims[i], covers[i] = u, covers[i] + covers[j]
                del prims[j], covers[j]
                break
        else:
            return prims


def extent(p):
    if p['kind'] == 'box':
        return p['size']
    return np.array([2 * p['radius'], 2 * p['radius'], p['length']])


def volume(p):
    if p['kind'] == 'box':
        return float(np.prod(p['size']))
    return math.pi * p['radius'] ** 2 * p['length']


def area(p):
    if p['kind'] == 'box':
        a, b, c = p['size']
        return 2 * (a * b + b * c + a * c)
    return 2 * math.pi * p['radius'] * (p['radius'] + p['length'])


XACRO_NS = 'http://www.ros.org/wiki/xacro'
INDENT = ' ' * 4
LINE_WIDTH = 100
# Placeholder comment replaced by two blank lines when formatting
_GAP = 'GAP'


def gap(parent):
    parent.append(ET.Comment(_GAP))


def format_xml(robot):
    """Pretty-print with 4-space indents, wrapped long tags and blank lines between groups."""
    xml = minidom.parseString(ET.tostring(robot)).toprettyxml(indent=INDENT)
    out = []
    for line in xml.splitlines():
        if not line.strip():
            continue
        if line.strip() == f'<!--{_GAP}-->':
            out.extend(['', ''])
            continue
        out.extend(wrap_tag(line))
    # no gap before the closing </robot>
    while len(out) > 1 and not out[-2].strip():
        out.pop(-2)
    return '\n'.join(out) + '\n'


def wrap_tag(line):
    """Split an over-long single tag so each continuation line holds whole attributes."""
    if len(line) <= LINE_WIDTH:
        return [line]
    m = re.match(r'^(\s*)(<[^\s>]+)((?:\s+[^\s=]+="[^"]*")+)\s*(/?>)$', line)
    if not m:
        return [line]
    lead, tag, attrs, end = m.groups()
    attrs = re.findall(r'[^\s=]+="[^"]*"', attrs)
    cont = lead + INDENT * 2
    lines = [lead + tag + ' ' + attrs[0]]
    for i, a in enumerate(attrs[1:], 2):
        tail = len(end) if i == len(attrs) else 0
        if len(lines[-1]) + 1 + len(a) + tail <= LINE_WIDTH:
            lines[-1] += ' ' + a
        else:
            lines.append(cont + a)
    lines[-1] += end
    return lines


def add_properties(robot, joints, ratios):
    """Per-group joint properties (defaults only: a file including this one overrides them
    by defining them first) and the differential linkage ratios."""
    robot.append(ET.Comment(
        ' Joint properties per group. These are fallbacks: define the same names before\n'
        f'{INDENT}     including this file (see dy2026.urdf.xacro) to set the real values. '))
    for g in dict.fromkeys(j['group'] for j in joints):
        for key, value in JOINT_GROUP_DEFAULTS.items():
            ET.SubElement(robot, 'xacro:property', name=f'{g}_{key}', default=str(value))
    gap(robot)

    robot.append(ET.Comment(
        ' Differential linkage (a closed loop, so not part of the tree above): bar angle per\n'
        f'{INDENT}     rocker angle, from the CAD. differential_joint = ratio * rocker angle, for\n'
        f'{INDENT}     each rocker. Used by the rocker differential Gazebo plugin. '))
    for joint, ratio in ratios.items():
        ET.SubElement(robot, 'xacro:property', name=f'{joint}_differential_ratio',
                      value=f'{ratio:.6g}')
    gap(robot)


def add_macros(robot):
    """xacro macros so each link/joint reads as a few one-line calls."""

    # One shape = a visual plus an identical collision
    for kind, params, geom_attrs in (
            ('box', 'size', dict(size='${size}')),
            ('cylinder', 'radius length', dict(radius='${radius}', length='${length}'))):
        macro = ET.SubElement(robot, 'xacro:macro', name=f'{kind}_shape',
                              params=f"xyz {params} material rpy:='0 0 0'")
        for tag in ('visual', 'collision'):
            el = ET.SubElement(macro, tag)
            ET.SubElement(el, 'origin', xyz='${xyz}', rpy='${rpy}')
            ET.SubElement(ET.SubElement(el, 'geometry'), kind, **geom_attrs)
            if tag == 'visual':
                ET.SubElement(el, 'material', name='${material}')
        gap(robot)

    macro = ET.SubElement(robot, 'xacro:macro', name='inertial',
                          params='xyz mass ixx ixy ixz iyy iyz izz')
    el = ET.SubElement(macro, 'inertial')
    ET.SubElement(el, 'origin', xyz='${xyz}', rpy='0 0 0')
    ET.SubElement(el, 'mass', value='${mass}')
    ET.SubElement(el, 'inertia', **{k: '${%s}' % k for k in
                                    ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz')})
    gap(robot)

    # Every moving joint turns/slides about its own Z axis
    macro = ET.SubElement(robot, 'xacro:macro', name='z_joint',
                          params="name type parent child xyz rpy effort velocity damping friction "
                                 "lower:=0 upper:=0 mimic:=''")
    el = ET.SubElement(macro, 'joint', name='${name}', type='${type}')
    ET.SubElement(el, 'origin', xyz='${xyz}', rpy='${rpy}')
    ET.SubElement(el, 'parent', link='${parent}')
    ET.SubElement(el, 'child', link='${child}')
    ET.SubElement(el, 'axis', xyz='0 0 1')
    cond = ET.SubElement(el, 'xacro:if', value="${type in ('revolute', 'prismatic')}")
    ET.SubElement(cond, 'limit', lower='${lower}', upper='${upper}',
                  effort='${effort}', velocity='${velocity}')
    # continuous joints: effort/velocity limits only
    cond = ET.SubElement(el, 'xacro:if', value="${type == 'continuous'}")
    ET.SubElement(cond, 'limit', effort='${effort}', velocity='${velocity}')
    ET.SubElement(el, 'dynamics', damping='${damping}', friction='${friction}')
    cond = ET.SubElement(el, 'xacro:if', value="${mimic != ''}")
    ET.SubElement(cond, 'mimic', joint='${mimic}', multiplier='1', offset='0')
    gap(robot)


def primitive_shapes(link, name, L, material):
    """A link's fitted primitives as xacro:box_shape / xacro:cylinder_shape calls."""
    for prim in L['visuals']:
        attrs = dict(xyz=fmt(prim['origin'][:3, 3]))
        if prim['kind'] == 'box':
            attrs['size'] = fmt(prim['size'])
        else:
            attrs['radius'] = f"{prim['radius']:.6g}"
            attrs['length'] = f"{prim['length']:.6g}"
        attrs['material'] = material
        rpy = fmt(matrix_rpy(prim['origin'][:3, :3]))
        if rpy != '0 0 0':
            attrs['rpy'] = rpy
        ET.SubElement(link, f"xacro:{prim['kind']}_shape", **attrs)


def write_urdf(path, W_new, joints, links, color_of, height, ratios, geometry=primitive_shapes,
               extra_macros=None, script='simplify_dy2026.py'):
    """geometry(link element, link name, links[name], material) adds a link's shapes;
    extra_macros(robot) adds properties / macros after the shared ones; script names the
    generator in the header comment."""
    robot = ET.Element('robot', {'name': 'dy2026_simple', 'xmlns:xacro': XACRO_NS})
    robot.append(ET.Comment(
        f' Generated by sim_description/tools/simplify_dy2026/{script} from the Onshape export\n'
        f'{INDENT}     (dy2026_masterassembly). Do not edit by hand; re-run the script. '))
    gap(robot)

    robot.append(ET.Comment(' Materials '))
    for cname, rgba in COLORS.items():
        m = ET.SubElement(robot, 'material', name=cname)
        ET.SubElement(m, 'color', rgba=fmt(rgba))
    gap(robot)

    add_properties(robot, joints, ratios)

    robot.append(ET.Comment(' Macros '))
    add_macros(robot)
    if extra_macros:
        extra_macros(robot)

    robot.append(ET.Comment(' Links, each followed by the joint attaching it to its parent '))
    ET.SubElement(robot, 'link', name='base_footprint')
    gap(robot)

    parent_joint = {j['child']: j for j in joints}
    for name, L in links.items():
        link = ET.SubElement(robot, 'link', name=name)
        if L['mass'] > 0:
            I = L['I']
            ET.SubElement(link, 'xacro:inertial', xyz=fmt(L['com']), mass=f"{L['mass']:.6g}",
                          ixx=f'{I[0, 0]:.6g}', ixy=f'{I[0, 1]:.6g}', ixz=f'{I[0, 2]:.6g}',
                          iyy=f'{I[1, 1]:.6g}', iyz=f'{I[1, 2]:.6g}', izz=f'{I[2, 2]:.6g}')
        geometry(link, name, L, color_of[name])

        if name == 'base_link':
            fj = ET.SubElement(robot, 'joint', name='base_footprint_joint', type='fixed')
            ET.SubElement(fj, 'origin', xyz=fmt([0, 0, height]), rpy='0 0 0')
            ET.SubElement(fj, 'parent', link='base_footprint')
            ET.SubElement(fj, 'child', link='base_link')
        else:
            j = parent_joint[name]
            T = np.linalg.inv(W_new[j['parent']]) @ W_new[j['child']]
            attrs = dict(name=j['name'], type=j['type'], parent=j['parent'], child=j['child'],
                         xyz=fmt(T[:3, 3]), rpy=fmt(matrix_rpy(T[:3, :3])))
            for key in JOINT_GROUP_DEFAULTS:
                attrs[key] = '${%s_%s}' % (j['group'], key)
            if j['type'] in ('revolute', 'prismatic'):
                attrs['lower'] = f"{j['lower']:.6g}"
                attrs['upper'] = f"{j['upper']:.6g}"
            if j['mimic']:
                attrs['mimic'] = j['mimic']
            ET.SubElement(robot, 'xacro:z_joint', **attrs)
        gap(robot)

    xml = format_xml(robot)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        f.write(xml)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--source', default=DEFAULT_SOURCE)
    parser.add_argument('--out', default=DEFAULT_OUT)
    args = parser.parse_args()

    src = Source(os.path.abspath(args.source))
    W_new, joints, links, color_of, height, ratios = build(src)
    write_urdf(os.path.abspath(args.out), W_new, joints, links, color_of, height, ratios)

    n_vis = sum(len(L['visuals']) for L in links.values())
    print(f'Wrote {os.path.abspath(args.out)}')
    print(f'  {len(links) + 1} links, {len(joints) + 1} joints, {n_vis} primitives '
          f'(from {len(src.links)} links, {len(src.joints)} joints)')
    for name, L in links.items():
        print(f'  {name:18s} {L["mass"]:7.3f} kg  {len(L["visuals"]):3d} shapes  '
              f'{len(L["parts"]):3d} source links')


if __name__ == '__main__':
    main()
