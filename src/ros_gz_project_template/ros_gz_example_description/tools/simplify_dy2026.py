#!/usr/bin/env python3
"""Generate a heavily simplified URDF of the dy2026 rover from the Onshape export.

The Onshape export has 147 links / 146 joints, most of which are artifacts of how
Onshape decomposes mates (ball, cylindrical, planar and parallel mates become chains
of dummy links with +-10000 limits, plus *_loop_closure links). This script:

  * keeps only the real degrees of freedom (listed in KEEP below),
  * merges every other link rigidly into its nearest kept ancestor (or an explicit
    group in ASSIGN), dropping massless dummy / loop-closure links,
  * replaces the meshes with one box or cylinder per original part, fitted to the
    part's bounding box. Parts smaller than MIN_PART_SIZE keep their mass but get
    no geometry, shapes fully inside a bigger box are dropped, and each link keeps
    only its largest few shapes,
  * ignores stray CAD in the root link (loose PCBs etc. placed around the assembly)
    by keeping only the parts connected to the main chassis,
  * combines masses, centres of mass and inertia tensors of merged parts,
  * re-expresses everything in a ROS-style base_link (X forward, Z up) instead of
    Onshape's tilted Y-up world, with a base_footprint on the ground below it.

Joint zero positions match the original export, so joint values carry over.

Usage (needs numpy and trimesh):
    python3 simplify_dy2026.py [--source <onshape model dir>] [--out <urdf path>]
"""

import argparse
import math
import os
import xml.etree.ElementTree as ET
from xml.dom import minidom

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_SOURCE = os.path.join(HERE, '..', 'models', 'dy2026_masterassembly')
DEFAULT_OUT = os.path.join(HERE, '..', 'models', 'dy2026_simple', 'urdf', 'dy2026_simple.urdf')

# Parts whose bounding box is smaller than this (largest side, metres) get no geometry
MIN_PART_SIZE = 0.07
# Only the largest shapes of each link are kept (by volume)
MAX_SHAPES_PER_LINK = 10
MAX_SHAPES_BASE = 16
# Root-link parts count as touching when their bounding boxes are within this distance
TOUCH_TOLERANCE = 0.005

# --- simplified kinematic tree -------------------------------------------------------
# new link: (parent new link, joint name, joint type, source joint, original head link,
#            colour). The new joint takes its frame/axis/limits from the source joint.
# 'axis_from' / 'limits' / 'flip' override the source for mates Onshape split up.
KEEP = {
    'differential_bar': dict(
        parent='base_link', joint='differential_joint', src='revolute_2',
        head='60355k291_steel_ball_bearing_25', color='steel'),
    'rocker_left': dict(
        parent='base_link', joint='rocker_left_joint', src='revolute_2_3',
        head='fillet8_1', color='frame'),
    'rocker_right': dict(
        parent='base_link', joint='rocker_right_joint', src='revolute_1_17',
        head='fillet8_2', color='frame'),
    'steer_front_left': dict(
        parent='rocker_left', joint='steer_front_left_joint', src='revolute_1_4',
        head='ttb_0115__1__12', color='module'),
    'steer_rear_left': dict(
        parent='rocker_left', joint='steer_rear_left_joint', src='revolute_1_8',
        head='ttb_0115__1_', color='module'),
    'steer_rear_right': dict(
        parent='rocker_right', joint='steer_rear_right_joint', src='revolute_1_12',
        head='ttb_0115__1__24', color='module'),
    'steer_front_right': dict(
        parent='rocker_right', joint='steer_front_right_joint', src='revolute_1_16',
        head='ttb_0115__1__36', color='module'),
    'wheel_front_left': dict(
        parent='steer_front_left', joint='wheel_front_left_joint', src='revolute_2_1',
        head='solid16_2', color='tire', wheel=True),
    'wheel_rear_left': dict(
        parent='steer_rear_left', joint='wheel_rear_left_joint', src='revolute_2_2',
        head='solid16', color='tire', wheel=True),
    'wheel_rear_right': dict(
        parent='steer_rear_right', joint='wheel_rear_right_joint', src='revolute_2_4',
        head='solid16_5', color='tire', wheel=True),
    'wheel_front_right': dict(
        parent='steer_front_right', joint='wheel_front_right_joint', src='revolute_2_5',
        head='solid16_6', color='tire', wheel=True),
    'mast_lower': dict(
        parent='base_link', joint='mast_lower_joint', src='slider_1',
        head='part_1_12', color='accent'),
    'mast_upper': dict(
        parent='mast_lower', joint='mast_upper_joint', src='slider_2',
        head='part_1_9', color='accent'),
    # The carriage rides the lead screw; its travel comes from the nut's loop-closure mate
    'arm_carriage': dict(
        parent='base_link', joint='arm_rail_joint', src='cylindrical_1_loop_closure_4',
        head='a26_s102_railcrossbar_1', color='arm', flip=True),
    'arm_upper': dict(
        parent='arm_carriage', joint='arm_shoulder_joint', src='revolute_1_22',
        head='a26_s107_shaft', color='arm'),
    'arm_forearm': dict(
        parent='arm_upper', joint='arm_elbow_joint', src='planar_1_5',
        head='a26_e103_forearmtube', color='arm'),
    'arm_wrist': dict(
        parent='arm_forearm', joint='arm_wrist_pitch_joint', src='revolute_1_19',
        head='a26_e107_driveshaftoutput', color='arm'),
    'gripper': dict(
        parent='arm_wrist', joint='arm_wrist_roll_joint', src='revolute_5_1',
        head='a26_w110_crowngear', color='gripper'),
    # Claws slide on the gripper's linear rods (positive = open); the right claw mirrors the left
    'claw_left': dict(
        parent='gripper', joint='claw_left_joint', src='cylindrical_1',
        head='a_h_2_left_claw', color='claw', type='prismatic'),
    'claw_right': dict(
        parent='gripper', joint='claw_right_joint', src='cylindrical_1',
        head='a_h_3_right_claw', color='claw', type='prismatic', mimic='claw_left_joint'),
}

# Original links that belong to a group other than their nearest kept ancestor
ASSIGN = {
    # gripper body hangs off the crown gear in reality (Onshape routed it elsewhere)
    'a_h_9_double_sided_flat_shaft_collar': 'gripper',
    'a_h_9_double_sided_flat_shaft_collar_1': 'gripper',
    'a_h_12_linear_rod': 'gripper',
    'a_h_12_linear_rod_1': 'gripper',
    'a_h_1_body': 'gripper',
    '93365a132_brass_tapered_heat_set_inserts_for_plastic_1': 'gripper',
    'roller_lever_limit_switch___3103_0001_0001': 'gripper',
    'a_h_4_motor_gear': 'gripper',
    'a_h_5_lead_screw_gear': 'gripper',
    'a_h_5_lead_screw_gear_1': 'gripper',
    # claw guide bushings move with the claws
    '6338k416_oil_embedded_flanged_sleeve_bearing__1__1': 'claw_left',
    '6338k416_oil_embedded_flanged_sleeve_bearing__1__2': 'claw_left',
    '6338k416_oil_embedded_flanged_sleeve_bearing__1_': 'claw_right',
    '6338k416_oil_embedded_flanged_sleeve_bearing__1__3': 'claw_right',
    # the lead-screw nut moves with the arm carriage
    'a26_s105_nutmount': 'arm_carriage',
}

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
        root = ET.parse(os.path.join(model_dir, 'urdf', name + '.urdf')).getroot()
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

    def mesh_vertices(self, filename):
        path = os.path.join(self.model_dir, 'meshes', os.path.basename(filename))
        if path not in self._meshes:
            scene = trimesh.load(path, force='scene')
            geom = scene.to_geometry() if hasattr(scene, 'to_geometry') else scene.dump(True)
            v = np.asarray(geom.vertices)
            if len(v) > 20000:  # bounding boxes don't need every vertex
                v = v[:: len(v) // 20000]
            self._meshes[path] = v
        return self._meshes[path]

    def visual_points(self, link_name):
        """World-space vertices of each visual of a link: list of arrays."""
        out = []
        Wl = self.world[link_name]
        for visual in self.links[link_name].findall('visual'):
            mesh = visual.find('geometry/mesh')
            if mesh is None:
                continue
            v = self.mesh_vertices(mesh.get('filename'))
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

def fit_primitive(points, T_link_world, force_cylinder_axis=None):
    """Fit a box (or cylinder) to world points, expressed in the given link frame.

    Returns dict(kind, origin(4x4 in link frame), size) or None if too small.
    """
    local = (T_link_world[:3, :3] @ points.T).T + T_link_world[:3, 3]

    if force_cylinder_axis is not None:
        T = frame_from_z(force_cylinder_axis, np.zeros(3))
        p = local @ T[:3, :3]
        lo, hi = p.min(0), p.max(0)
        centre = T[:3, :3] @ ((lo + hi) / 2)
        T[:3, 3] = centre
        radius = max(hi[0] - lo[0], hi[1] - lo[1]) / 2
        return dict(kind='cylinder', origin=T, radius=radius, length=hi[2] - lo[2])

    # Axis-aligned box in the link frame vs. a minimum-volume oriented box: keep the
    # tighter one (CAD parts are usually aligned with something, but not always)
    lo, hi = local.min(0), local.max(0)
    aabb_ext = hi - lo
    aabb_T = np.eye(4)
    aabb_T[:3, 3] = (lo + hi) / 2
    try:
        obb_T, obb_ext = trimesh.bounds.oriented_bounds(local)
        obb_T = np.linalg.inv(obb_T)
    except Exception:  # degenerate point sets
        obb_T, obb_ext = aabb_T, aabb_ext
    if np.prod(obb_ext + 1e-3) < 0.8 * np.prod(aabb_ext + 1e-3):
        T, ext = obb_T, np.asarray(obb_ext)
    else:
        T, ext = aabb_T, aabb_ext

    if ext.max() < MIN_PART_SIZE:
        return None

    # Long round-ish parts (tubes, shafts, rods, lead screws) read better as cylinders
    order = np.argsort(ext)
    a, b, c = ext[order]
    if c > 2.5 * b and b > 0 and a / b > 0.8:
        axis = T[:3, :3][:, order[2]]
        Tc = frame_from_z(axis, T[:3, 3])
        return dict(kind='cylinder', origin=Tc, radius=(a + b) / 4, length=c)
    return dict(kind='box', origin=T, size=ext)


# --- main ----------------------------------------------------------------------------

def build(src):
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
    _, up = src.joint_frame('revolute_2')
    up = -up / np.linalg.norm(up)
    _, lateral = src.joint_frame('revolute_2_3')
    lateral -= up * np.dot(lateral, up)
    lateral /= np.linalg.norm(lateral)
    forward = np.cross(lateral, up)
    # The arm reaches forward
    chassis_pts = np.vstack([p for p, keep in zip(root_parts, root_keep) if keep])
    centre = (chassis_pts.min(0) + chassis_pts.max(0)) / 2
    if np.dot(src.world['a_h_1_body'][:3, 3] - centre, forward) < 0:
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
        # Frame: joint position of the source mate, Z along the joint axis
        frame = frame_from_z(axis, Tj[:3, 3], hint=R_base[:, 0])
        if name.startswith('claw_'):
            # put each claw's frame at its own part, sliding along the shared rod axis
            frame[:3, 3] = src.world[spec['head']][:3, 3]
        W_new[name] = frame
        joints.append(dict(name=spec['joint'], type=jtype, parent=spec['parent'], child=name,
                           lower=lower, upper=upper, mimic=spec.get('mimic')))

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

    # Visual primitives and combined inertia per new link
    links = {name: dict(visuals=[], mass=0.0, com=np.zeros(3), I=np.zeros((3, 3)), parts=[])
             for name in W_new}
    color_of = {name: spec['color'] for name, spec in KEEP.items()}
    color_of['base_link'] = 'chassis'

    for link_name, g in group.items():
        L = links[g]
        L['parts'].append(link_name)
        inv = np.linalg.inv(W_new[g])
        inertial = src.inertial(link_name)
        if inertial is not None:
            L.setdefault('_inertials', []).append(inertial)

        visual_sets = src.visual_points(link_name)
        if not visual_sets:
            continue
        if link_name == src.root:
            visual_sets = [p for p, keep in zip(visual_sets, root_keep) if keep]
        spec = KEEP.get(g, {})
        if spec.get('wheel') and link_name != spec['head'] and \
                src.inertial(link_name) and src.inertial(link_name)[0] > 1.0:
            # the tyre: one cylinder about the wheel axis
            L['visuals'].append(
                fit_primitive(np.vstack(visual_sets), inv, np.array([0, 0, 1.0])))
            continue
        # one primitive per part
        for pts in visual_sets:
            prim = fit_primitive(pts, inv)
            if prim:
                L['visuals'].append(prim)

    # Drop boxes fully inside other boxes of the same link (lots of chassis internals)
    for name, L in links.items():
        cap = MAX_SHAPES_BASE if name == 'base_link' else MAX_SHAPES_PER_LINK
        L['visuals'] = prune_contained(L['visuals'])[:cap]  # sorted largest first

    for name, L in links.items():
        inertials = L.pop('_inertials', [])
        if not inertials:
            continue
        inv = np.linalg.inv(W_new[name])
        R = inv[:3, :3]
        m = sum(i[0] for i in inertials)
        com_w = sum(i[0] * i[1] for i in inertials) / m
        I = np.zeros((3, 3))
        for mi, ci, Ii in inertials:
            d = ci - com_w
            I += Ii + mi * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
        L['mass'] = m
        L['com'] = R @ com_w + inv[:3, 3]
        L['I'] = R @ I @ R.T

    # base_footprint: on the ground (lowest wheel point) straight below base_link
    wheel_pts = np.vstack([p for n, g in group.items() if KEEP.get(g, {}).get('wheel')
                           for p in src.visual_points(n)])
    height = np.dot(base_origin - wheel_pts[np.argmin(wheel_pts @ up)], up)

    return W_new, joints, links, color_of, height


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


def prune_contained(prims):
    def corners(p):
        if p['kind'] == 'box':
            e = p['size'] / 2
        else:
            e = np.array([p['radius'], p['radius'], p['length'] / 2])
        c = np.array([[sx * e[0], sy * e[1], sz * e[2], 1]
                      for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
        return (p['origin'] @ c.T).T[:, :3]

    def inside(points, box):
        inv = np.linalg.inv(box['origin'])
        local = (inv[:3, :3] @ points.T).T + inv[:3, 3]
        return np.all(np.abs(local) <= box['size'] / 2 + 1e-3)

    keep = []
    boxes = sorted([p for p in prims if p['kind'] == 'box'], key=lambda p: -np.prod(p['size']))
    for p in sorted(prims, key=lambda p: -volume(p)):
        if not any(q is not p and inside(corners(p), q) for q in boxes if volume(q) > volume(p)):
            keep.append(p)
    return keep


def volume(p):
    if p['kind'] == 'box':
        return float(np.prod(p['size']))
    return math.pi * p['radius'] ** 2 * p['length']


def write_urdf(path, W_new, joints, links, color_of, height):
    robot = ET.Element('robot', name='dy2026_simple')
    robot.append(ET.Comment(
        ' Generated by ros_gz_example_description/tools/simplify_dy2026.py from the Onshape '
        'export (dy2026_masterassembly). Do not edit by hand; re-run the script. '))
    for cname, rgba in COLORS.items():
        m = ET.SubElement(robot, 'material', name=cname)
        ET.SubElement(m, 'color', rgba=fmt(rgba))

    ET.SubElement(robot, 'link', name='base_footprint')
    fj = ET.SubElement(robot, 'joint', name='base_footprint_joint', type='fixed')
    ET.SubElement(fj, 'origin', xyz=fmt([0, 0, height]), rpy='0 0 0')
    ET.SubElement(fj, 'parent', link='base_footprint')
    ET.SubElement(fj, 'child', link='base_link')

    for name, L in links.items():
        link = ET.SubElement(robot, 'link', name=name)
        if L['mass'] > 0:
            inertial = ET.SubElement(link, 'inertial')
            ET.SubElement(inertial, 'origin', xyz=fmt(L['com']), rpy='0 0 0')
            ET.SubElement(inertial, 'mass', value=f"{L['mass']:.6g}")
            I = L['I']
            ET.SubElement(inertial, 'inertia', ixx=f'{I[0, 0]:.6g}', ixy=f'{I[0, 1]:.6g}',
                          ixz=f'{I[0, 2]:.6g}', iyy=f'{I[1, 1]:.6g}', iyz=f'{I[1, 2]:.6g}',
                          izz=f'{I[2, 2]:.6g}')
        for prim in L['visuals']:
            for tag in ('visual', 'collision'):
                el = ET.SubElement(link, tag)
                ET.SubElement(el, 'origin', xyz=fmt(prim['origin'][:3, 3]),
                              rpy=fmt(matrix_rpy(prim['origin'][:3, :3])))
                geom = ET.SubElement(el, 'geometry')
                if prim['kind'] == 'box':
                    ET.SubElement(geom, 'box', size=fmt(prim['size']))
                else:
                    ET.SubElement(geom, 'cylinder', radius=f"{prim['radius']:.6g}",
                                  length=f"{prim['length']:.6g}")
                if tag == 'visual':
                    ET.SubElement(el, 'material', name=color_of[name])

    for j in joints:
        T = np.linalg.inv(W_new[j['parent']]) @ W_new[j['child']]
        el = ET.SubElement(robot, 'joint', name=j['name'], type=j['type'])
        ET.SubElement(el, 'origin', xyz=fmt(T[:3, 3]), rpy=fmt(matrix_rpy(T[:3, :3])))
        ET.SubElement(el, 'parent', link=j['parent'])
        ET.SubElement(el, 'child', link=j['child'])
        ET.SubElement(el, 'axis', xyz='0 0 1')
        if j['type'] in ('revolute', 'prismatic'):
            ET.SubElement(el, 'limit', lower=f"{j['lower']:.6g}", upper=f"{j['upper']:.6g}",
                          effort='100', velocity='1')
        if j['mimic']:
            ET.SubElement(el, 'mimic', joint=j['mimic'], multiplier='1', offset='0')

    xml = minidom.parseString(ET.tostring(robot)).toprettyxml(indent='  ')
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        f.write(xml)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--source', default=DEFAULT_SOURCE)
    parser.add_argument('--out', default=DEFAULT_OUT)
    args = parser.parse_args()

    src = Source(os.path.abspath(args.source))
    W_new, joints, links, color_of, height = build(src)
    write_urdf(os.path.abspath(args.out), W_new, joints, links, color_of, height)

    n_vis = sum(len(L['visuals']) for L in links.values())
    print(f'Wrote {os.path.abspath(args.out)}')
    print(f'  {len(links) + 1} links, {len(joints) + 1} joints, {n_vis} primitives '
          f'(from {len(src.links)} links, {len(src.joints)} joints)')
    for name, L in links.items():
        print(f'  {name:18s} {L["mass"]:7.3f} kg  {len(L["visuals"]):3d} shapes  '
              f'{len(L["parts"]):3d} source links')


if __name__ == '__main__':
    main()
