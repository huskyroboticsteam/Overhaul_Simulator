#!/usr/bin/env python3
"""Generate a simplified URDF of the dy2026 rover that keeps a few of the real CAD meshes.

Same model as simplify_dy2026.py (same links, joints, inertials, base_link /
base_footprint, xacro properties and differential ratios, so the two outputs are
interchangeable in dy2026.urdf.xacro), but instead of fitting boxes and cylinders it
uses the original Onshape meshes, keeping only the parts that make up the robot's outer
shape and contact surfaces. Each original part (one visual of the export)
is dropped when, in this order:

  * it matches FORCE_DROP (or is kept regardless if it matches FORCE_KEEP),
  * it is stray CAD in the root link, not attached to the chassis,
  * its file name says it is a fastener, bearing, drivetrain part (gears, belts, lead
    screws, gearbox cartridges, shafts) or electronics (PCBs, connectors, chips),
  * its largest side is under MIN_PART_SIZE, or it is a sheet thinner than
    MIN_THICKNESS (PCBs, shims),
  * less than VISIBLE_MIN of its surface can be seen from outside past the other
    remaining parts of its link (internals of housings, tubes, actuators).

Inertials still come from every part. Each kept mesh is written once, decimated, as a
binary STL visual and a coarser collision (its convex hull, or a decimated copy when it is
concave) into models/dy2026_simple/meshes; every reduction is checked against the original
(area, bounding box) and backed off when it fails. --no-decimate references the export's
glTF originals instead (RViz only: Gazebo Fortress can't load glTF). On the wheels the
meshes are visual only and the rolling contact is the same single cylinder about the axle
as in simplify_dy2026.py (meshes roll badly in Gazebo).

The kinematics (grouping, frames, joints, inertials, footprint, tyre cylinders) and the
URDF writer are simplify_dy2026.py's, imported from it; only the part selection, the
decimation and each link's geometry elements are this script's.

Usage (same venv as simplify_dy2026.py, plus fast-simplification: pip install -r
requirements.txt):
    python3 simplify_dy2026_meshes.py [--source <onshape model dir>] [--out <urdf path>]
        [--mesh-dir <dir>] [--no-decimate] [--report <text file of every part and its fate>]
"""

import argparse
import math
import os
import re
import sys
import xml.etree.ElementTree as ET
from collections import Counter

import fast_simplification
import numpy as np
import trimesh
from scipy.spatial import cKDTree

sys.dont_write_bytecode = True  # no __pycache__ next to the shared module
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import simplify_dy2026 as simple  # noqa: E402
from simplify_dy2026 import (  # noqa: E402
    Source, fmt, gap, kinematics, matrix_rpy, origin_matrix, write_urdf)

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_SOURCE = simple.DEFAULT_SOURCE
DEFAULT_OUT = os.path.join(HERE, '..', '..', 'models', 'dy2026_simple', 'urdf', 'dy2026_meshes.urdf.xacro')

# Where the meshes are at runtime. sim_description installs models/ whole, so both the
# decimated copies (models/dy2026_simple/meshes, next to the output's urdf/) and the
# export's originals (--no-decimate) resolve as package:// for RViz and, via
# IGN_GAZEBO_RESOURCE_PATH, as model:// for Gazebo. Gazebo Fortress loads STL / OBJ / COLLADA but not the export's glTF,
# hence binary STL for the copies. Directories and extension are xacro properties,
# overridable like the joint ones.
DECIMATED_URI = 'package://sim_description/models/dy2026_simple/meshes'
ORIGINAL_URI = 'package://sim_description/models/dy2026_masterassembly/meshes'
# Files the script wrote last time (relative to the mesh dir): only these get cleaned up
MANIFEST = 'GENERATED_FILES.txt'

# --- part selection --------------------------------------------------------------------
# Patterns are matched (case-insensitive, re.search) against the mesh file name, e.g.
# 'Socket_head_cap_screw__10_32_x_0_5.gltf'. Names are Onshape part names: McMaster
# numbers with descriptions, REV part numbers, or the team's A26_* names.

# Always kept / always dropped, whatever the rules below say. Tune here.
FORCE_KEEP = [
    r'^Printed_PCB_cover',  # rocker electronics housing lid, not a PCB
    r'^A26_H108_shaftCollar',  # 81 mm end blocks of the claw rails: the gripper's outer ends
]
FORCE_DROP = [
    r'^_8774N37_DC_Equipment_Cooling_Fan',  # 4 fans inside the chassis bay, 207k tris each
]

NAME_RULES = {
    'fastener': [
        r'screw(?!_?nut)', r'shcs|bhcs', r'(^|_)nut(s?_|\.)', r'lock_nut', r'washer',
        r'retaining_ring', r'heat_set_insert', r'standoff', r'(^|_)rivet',
        r'(^|_)pins?(_|\.|\d)', r'(^|_)spacer', r'Electrical_Box_Spacer',
    ],
    # bearing blocks / mounts are structure, not bearings
    'bearing': [r'bearing(?!_?block|_?mount)', r'bushing'],
    # power transmission, mostly inside the structure: gears, belts, lead screws, shafts,
    # UltraPlanetary gearbox stages (REV-41-1601/02/03 cartridges, 41-1615 output stage)
    'drivetrain': [
        r'gear', r'pinion', r'pulley', r'belt', r'chain', r'sprocket', r'lead_?screw',
        r'shaft', r'coupler', r'REV_41_16(01|02|03|15)', r'UltraPlanetary_(550_Motor_Plate|1_2in_Hex)',
        r'(^|_)hex_shaft',
    ],
    # PCBs, connectors, chips, logos
    'electronics': [
        r'pcb', r'micro_?fit|mirco_fit', r'usb', r'terminal', r'data_pins', r'interface__interface',
        r'qfn|ssop|sot\d|tvs|tsw_|t_1s6|sm03b|shdrra|fp_pcm|c_1825910|tda04|2n5639g|d0008a',
        r'logo|marking', r'xt30', r'limit_?switch', r'laser', r'UC_A53', r'Board_AK',
        r'(^|_)LED(_|\.)', r'^Open_CASCADE_STEP_translator\.', r'^m12_',
    ],
}
# Parts smaller than this (largest side of their own bounding box, metres) are dropped
MIN_PART_SIZE = 0.02
# ...and so are sheets thinner than this: the 1 mm boards in the rocker housings (a PCB)
# and a 1 mm shim. Not thicker: the gripper cover and claw plates are 1/16 in (1.59 mm).
MIN_THICKNESS = 0.0012
# A part is hidden when less than this fraction of its surface can be seen from outside
# past the other (not yet dropped) parts of its link
VISIBLE_MIN = 0.1
# Visibility grid (m), rays per surface sample, samples per part
VOXEL = 0.003
RAY_DIRECTIONS = 32
PROBE_SAMPLES = 200
# Surface samples per square metre when voxelizing (9 per voxel face: no gaps for rays)
DENSE_SAMPLES = 1e6

# --- decimation ------------------------------------------------------------------------
# Each kept source mesh is written once (instances share it) as a visual and, unless it is
# only used on a wheel, a collision mesh. A reduction is accepted only if the result keeps
# AREA_KEEP of the surface area, its bounding box within BOUNDS_TOL, and 99 % of the
# original surface within DEVIATION of it, beyond the sampling noise of that measurement
# (catches torn rims and slots that area and bounds miss); otherwise the face target is doubled until it does (a fixed face count silently
# deletes thin walls). The export's meshes are welded first: unwelded, quadric decimation
# tears them open.
VISUAL_FACES = 4000
VISUAL_AREA_KEEP = 0.95
VISUAL_BOUNDS_TOL = 0.001
VISUAL_DEVIATION = 0.001
# fast_simplification aggressiveness: 3 keeps area and bounds far better than the default
# 7 on these meshes, at the same speed
DECIMATION_AGG = 3
# Collision is either the part's convex hull (cheap, robust contact) or, for a concave
# part, a coarse decimation of it. A part is concave when more than CONCAVE_FRACTION of
# its hull surface is over CONCAVE_GAP away from the part (a C-bracket's or claw's open
# side, a U-channel); holes in plates and trusses are smaller than that, so they get hulls.
COLLISION_FACES = 400
COLLISION_AREA_KEEP = 0.85
COLLISION_BOUNDS_TOL = 0.003
COLLISION_DEVIATION = 0.004
HULL_FACES = 200
CONCAVE_GAP = 0.012
CONCAVE_FRACTION = 0.1
# Per-part overrides (first matching pattern on the mesh file name): visual_faces,
# collision_faces, collision='hull' | 'mesh'.
MESH_OVERRIDES = [
    (r'^A26_E10[39]_', dict(collision='hull')),          # arm truss tubes: holes, no pockets
    (r'^A26_H10[23]_(left|right)Claw', dict(collision='mesh')),  # things sit between the jaws
]


def name_rule(filename):
    """('force-keep' | 'force-drop' | NAME_RULES key | None) for a mesh file name."""
    for pattern in FORCE_KEEP:
        if re.search(pattern, filename, re.I):
            return 'force-keep'
    for pattern in FORCE_DROP:
        if re.search(pattern, filename, re.I):
            return 'force-drop'
    for rule, patterns in NAME_RULES.items():
        if any(re.search(p, filename, re.I) for p in patterns):
            return rule
    return None


class Meshes:
    """Per mesh file: triangle count, own-frame size, area, and surface samples (dense
    ones to voxelize it, a few to cast visibility rays from)."""

    def __init__(self, model_dir):
        self.dir = os.path.join(model_dir, 'meshes')
        self._info = {}

    def __call__(self, filename):
        name = os.path.basename(filename)
        if name not in self._info:
            scene = trimesh.load(os.path.join(self.dir, name), force='scene')
            mesh = scene.to_geometry() if hasattr(scene, 'to_geometry') else scene.dump(True)
            lo, hi = mesh.bounds
            n = int(min(max(mesh.area * DENSE_SAMPLES, 100), 500000))
            self._info[name] = dict(
                tris=len(mesh.faces), size=hi - lo, area=float(mesh.area),
                dense=np.asarray(trimesh.sample.sample_surface(mesh, n, seed=0)[0]),
                probe=np.asarray(trimesh.sample.sample_surface(mesh, PROBE_SAMPLES, seed=1)[0]))
        return self._info[name]


def source_parts(src, group, root_keep, meshes):
    """One dict per visual of the export: source link, group, world pose, mesh info."""
    parts = []
    for link_name, g in group.items():
        visuals = [v for v in src.links[link_name].findall('visual')
                   if v.find('geometry/mesh') is not None]
        for k, visual in enumerate(visuals):
            mesh = visual.find('geometry/mesh')
            info = meshes(mesh.get('filename'))
            parts.append(dict(
                link=link_name, group=g, file=os.path.basename(mesh.get('filename')),
                scale=mesh.get('scale') or '1 1 1', info=info,
                world=src.world[link_name] @ origin_matrix(visual.find('origin')),
                stray=link_name == src.root and not root_keep[k]))
    return parts


def to_world(part, points):
    T = part['world']
    return points @ T[:3, :3].T + T[:3, 3]


def fibonacci_directions(n):
    i = np.arange(n) + 0.5
    z = 1 - 2 * i / n
    t = math.pi * (1 + 5 ** 0.5) * i
    r = np.sqrt(1 - z ** 2)
    return np.column_stack([r * np.cos(t), r * np.sin(t), z])


def visibility(parts):
    """Fraction of each part's surface samples seen from outside past the other parts.

    The parts' surfaces are voxelized (each voxel remembers its part, or -2 if shared);
    rays from each part's samples march through the grid and a sample is visible when
    one leaves the grid without entering another part's voxel. Unlike a convex hull this
    keeps parts in the open side of a concave one (claw pads, hubs behind wheel spokes).
    """
    pts = [to_world(p, p['info']['dense']) for p in parts]
    lo = np.vstack(pts).min(0) - 2 * VOXEL
    dims = np.ceil((np.vstack(pts).max(0) + 2 * VOXEL - lo) / VOXEL).astype(int)
    owner = np.full(dims, -1, np.int32)
    for i, p in enumerate(pts):
        v = np.unique(((p - lo) / VOXEL).astype(int), axis=0)
        cur = owner[tuple(v.T)]
        owner[tuple(v.T)] = np.where(cur == -1, i, np.where(cur == i, i, -2))

    dirs = fibonacci_directions(RAY_DIRECTIONS)
    steps = int(np.linalg.norm(dims) * 2) + 1
    out = []
    for i, p in enumerate(parts):
        start = to_world(p, p['info']['probe'])
        # every (sample, direction) ray; positions in voxel units
        org = np.repeat((start - lo) / VOXEL, len(dirs), axis=0)
        d = np.tile(dirs, (len(start), 1)) * 0.5
        alive = np.arange(len(org))
        escaped = np.zeros(len(org), bool)
        for k in range(1, steps):
            v = (org[alive] + d[alive] * k).astype(int)
            outside = np.any((v < 0) | (v >= dims), axis=1)
            escaped[alive[outside]] = True
            inside = ~outside
            o = owner[tuple(v[inside].T)]
            blocked = (o != -1) & (o != i)
            alive = alive[inside][~blocked]
            if not len(alive):
                break
        out.append(escaped.reshape(len(start), len(dirs)).any(1).mean())
    return out


def select_parts(parts):
    """Set part['keep'] and part['reason'] for every part."""
    for p in parts:
        rule = name_rule(p['file'])
        p['keep'], p['reason'] = None, rule
        if rule == 'force-keep':
            p['keep'] = True
        elif rule == 'force-drop':
            p['keep'] = False
        elif p['stray']:
            p['keep'], p['reason'] = False, 'stray'
        elif rule:
            p['keep'] = False
        elif p['info']['size'].max() < MIN_PART_SIZE:
            p['keep'], p['reason'] = False, 'small'
        elif p['info']['size'].min() < MIN_THICKNESS:
            p['keep'], p['reason'] = False, 'thin'

    # Hidden parts: what can't be seen past the other candidate parts of the same link
    # (parts dropped above don't hide anything)
    by_group = {}
    for p in parts:
        if p['keep'] is not False:
            by_group.setdefault(p['group'], []).append(p)
    for candidates in by_group.values():
        for p, seen in zip(candidates, visibility(candidates)):
            p['visible'] = seen
            if p['keep'] is None:
                hidden = seen < VISIBLE_MIN
                p['keep'], p['reason'] = not hidden, 'hidden' if hidden else 'kept'


# --- decimated copies -----------------------------------------------------------------

def load_mesh(model_dir, filename):
    scene = trimesh.load(os.path.join(model_dir, 'meshes', filename), force='scene')
    return scene.to_geometry() if hasattr(scene, 'to_geometry') else scene.dump(True)


def surface_gap(mesh, dense):
    """99th percentile distance (m) from the original's surface samples `dense` to `mesh`,
    measured to samples of `mesh` (so it includes some sampling noise)."""
    samples, _ = trimesh.sample.sample_surface(mesh, min(4 * len(dense), 2000000), seed=3)
    return np.percentile(cKDTree(samples).query(dense)[0], 99)


def faithful(reduced, original, dense, noise, area_keep, tol, deviation):
    """Does the reduced mesh keep the original's surface area and bounding box, and come
    within `deviation` (+ the measurement's `noise`) of 99 % of its surface?"""
    ratio = reduced.area / original.area
    return (area_keep <= ratio <= 1 / area_keep
            and np.abs(reduced.bounds - original.bounds).max() <= tol
            and surface_gap(reduced, dense) <= noise + deviation)


def cluster(mesh, cell):
    """Vertex clustering: merge the vertices in each `cell`-sized grid cell (at their mean)
    and drop the faces that collapse. Coarser than quadric decimation, but works on the
    non-manifold meshes the export stitches from thousands of pieces, where quadric
    collapses stall."""
    key = np.floor((mesh.vertices - mesh.bounds[0]) / cell).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    inv = inv.ravel()
    v = np.zeros((inv.max() + 1, 3))
    np.add.at(v, inv, mesh.vertices)
    v /= np.bincount(inv)[:, None]
    f = inv[mesh.faces]
    f = f[(f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])]
    _, first = np.unique(np.sort(f, axis=1), axis=0, return_index=True)
    return trimesh.Trimesh(v, f[np.sort(first)])


def decimate(mesh, faces, area_keep, tol, deviation, dense=None):
    """(reduced mesh, backed off?). Per face target (`faces`, then doubling): quadric
    decimation, else vertex clustering on a grid fine enough for `deviation`; the first
    faithful result within 1.5x the target wins. The original if none is."""
    mesh = trimesh.Trimesh(mesh.vertices, mesh.faces)  # welds coincident vertices
    if dense is None:
        dense = trimesh.sample.sample_surface(
            mesh, int(min(max(mesh.area * DENSE_SAMPLES, 100), 500000)), seed=0)[0]
    if len(mesh.faces) <= faces:
        return mesh, False
    noise = surface_gap(mesh, dense)
    # a vertex moves at most a cell's half-diagonal; small parts get a finer grid, and a
    # half-size grid is the fallback
    cell = min(2 * deviation, mesh.extents.max() / 50)
    clustered = {}

    def clustered_ok(c):
        if c not in clustered:
            r = cluster(mesh, c)
            clustered[c] = r if faithful(r, mesh, dense, noise, area_keep, tol, deviation) else None
        return clustered[c]

    target = faces
    while target < len(mesh.faces):
        v, f = fast_simplification.simplify(mesh.vertices, mesh.faces, target_count=target,
                                            agg=DECIMATION_AGG)
        reduced = trimesh.Trimesh(v, f)
        if len(reduced.faces) <= 1.5 * target and \
                faithful(reduced, mesh, dense, noise, area_keep, tol, deviation):
            return reduced, target > faces
        for c in (cell, cell / 2):
            reduced = clustered_ok(c)
            if reduced is not None and len(reduced.faces) <= 1.5 * target:
                return reduced, target > faces
        target *= 2
    return mesh, True


def concave_fraction(mesh, dense):
    """Fraction of the convex hull's surface further than CONCAVE_GAP from the part
    (distance to the part's dense surface samples)."""
    samples, _ = trimesh.sample.sample_surface(mesh.convex_hull, 1000, seed=2)
    dist, _ = cKDTree(dense).query(samples)
    return float((dist > CONCAVE_GAP).mean())


def mesh_override(filename):
    for pattern, override in MESH_OVERRIDES:
        if re.search(pattern, filename, re.I):
            return override
    return {}


def decimate_parts(parts, model_dir, mesh_dir, wheels):
    """Write a visual and a collision STL per kept source mesh into mesh_dir/visual and
    mesh_dir/collision; returns {file: stats}. Removes files a previous run wrote that are
    no longer produced (listed in MANIFEST), nothing else."""
    used = {}
    for p in parts:
        if p['keep']:
            # collision only if some instance is off the wheels (wheels collide as cylinders)
            used[p['file']] = used.get(p['file'], False) or p['group'] not in wheels
    stats, written = {}, []
    for filename in sorted(used):
        mesh = load_mesh(model_dir, filename)
        info, override = parts_info(parts, filename), mesh_override(filename)
        stem = os.path.splitext(filename)[0]
        visual, v_back = decimate(mesh, override.get('visual_faces', VISUAL_FACES),
                                  VISUAL_AREA_KEEP, VISUAL_BOUNDS_TOL, VISUAL_DEVIATION,
                                  info['dense'])
        st = dict(tris=len(mesh.faces), visual=len(visual.faces), visual_backoff=v_back,
                  collision=0, kind='-', collision_backoff=False, concave=None)
        written.append(export(visual, mesh_dir, 'visual', stem))
        if used[filename]:
            st['concave'] = concave_fraction(mesh, info['dense'])
            kind = override.get('collision') or (
                'mesh' if st['concave'] > CONCAVE_FRACTION else 'hull')
            if kind == 'hull':
                hull = mesh.convex_hull
                coll, c_back = decimate(hull, HULL_FACES, COLLISION_AREA_KEEP,
                                        COLLISION_BOUNDS_TOL, COLLISION_DEVIATION)
            else:
                coll, c_back = decimate(mesh, override.get('collision_faces', COLLISION_FACES),
                                        COLLISION_AREA_KEEP, COLLISION_BOUNDS_TOL,
                                        COLLISION_DEVIATION, info['dense'])
            st.update(collision=len(coll.faces), kind=kind, collision_backoff=c_back)
            written.append(export(coll, mesh_dir, 'collision', stem))
        stats[filename] = st

    manifest = os.path.join(mesh_dir, MANIFEST)
    if os.path.exists(manifest):
        with open(manifest) as f:
            for rel in f.read().split():
                if rel not in written and os.path.isfile(os.path.join(mesh_dir, rel)):
                    os.remove(os.path.join(mesh_dir, rel))
    with open(manifest, 'w') as f:
        f.write('\n'.join(written) + '\n')
    return stats


def parts_info(parts, filename):
    return next(p['info'] for p in parts if p['file'] == filename)


def export(mesh, mesh_dir, kind, stem):
    """Write a binary STL (same bytes for the same mesh) and return its relative path."""
    rel = f'{kind}/{stem}.stl'
    os.makedirs(os.path.join(mesh_dir, kind), exist_ok=True)
    with open(os.path.join(mesh_dir, rel), 'wb') as f:
        f.write(trimesh.exchange.stl.export_stl(mesh))
    return rel


# --- output --------------------------------------------------------------------------

def add_mesh_macros(robot, decimated):
    """A kept part = its visual mesh plus its collision mesh (visual only on wheels, whose
    collision is one cylinder)."""
    if decimated:
        dirs = dict(visual=DECIMATED_URI + '/visual', collision=DECIMATED_URI + '/collision')
        ext = '.stl'
    else:
        dirs = dict(visual=ORIGINAL_URI, collision=ORIGINAL_URI)
        ext = '.gltf'
    for tag, uri in dirs.items():
        ET.SubElement(robot, 'xacro:property', name=f'dy2026_{tag}_dir', default=uri)
    ET.SubElement(robot, 'xacro:property', name='dy2026_mesh_ext', default=ext)
    for name, tags in (('mesh_shape', ('visual', 'collision')), ('mesh_visual', ('visual',))):
        macro = ET.SubElement(robot, 'xacro:macro', name=name,
                              params="file xyz material rpy:='0 0 0' scale:='1 1 1'")
        for tag in tags:
            el = ET.SubElement(macro, tag)
            ET.SubElement(el, 'origin', xyz='${xyz}', rpy='${rpy}')
            ET.SubElement(ET.SubElement(el, 'geometry'), 'mesh',
                          filename='${dy2026_%s_dir}/${file}${dy2026_mesh_ext}' % tag,
                          scale='${scale}')
            if tag == 'visual':
                ET.SubElement(el, 'material', name='${material}')
        gap(robot)
    macro = ET.SubElement(robot, 'xacro:macro', name='cylinder_collision',
                          params="xyz radius length rpy:='0 0 0'")
    el = ET.SubElement(macro, 'collision')
    ET.SubElement(el, 'origin', xyz='${xyz}', rpy='${rpy}')
    ET.SubElement(ET.SubElement(el, 'geometry'), 'cylinder', radius='${radius}', length='${length}')
    gap(robot)


def mesh_geometry(W_new, parts, wheels):
    """write_urdf geometry callback: a link's kept parts as mesh_shape calls (mesh_visual
    on wheels, plus their collision cylinder), largest first."""
    kept = {}
    for p in parts:
        if p['keep']:
            kept.setdefault(p['group'], []).append(p)

    def geometry(link, name, L, material):
        inv = np.linalg.inv(W_new[name])
        macro = 'xacro:mesh_visual' if name in wheels else 'xacro:mesh_shape'
        for p in sorted(kept.get(name, []), key=lambda p: -p['info']['area']):
            # pose in the merged link = inverse(new link frame) x source link x visual origin
            T = inv @ p['world']
            attrs = dict(file=os.path.splitext(p['file'])[0], xyz=fmt(T[:3, 3]),
                         material=material)
            rpy = fmt(matrix_rpy(T[:3, :3]))
            if rpy != '0 0 0':
                attrs['rpy'] = rpy
            if p['scale'] != '1 1 1':
                attrs['scale'] = p['scale']
            ET.SubElement(link, macro, **attrs)
        if name in wheels:
            c = wheels[name]
            attrs = dict(xyz=fmt(c['origin'][:3, 3]), radius=f"{c['radius']:.6g}",
                         length=f"{c['length']:.6g}")
            rpy = fmt(matrix_rpy(c['origin'][:3, :3]))
            if rpy != '0 0 0':
                attrs['rpy'] = rpy
            ET.SubElement(link, 'xacro:cylinder_collision', **attrs)

    return geometry


def write_report(path, parts, stats):
    """Every source part with its fate, grouped by new link (tab-separated); kept parts
    also get their visual / collision triangle counts."""
    with open(path, 'w') as f:
        f.write('link\tkept\treason\tvisible\tsize_mm\ttris\tvisual_tris\tcollision\t'
                'source_link\tmesh\n')
        for p in sorted(parts, key=lambda p: (p['group'], not p['keep'], p['reason'] or '',
                                              -p['info']['area'])):
            size = 'x'.join(f'{v * 1000:.0f}' for v in sorted(p['info']['size'], reverse=True))
            seen = f"{p['visible']:.2f}" if 'visible' in p else '-'
            st = stats.get(p['file']) if p['keep'] else None
            dec = (f"{st['visual']}{'*' if st['visual_backoff'] else ''}\t"
                   f"{st['kind']} {st['collision']}{'*' if st['collision_backoff'] else ''}"
                   if st else '-\t-')
            f.write(f"{p['group']}\t{'yes' if p['keep'] else 'no'}\t{p['reason']}\t{seen}\t{size}\t"
                    f"{p['info']['tris']}\t{dec}\t{p['link']}\t{p['file']}\n")


def summary(links, parts, wheels, stats):
    rules = ['stray', 'force-drop', *NAME_RULES, 'small', 'thin', 'hidden']
    print(f'  {"link":18s} {"parts":>5s} {"kept":>4s} {"tris kept":>9s}  '
          + ' '.join(f'{r[:6]:>6s}' for r in rules))
    total = Counter()
    for name in links:
        mine = [p for p in parts if p['group'] == name]
        dropped = Counter(p['reason'] for p in mine if not p['keep'])
        kept = [p for p in mine if p['keep']]
        tris = sum(p['info']['tris'] for p in kept)
        total.update(dropped)
        total.update(parts=len(mine), kept=len(kept), tris=tris)
        print(f'  {name:18s} {len(mine):5d} {len(kept):4d} {tris:9d}  '
              + ' '.join(f'{dropped[r]:6d}' for r in rules)
              + ('  (+ cylinder collision)' if name in wheels else ''))
    print(f'  {"total":18s} {total["parts"]:5d} {total["kept"]:4d} {total["tris"]:9d}  '
          + ' '.join(f'{total[r]:6d}' for r in rules))
    print(f'  ({sum(p["info"]["tris"] for p in parts)} triangles in the full export)')
    if not stats:
        return
    # triangles drawn / collided per link, every instance counted (meshes are shared)
    print(f'\n  {"link":18s} {"kept tris":>9s} {"visual":>7s} {"collision":>9s}')
    total = Counter()
    for name in links:
        kept = [p for p in parts if p['group'] == name and p['keep']]
        row = Counter(kept=sum(p['info']['tris'] for p in kept),
                      visual=sum(stats[p['file']]['visual'] for p in kept),
                      collision=0 if name in wheels else
                      sum(stats[p['file']]['collision'] for p in kept))
        total.update(row)
        print(f'  {name:18s} {row["kept"]:9d} {row["visual"]:7d} {row["collision"]:9d}'
              + ('  (+ cylinder)' if name in wheels else ''))
    print(f'  {"total":18s} {total["kept"]:9d} {total["visual"]:7d} {total["collision"]:9d}')
    files = stats.values()
    print(f'  {len(stats)} mesh files ({sum(s["kind"] == "hull" for s in files)} hull, '
          f'{sum(s["kind"] == "mesh" for s in files)} decimated-mesh collisions); backed off: '
          f'{sum(s["visual_backoff"] for s in files)} visual, '
          f'{sum(s["collision_backoff"] for s in files)} collision')


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--source', default=DEFAULT_SOURCE)
    parser.add_argument('--out', default=DEFAULT_OUT)
    parser.add_argument('--report', help='write every part with kept/dropped and why')
    parser.add_argument('--mesh-dir', help='where to write the decimated meshes '
                        '(default: meshes/ next to the urdf/ directory of --out)')
    parser.add_argument('--no-decimate', action='store_true',
                        help="reference the export's original glTF meshes instead")
    args = parser.parse_args()
    out = os.path.abspath(args.out)
    mesh_dir = os.path.abspath(args.mesh_dir or os.path.join(os.path.dirname(out), '..', 'meshes'))

    src = Source(os.path.abspath(args.source))
    kin = kinematics(src)
    links, wheels = kin['links'], kin['wheels']
    parts = source_parts(src, kin['group'], kin['root_keep'], Meshes(src.model_dir))
    select_parts(parts)
    stats = {} if args.no_decimate else decimate_parts(parts, src.model_dir, mesh_dir, wheels)
    write_urdf(out, kin['W_new'], kin['joints'], links, kin['color_of'], kin['height'],
               kin['ratios'], geometry=mesh_geometry(kin['W_new'], parts, wheels),
               extra_macros=lambda robot: add_mesh_macros(robot, not args.no_decimate),
               script=os.path.basename(__file__))
    if args.report:
        write_report(os.path.abspath(args.report), parts, stats)

    print(f'Wrote {out}' + ('' if args.no_decimate else f' and meshes in {mesh_dir}'))
    print(f'  {len(links) + 1} links, {len(kin["joints"]) + 1} joints, '
          f'{sum(p["keep"] for p in parts)} of {len(parts)} CAD parts kept')
    summary(links, parts, wheels, stats)


if __name__ == '__main__':
    main()
