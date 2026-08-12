#!/usr/bin/env python3
"""Plan and check walking paths for the hospital world's actors.

Actors follow a scripted trajectory and perceive nothing, so a path that clips a
chair is walked through rather than avoided. Every path therefore has to be
proved clear before it ships, and that is what this does. It has caught, in
order: a walker crossing a sofa at 0.24 m, another crossing a standing figure at
0.50 m, and a wheelchair routed through a chair at 0.70 m - each time on a path
that had been added without being run past it.

Obstacles come from three places, and leaving any one out is how each of those
bugs got in:
  * WALLS, rasterised from the building's collision mesh. The mesh is in
    centimetres (<unit meter="0.01">) and its pieces carry per-node transforms;
    both must be applied or the floorplan does not line up with the world.
  * FURNITURE, as a measured RECTANGLE per <include>, not as its origin. This
    matters more than it sounds: the reception desk is 4.84 x 3.02 m, so a path
    2 m from its origin runs straight through it. A point-based version of this
    check passed three walkers that were crossing the reception desk.
  * PEOPLE, both the standing/seated actors and the paths of the moving ones -
    a corridor that is clear of furniture is not clear if someone else is
    already walking down it.

Run it after touching any trajectory:

    ros2 run pickplace_arm_bringup hospital_actor_paths          # check
    ros2 run pickplace_arm_bringup hospital_actor_paths --plan   # suggest paths
"""
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np
from ament_index_python.packages import get_package_share_directory

NS = '{http://www.collada.org/2005/11/COLLADASchema}'
RES = 0.10
Z_LO, Z_HI = 0.05, 1.20
# A person needs less room than the robot; this is shoulder width plus margin.
WALL_CLEAR = 0.55
# Measured from the object EDGE, not its origin, so this is body half width
# plus margin rather than a guess at how big things are.
OBJ_CLEAR = 0.55
PATH_CLEAR = 1.00
# A TURN POINT is where an actor stands still for two seconds before reversing,
# so it needs more room than somewhere it merely walks past: at 0.7 m from a
# trolley a stationary figure reads as standing inside it, which is exactly what
# was happening - the walkers appeared to enter an obstacle, pause, then come
# back out. The extra margin is only wanted from OBJECTS; 0.7 m from a wall
# looks perfectly normal, and demanding more makes the narrow corridors
# unusable.
TURN_OBJ_CLEAR = 1.20
TURN_WALL_CLEAR = 0.70


def _desc_share():
    return get_package_share_directory('pickplace_arm_description')


def wall_grid(which='visual'):
    """Occupancy of the building's walls at person height, plus its origin.

    VISUAL by default, not collision. The two differ: the collision mesh leaves
    the door openings clear so a robot can drive through, while the visual mesh
    carries the door panels and glazing. Checking against collision therefore
    passes paths that walk straight through a door you can see - which is what
    happened, and why the actors appeared to walk into walls while this said
    every path had 0.7 m of clearance."""
    name = ('aws_robomaker_hospital_floor_01_walls_collision.dae'
            if which == 'collision'
            else 'aws_robomaker_hospital_floor_01_walls_visual.dae')
    dae = os.path.join(
        _desc_share(), 'aws_hospital_models',
        'aws_robomaker_hospital_floor_01_walls', 'meshes', name)
    root = ET.parse(dae).getroot()

    src = {'#' + s.get('id'): s for s in root.iter(f'{NS}source') if s.get('id')}
    xf = {}
    for node in root.iter(f'{NS}node'):
        inst = node.find(f'{NS}instance_geometry')
        if inst is None:
            continue
        M = np.eye(4)
        for child in node:
            tag = child.tag.split('}')[-1]
            v = np.fromstring(child.text, sep=' ') if child.text else None
            if tag == 'translate':
                T = np.eye(4); T[:3, 3] = v[:3]; M = M @ T
            elif tag == 'rotate' and abs(v[3]) > 1e-9:
                ax = v[:3] / (np.linalg.norm(v[:3]) or 1.0)
                a = np.deg2rad(v[3]); c, s_, C = np.cos(a), np.sin(a), 1 - np.cos(a)
                x, y, z = ax
                R = np.array([[x*x*C+c, x*y*C-z*s_, x*z*C+y*s_, 0],
                              [y*x*C+z*s_, y*y*C+c, y*z*C-x*s_, 0],
                              [z*x*C-y*s_, z*y*C+x*s_, z*z*C+c, 0],
                              [0, 0, 0, 1]])
                M = M @ R
            elif tag == 'scale':
                M = M @ np.diag([v[0], v[1], v[2], 1.0])
        xf['#' + inst.get('url').lstrip('#')] = M

    unit = 1.0
    asset = root.find(f'{NS}asset')
    if asset is not None:
        u = asset.find(f'{NS}unit')
        if u is not None and u.get('meter'):
            unit = float(u.get('meter'))

    tris = []
    for geom in root.iter(f'{NS}geometry'):
        M = xf.get('#' + geom.get('id'), np.eye(4))
        mesh = geom.find(f'{NS}mesh')
        if mesh is None:
            continue
        vel = mesh.find(f'{NS}vertices')
        vid = '#' + vel.get('id') if vel is not None else None
        vsrc = vel.find(f'{NS}input').get('source') if vel is not None else None
        for prim in list(mesh.iter(f'{NS}polylist')) + list(mesh.iter(f'{NS}triangles')):
            inputs = prim.findall(f'{NS}input')
            stride = max(int(i.get('offset', 0)) for i in inputs) + 1
            off = psrc = None
            for i in inputs:
                if i.get('semantic') == 'VERTEX':
                    off = int(i.get('offset', 0))
                    psrc = vsrc if i.get('source') == vid else i.get('source')
            if psrc not in src:
                continue
            pel = prim.find(f'{NS}p')
            if pel is None:
                continue
            pts = np.fromstring(src[psrc].find(f'{NS}float_array').text,
                                sep=' ').reshape(-1, 3)
            pts = ((M[:3, :3] @ pts.T).T + M[:3, 3]) * unit
            idx = np.fromstring(pel.text, sep=' ').astype(np.int64)
            idx = idx.reshape(-1, stride)[:, off]
            vc = prim.find(f'{NS}vcount')
            counts = (np.fromstring(vc.text, sep=' ').astype(int)
                      if vc is not None else np.full(len(idx) // 3, 3))
            k, fan = 0, []
            for c in counts:
                face = idx[k:k + c]
                fan += [(face[0], face[j], face[j + 1]) for j in range(1, c - 1)]
                k += c
            if fan:
                tris.append(pts[np.array(fan)])
    tris = np.concatenate(tris)
    zmin, zmax = tris[:, :, 2].min(axis=1), tris[:, :, 2].max(axis=1)
    tris = tris[(zmax >= Z_LO) & (zmin <= Z_HI)]

    xy = tris[:, :, :2]
    lo = xy.reshape(-1, 2).min(axis=0) - 1.0
    hi = xy.reshape(-1, 2).max(axis=0) + 1.0
    w = int((hi[0] - lo[0]) / RES) + 1
    h = int((hi[1] - lo[1]) / RES) + 1
    occ = np.zeros((h, w), bool)
    steps = np.linspace(0, 1, 64).reshape(1, -1, 1)
    for a, b in ((0, 1), (1, 2), (2, 0)):
        p = (xy[:, a, :][:, None, :]
             + (xy[:, b, :][:, None, :] - xy[:, a, :][:, None, :]) * steps)
        p = p.reshape(-1, 2)
        ix = ((p[:, 0] - lo[0]) / RES).astype(int)
        iy = ((p[:, 1] - lo[1]) / RES).astype(int)
        m = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
        occ[iy[m], ix[m]] = True

    from scipy import ndimage
    return ndimage.distance_transform_edt(~occ) * RES, lo


def footprints():
    """Measured x,y extent of every vendored model, keyed by model name.

    Read from the meshes themselves rather than declared anywhere: Collada
    assets carry their own <unit>, and several here are authored in
    centimetres."""
    import glob
    out = {}
    share = _desc_share()
    roots = [os.path.join(share, 'fuel_cache', 'fuel.gazebosim.org',
                          'openrobotics', 'models'),
             os.path.join(share, 'aws_hospital_models')]

    def obj_bb(path):
        lo = np.array([np.inf] * 3); hi = np.array([-np.inf] * 3)
        for line in open(path, errors='ignore'):
            if line.startswith('v '):
                f = line.split()
                try:
                    v = np.array([float(f[1]), float(f[2]), float(f[3])])
                except ValueError:
                    continue
                lo = np.minimum(lo, v); hi = np.maximum(hi, v)
        return (None, None) if not np.isfinite(lo).all() else (lo, hi)

    def dae_bb(path):
        try:
            root = ET.parse(path).getroot()
        except ET.ParseError:
            return None, None
        unit = 1.0
        a = root.find(f'{NS}asset')
        if a is not None:
            u = a.find(f'{NS}unit')
            if u is not None and u.get('meter'):
                unit = float(u.get('meter'))
        src = {'#' + x.get('id'): x for x in root.iter(f'{NS}source') if x.get('id')}
        pts = []
        for mesh in root.iter(f'{NS}mesh'):
            v = mesh.find(f'{NS}vertices')
            if v is None:
                continue
            i = v.find(f'{NS}input')
            x = src.get(i.get('source')) if i is not None else None
            if x is None:
                continue
            arr = x.find(f'{NS}float_array')
            if arr is None:
                continue
            P = np.fromstring(arr.text, sep=' ')
            if P.size and P.size % 3 == 0:
                pts.append(P.reshape(-1, 3))
        if not pts:
            return None, None
        P = np.vstack(pts) * unit
        return P.min(axis=0), P.max(axis=0)

    for r in roots:
        if not os.path.isdir(r):
            continue
        for d in sorted(os.listdir(r)):
            base = os.path.join(r, d)
            if not os.path.isdir(base):
                continue
            for c in [base] + [os.path.join(base, v) for v in sorted(os.listdir(base))]:
                sdf = os.path.join(c, 'model.sdf')
                if not os.path.isdir(c) or not os.path.exists(sdf):
                    continue
                txt = open(sdf, errors='ignore').read()
                sc = 1.0
                m = re.search(r'<scale>([\d.eE+-]+)', txt)
                if m:
                    sc = float(m.group(1))
                best = None
                for mm in re.finditer(r'<uri>([^<]*\.(?:obj|dae))</uri>', txt):
                    rel = re.sub(r'^.*?/files/', '', mm.group(1))
                    rel = re.sub(r'^model://[^/]+/', '', rel)
                    path = os.path.join(c, rel)
                    if not os.path.exists(path):
                        continue
                    lo, hi = (obj_bb(path) if path.lower().endswith('.obj')
                              else dae_bb(path))
                    if lo is None:
                        continue
                    ext = ((hi[0] - lo[0]) * sc, (hi[1] - lo[1]) * sc)
                    if best is None or max(ext) > max(best):
                        best = ext
                if best:
                    out[d.lower()] = best
                    break
    return out


def world_contents():
    """Furniture rectangles, plus every actor's waypoint list."""
    path = os.path.join(_desc_share(), 'worlds', 'aws_hospital.sdf')
    w = open(path).read()
    fp = footprints()
    furn = []
    for blk in re.findall(r'<include>(.*?)</include>', w, re.S):
        n = re.search(r'<name>([^<]+)</name>', blk)
        # Two URI shapes live in this world: Fuel models are
        # '.../OpenRobotics/models/Name' while the vendored AWS ones are just
        # 'model://name'. Matching only the first silently fell back to a
        # 0.6 x 0.6 m default for EVERY AWS model - including the 4.84 x 3.02 m
        # reception desk, which is why a walker was routed straight through it
        # while this check reported the path clear.
        u = re.search(r'(?:models/|model://)([^<\s]+)</uri>', blk)
        q = re.search(r'<pose>([^<]+)</pose>', blk)
        if not n or 'floor_01' in n.group(1):
            continue
        v = q.group(1).split() if q else ['0', '0', '0', '0', '0', '0']
        x, y = float(v[0]), float(v[1])
        yaw = float(v[5]) if len(v) > 5 else 0.0
        ex, ey = fp.get(u.group(1).lower(), (0.6, 0.6)) if u else (0.6, 0.6)
        # Yaw only ever matters here as a quarter turn; swapping the extents is
        # enough and keeps the rectangle axis-aligned.
        if abs(math.sin(yaw)) > 0.7:
            ex, ey = ey, ex
        furn.append((n.group(1), x, y, ex, ey))
    actors = {}
    for m in re.finditer(r'<actor name="([^"]+)">(.*?)</actor>', w, re.S):
        pts = [tuple(map(float, p.split()[:2]))
               for p in re.findall(r'<pose>([^<]+)</pose>', m.group(2))]
        if pts:
            actors[m.group(1)] = pts
    return furn, actors


def seg_rect(a, b, cx, cy, ex, ey, samples=40):
    """Distance from segment AB to an axis-aligned rectangle centred (cx,cy)."""
    hx, hy = ex / 2.0, ey / 2.0
    best = float('inf')
    for i in range(samples + 1):
        t = i / samples
        px = a[0] + (b[0] - a[0]) * t
        py = a[1] + (b[1] - a[1]) * t
        dx = max(abs(px - cx) - hx, 0.0)
        dy = max(abs(py - cy) - hy, 0.0)
        best = min(best, math.hypot(dx, dy))
    return best


def seg_point(a, b, p):
    ax, ay = a; bx, by = b; px, py = p
    dx, dy = bx - ax, by - ay
    L = dx * dx + dy * dy
    t = 0.0 if L == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L))
    return math.hypot(ax + t * dx - px, ay + t * dy - py)


def check():
    dist, lo = wall_grid()
    furn, actors = world_contents()

    def wall_clear(p):
        ix = int((p[0] - lo[0]) / RES); iy = int((p[1] - lo[1]) / RES)
        if not (0 <= ix < dist.shape[1] and 0 <= iy < dist.shape[0]):
            return 0.0
        return dist[iy, ix]

    moving = {k: v for k, v in actors.items() if len(set(v)) > 1}
    still = {k: v[0] for k, v in actors.items() if len(set(v)) == 1}
    bad = 0

    for name, pts in sorted(moving.items()):
        worst_obj, worst_wall = (99.0, None), (99.0, None)
        for i in range(len(pts) - 1):
            a, b = pts[i], pts[i + 1]
            n = max(2, int(math.dist(a, b) / 0.25) + 1)
            for s in np.linspace(0, 1, n):
                q = (a[0] + (b[0] - a[0]) * s, a[1] + (b[1] - a[1]) * s)
                d = wall_clear(q)
                if d < worst_wall[0]:
                    worst_wall = (d, q)
            for on, ox, oy, ex, ey in furn:
                d = seg_rect(a, b, ox, oy, ex, ey)
                if d < worst_obj[0]:
                    worst_obj = (d, on)
            for sn, sp in still.items():
                d = seg_point(a, b, sp)
                if d < worst_obj[0]:
                    worst_obj = (d, 'person ' + sn)
        # Turn points: the first waypoint and the far end of the out-and-back.
        turn_pts = [pts[0], pts[len(pts) // 2]]
        turn_ok = True
        turn_worst = (99.0, None)
        for q in turn_pts:
            o = min((seg_rect(q, q, ox, oy, ex, ey, samples=1)
                     for _, ox, oy, ex, ey in furn), default=99.0)
            wl = wall_clear(q)
            if o < turn_worst[0]:
                turn_worst = (o, q)
            if o < TURN_OBJ_CLEAR or wl < TURN_WALL_CLEAR:
                turn_ok = False

        ok = (worst_obj[0] >= OBJ_CLEAR and worst_wall[0] >= WALL_CLEAR
              and turn_ok)
        bad += 0 if ok else 1
        note = '' if turn_ok else '  <-- TURNS TOO CLOSE'
        print(f'  {name:24s} object {worst_obj[0]:5.2f} m ({worst_obj[1]})'
              f'   wall {worst_wall[0]:5.2f} m   turn {turn_worst[0]:5.2f} m'
              f'   {"OK" if ok else "<-- CLIPS"}{note}')

    names = sorted(moving)
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            if names[i].startswith('wheelchair') and names[j].startswith('wheelchair'):
                # The chair and the person pushing it deliberately share a lane,
                # so a spatial test is meaningless for them - but exempting them
                # entirely is how they came to overlap: both trajectories ended
                # at the same point, and the attendant walked into the chair
                # there. They are compared IN TIME below instead.
                continue
            a, b = moving[names[i]], moving[names[j]]
            d = min(seg_point(a[k], a[k + 1], p)
                    for k in range(len(a) - 1) for p in b)
            if d < PATH_CLEAR:
                bad += 1
                print(f'  paths cross: {names[i]} x {names[j]}  {d:.2f} m')
    # The wheelchair pair travel one lane together, so what matters is whether
    # they are ever in the same place AT THE SAME MOMENT. Their waypoints are
    # timed, so the separation can be sampled directly along the timeline.
    if 'wheelchair_pushed' in actors and 'wheelchair_attendant' in actors:
        wpath = os.path.join(_desc_share(), 'worlds', 'aws_hospital.sdf')
        txt = open(wpath).read()

        def timed(name):
            blk = re.search(rf'<actor name="{name}">(.*?)</actor>', txt, re.S).group(1)
            out = []
            for m in re.finditer(r'<waypoint><time>([\d.]+)</time>'
                                 r'<pose>([^<]+)</pose>', blk):
                v = m.group(2).split()
                out.append((float(m.group(1)), float(v[0]), float(v[1])))
            return out

        def at(track, t):
            if t <= track[0][0]:
                return track[0][1], track[0][2]
            for k in range(len(track) - 1):
                t0, x0, y0 = track[k]
                t1, x1, y1 = track[k + 1]
                if t0 <= t <= t1:
                    f = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
                    return x0 + (x1 - x0) * f, y0 + (y1 - y0) * f
            return track[-1][1], track[-1][2]

        ch, at_ = timed('wheelchair_pushed'), timed('wheelchair_attendant')
        end = max(ch[-1][0], at_[-1][0])
        worst = min((math.dist(at(ch, t), at(at_, t)), t)
                    for t in np.arange(0.0, end + 0.05, 0.05))
        # 0.45 m is the chair's own half depth plus a little; closer than that
        # and the person is inside it.
        ok = worst[0] >= 0.45
        bad += 0 if ok else 1
        print(f'  wheelchair pair          closest in TIME {worst[0]:.2f} m '
              f'at t={worst[1]:.1f}s   {"OK" if ok else "<-- COLLIDE"}')

    print(f'\n{"FAIL" if bad else "OK"}: {bad} problem(s); '
          f'{len(moving)} moving actors, {len(still)} stationary')
    return 1 if bad else 0


def main():
    return check()


if __name__ == '__main__':
    sys.exit(main())
