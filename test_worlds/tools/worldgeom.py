"""Load 2D wall geometry (boxes + cylinders) from a Gazebo .world and raster it."""
import math
import xml.etree.ElementTree as ET

import numpy as np


def parse_pose(txt):
    v = [float(a) for a in txt.split()] if txt else [0.0] * 6
    return v + [0.0] * (6 - len(v))


def compose(a, b):
    """2D compose pose a (x,y,yaw) with local pose b."""
    x, y, t = a
    return (x + b[0] * math.cos(t) - b[1] * math.sin(t),
            y + b[0] * math.sin(t) + b[1] * math.cos(t), t + b[2])


def load_world(path):
    """Return list of shapes: ('box', cx, cy, yaw, sx, sy) / ('cyl', cx, cy, r)."""
    root = ET.parse(path).getroot()
    w = root.find('world')
    state_poses = {}
    st = w.find('state')
    if st is not None:
        for m in st.findall('model'):
            p = parse_pose(m.find('pose').text)
            links = {}
            for lk in m.findall('link'):
                lp = lk.find('pose')
                if lp is not None:
                    links[lk.get('name')] = parse_pose(lp.text)
            state_poses[m.get('name')] = (p, links)
    shapes = []
    for m in w.findall('model'):
        name = m.get('name')
        if 'ground' in name:
            continue
        mp = parse_pose(m.find('pose').text if m.find('pose') is not None else None)
        links_state = {}
        if name in state_poses:
            mp, links_state = state_poses[name]
        mpose = (mp[0], mp[1], mp[5])
        for lk in m.findall('link'):
            if lk.get('name') in links_state:
                lp = links_state[lk.get('name')]
                lpose = (lp[0], lp[1], lp[5])          # state link poses are world poses
                base = lpose
            else:
                lpe = lk.find('pose')
                lp = parse_pose(lpe.text if lpe is not None else None)
                base = compose(mpose, (lp[0], lp[1], lp[5]))
            for c in lk.findall('collision'):
                ce = c.find('pose')
                cp = parse_pose(ce.text if ce is not None else None)
                if cp[2] - 0 > 0.6 and False:
                    continue
                pose = compose(base, (cp[0], cp[1], cp[5]))
                g = c.find('geometry')[0]
                if g.tag == 'box':
                    sx, sy, sz = [float(a) for a in g.find('size').text.split()]
                    shapes.append(('box', pose[0], pose[1], pose[2], sx, sy))
                elif g.tag == 'cylinder':
                    shapes.append(('cyl', pose[0], pose[1], float(g.find('radius').text)))
                elif g.tag == 'sphere':
                    # a unit sphere sits on the floor: at robot body height
                    # (0-0.2 m) it is ~0.4 m wide in radius, the lidar plane
                    # (0.12 m) cuts it at ~0.33 m
                    r = float(g.find('radius').text)
                    shapes.append(('cyl', pose[0], pose[1], math.sqrt(max(r * r - (r - 0.2) ** 2, 0.0))))
    return shapes


def raster(shapes, res=0.02, pad=0.5):
    xs, ys = [], []
    for s in shapes:
        if s[0] == 'box':
            r = 0.5 * math.hypot(s[4], s[5])
        else:
            r = s[3]
        xs += [s[1] - r, s[1] + r]
        ys += [s[2] - r, s[2] + r]
    x0, y0 = min(xs) - pad, min(ys) - pad
    W = int((max(xs) + pad - x0) / res) + 1
    H = int((max(ys) + pad - y0) / res) + 1
    yy, xx = np.mgrid[0:H, 0:W]
    px = x0 + (xx + 0.5) * res
    py = y0 + (yy + 0.5) * res
    sid = np.full((H, W), -1, np.int32)
    for i, s in enumerate(shapes):
        if s[0] == 'box':
            _, cx, cy, t, sx, sy = s
            r = 0.5 * math.hypot(sx, sy) + res
            c0, c1 = int((cx - r - x0) / res), int((cx + r - x0) / res) + 2
            r0, r1 = int((cy - r - y0) / res), int((cy + r - y0) / res) + 2
            c0, r0 = max(c0, 0), max(r0, 0)
            dx = px[r0:r1, c0:c1] - cx
            dy = py[r0:r1, c0:c1] - cy
            lx = dx * math.cos(t) + dy * math.sin(t)
            ly = -dx * math.sin(t) + dy * math.cos(t)
            inside = (np.abs(lx) <= sx / 2) & (np.abs(ly) <= sy / 2)
        else:
            _, cx, cy, rr = s
            c0, c1 = int((cx - rr - x0) / res), int((cx + rr - x0) / res) + 2
            r0, r1 = int((cy - rr - y0) / res), int((cy + rr - y0) / res) + 2
            c0, r0 = max(c0, 0), max(r0, 0)
            inside = (px[r0:r1, c0:c1] - cx) ** 2 + (py[r0:r1, c0:c1] - cy) ** 2 <= rr * rr
        sub = sid[r0:r1, c0:c1]
        sub[inside] = i
    return sid, (x0, y0, res)


def normal_at(shape, hx, hy):
    if shape[0] == 'box':
        _, cx, cy, t, sx, sy = shape
        dx, dy = hx - cx, hy - cy
        lx = dx * math.cos(t) + dy * math.sin(t)
        ly = -dx * math.sin(t) + dy * math.cos(t)
        if abs(lx) / (sx / 2) > abs(ly) / (sy / 2):
            nl = (math.copysign(1, lx), 0.0)
        else:
            nl = (0.0, math.copysign(1, ly))
        return (nl[0] * math.cos(t) - nl[1] * math.sin(t), nl[0] * math.sin(t) + nl[1] * math.cos(t))
    _, cx, cy, r = shape
    d = math.hypot(hx - cx, hy - cy) or 1.0
    return ((hx - cx) / d, (hy - cy) / d)
