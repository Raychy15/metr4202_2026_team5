"""Checks for generated worlds: overlaps, reachability, passage widths, layout plot."""
import math

import numpy as np
from scipy import ndimage

from worldgeom import load_world, raster

ROBOT_R = 0.22          # Nav2 robot_radius (turtlebot3 waffle_pi default)


def _corners(s):
    _, cx, cy, t, sx, sy = s
    return [(cx + dx * math.cos(t) - dy * math.sin(t), cy + dx * math.sin(t) + dy * math.cos(t))
            for dx, dy in [(-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2), (-sx / 2, sy / 2)]]


def _sat(a, b):
    A, B = _corners(a), _corners(b)
    for P in (A, B):
        for i in range(4):
            ex, ey = P[(i + 1) % 4][0] - P[i][0], P[(i + 1) % 4][1] - P[i][1]
            nx, ny = -ey, ex
            pa = [x * nx + y * ny for x, y in A]
            pb = [x * nx + y * ny for x, y in B]
            if max(pa) <= min(pb) or max(pb) <= min(pa):
                return False
    return True


def overlaps(shapes):
    out = []
    boxes = [s for s in shapes if s[0] == 'box']
    for i in range(len(boxes)):
        for j in range(i + 1, len(boxes)):
            a, b = boxes[i], boxes[j]
            if math.hypot(a[1] - b[1], a[2] - b[2]) < 1.5 and _sat(a, b):
                out.append(((round(a[1], 2), round(a[2], 2)), (round(b[1], 2), round(b[2], 2))))
    circ = [s for s in shapes if s[0] == 'cyl']
    for c in circ:
        for b in boxes + [d for d in circ if d is not c]:
            if b[0] == 'cyl':
                if math.hypot(c[1] - b[1], c[2] - b[2]) < c[3] + b[3] and c is not b:
                    out.append(((c[1], c[2]), (b[1], b[2])))
                continue
            _, bx, by, t, sx, sy = b
            lx = (c[1] - bx) * math.cos(t) + (c[2] - by) * math.sin(t)
            ly = -(c[1] - bx) * math.sin(t) + (c[2] - by) * math.cos(t)
            qx = max(-sx / 2, min(sx / 2, lx))
            qy = max(-sy / 2, min(sy / 2, ly))
            if math.hypot(lx - qx, ly - qy) < 0.5:     # real radius 0.5 at mid height
                out.append(((c[1], c[2]), (round(bx, 2), round(by, 2))))
    return out


def check(world, spawn, points, title, labels=(), png=None, res=0.02):
    shapes = load_world(world)
    sid, (x0, y0, r) = raster(shapes, res)
    occ = sid >= 0
    dist = ndimage.distance_transform_edt(~occ) * r
    ok = dist >= ROBOT_R
    lab, _ = ndimage.label(ok)
    sr, sc = int((spawn[1] - y0) / r), int((spawn[0] - x0) / r)
    reach = lab == lab[sr, sc]
    ov = overlaps(shapes)
    print('== %s' % title)
    print('  models: %d | overlapping pairs: %d %s' % (len(shapes), len(ov), ov[:4]))
    print('  reachable floor (robot radius %.2f m): %.1f m2 | farthest reachable point from an '
          'obstacle: %.2f m' % (ROBOT_R, reach.sum() * r * r, dist[reach].max()))
    res_points = {}
    for name, (x, y, expect) in points.items():
        rr_, cc_ = int((y - y0) / r), int((x - x0) / r)
        w = 2 * dist[rr_, cc_]
        rch = bool(reach[rr_, cc_])
        flag = '' if expect is None or rch == expect else '   <-- UNEXPECTED'
        res_points[name] = (w, rch)
        print('  %-44s width %.2f m  reachable: %-5s%s' % (name, w, rch, flag))
    if png:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        H, W = occ.shape
        fig, ax = plt.subplots(figsize=(11, 11 * H / W + 0.6))
        img = np.ones(occ.shape + (3,))
        img[reach] = (0.85, 0.93, 1.0)
        img[occ] = (0.25, 0.25, 0.25)
        ax.imshow(img, origin='lower', extent=[x0, x0 + W * r, y0, y0 + H * r])
        ax.plot(spawn[0], spawn[1], '*', color='#1f6fd1', ms=18)
        for x, y, t in labels:
            ax.text(x, y, t, fontsize=8.5, color='#b3261e')
        ax.set_xticks(range(0, int(x0 + W * r) + 1))
        ax.set_yticks(range(0, int(y0 + H * r) + 1))
        ax.grid(alpha=0.25)
        ax.set_title(title + '\nlight blue = where the robot centre can go (Nav2 radius 0.22 m); '
                     'star = default spawn', fontsize=10)
        plt.savefig(png, dpi=75, bbox_inches='tight')
        plt.close(fig)
    return ov, res_points
