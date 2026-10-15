"""loop_test: six rooms in a ring around a central wall. Tests driving distance vs
straight-line distance (rooms on the other side of the wall look close) and SLAM loop
closure when the robot comes back round to where it started."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from worldlib import World, write_launch  # noqa: E402

NAME = 'loop_test'
SPAWN = (1.2, 1.2, 0.0)
U = 1.03


def c(i):
    return 0.53 + U * i


# grid 12 x 9 cells; '#' = box. Row 8 is the top.
GRID = [
    '......#.....',   # r8  TL | wall c6 | TR
    '............',   # r7  TL | door c6 | TR
    '......#.....',   # r6
    '......#.....',   # r5
    '#..######..#',   # r4  left end (c1-2) | central wall c3-c8 | right end (c9-10)
    '....#...#...',   # r3  BL | wall c4 | BM | wall c8 | BR
    '....#...#...',   # r2
    '........#...',   # r1  door c4
    '....#.......',   # r0  door c8
]
w = World(NAME, U * 12 + 0.03, U * 9 + 0.03, seed=44)
for r_idx, line in enumerate(reversed(GRID)):
    for c_idx, ch in enumerate(line):
        if ch != '#':
            continue
        x, y = c(c_idx), c(r_idx)
        if (c_idx, r_idx) == (4, 0):
            y += 0.18          # door BL -> BM (row 1) 0.88 m
        if (c_idx, r_idx) == (6, 8):
            y -= 0.12          # door TR -> TL (row 7) 0.94 m
        w.box(x, y)
w.cylinder(c(6), c(1) + 0.5)               # BM
w.sphere(c(10), c(2))                      # BR
w.cluster(9.2, 6.7, 25, [(0, 0)])          # TR: one tilted box
w.cluster(2.0, 6.6, 0, [(0, 0), (1, 0), (1, 1)])   # TL: small L
w.cylinder(4.9, 7.6)                       # TL

if __name__ == '__main__':
    here = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), NAME)
    os.makedirs(here, exist_ok=True)
    w.write(os.path.join(here, NAME + '.world'), '6.2 -5.5 13 0 0.95 1.5707')
    write_launch(os.path.join(here, NAME + '.launch.py'), NAME, NAME + '.world',
                 NAME + '.launch.py', SPAWN,
                 presets='Start in the middle of the bottom row of rooms:  x_pose:=6.7 y_pose:=3.2 yaw:=1.57')
    print('wrote', NAME, len(w.boxes), 'boxes')
