"""tight_mix: a tight left half (0.80-0.84 m doors, an S-bend corridor whose
straight-on shortcut is a 0.38 m choke, a hall with a sphere, a 5 m dead-end alley)
next to an open right half (a fence with one usable 0.85 m gap, a room of clusters,
a sphere and cylinders). Tests the crash-prone tight spaces of the earlier worlds
together with open-area efficiency."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from worldlib import World, write_launch  # noqa: E402

NAME = 'tight_mix'
U = 1.03


def c(i):
    return 0.53 + U * i


SPAWN = (1.0, 1.0, 0.0)
GRID = [                # 15 x 11 cells, top row (r10) first
    '#..............',   # r10  T dead-end alley (c1-c5) | J | middle | NE room
    '#####..........',   # r9   P north wall
    '....#..........',   # r8   P | P east wall | J | middle | NE room
    '...............',   # r7   door P -> J at c4
    '....#..........',   # r6   (tongue c10-c14 placed separately)
    '....###........',   # r5   top of the S-bend
    '....#..........',   # r4   S-bend leg 2 (c5) and exit east (c6)
    '#.###.#........',   # r3   A north wall, door c1 | leg 2
    '...##.#........',   # r2   door A -> leg 1 below (3,2) | leg 2 | choke box (6,2)
    '...............',   # r1   A | leg 1 | choke cell c6 | middle
    '...####........',   # r0
]
w = World(NAME, U * 15 + 0.03, U * 11 + 0.03, seed=77)
w.grid(GRID, offsets={(2, 3): (-0.22, 0.0),     # door A -> P 0.84 m
                      (3, 2): (0.0, -0.26),     # door A -> S-bend 0.80 m
                      (6, 2): (0.0, -0.68)})    # straight-on shortcut: 0.38 m choke
# fence between the middle strip and the south-east room (x = c(10))
for lo in (0.03, 1.33, 3.18, 4.21, 5.24):       # gaps 0.30 and 0.85 (the only way in)
    w.box(c(10), lo + 0.5)
# tongue: closes the south-east room from the north-east room
for i in range(10, 15):
    w.box(c(i), c(6) + 0.06)
# P hall
w.sphere(1.6, 5.6)
w.box(2.8, 7.7, 20)
# middle strip
w.sphere(c(8), c(8))
w.box(c(8), 2.6, 30)
# north-east room: U pocket + cylinder
w.cluster(12.4, 9.0, 0, [(-1, 0), (0, -1), (1, 0)])
w.cylinder(14.4, 8.0)
# south-east room
w.sphere(13.4, 2.0)
w.cylinder(13.1, 4.9)

if __name__ == '__main__':
    here = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), NAME)
    os.makedirs(here, exist_ok=True)
    w.write(os.path.join(here, NAME + '.world'), '7.7 -6 14 0 0.95 1.5707')
    write_launch(os.path.join(here, NAME + '.launch.py'), NAME, NAME + '.world',
                 NAME + '.launch.py', SPAWN,
                 presets='Start in the open middle strip:  x_pose:=8.0 y_pose:=6.0 yaw:=3.14')
    print('wrote', NAME, len(w.boxes), 'boxes')
