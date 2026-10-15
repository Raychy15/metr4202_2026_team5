"""hub_mix: the robot starts in a small central room with four exits at the same
distance. Each exit leads to a different element: an open hall (N), a corridor whose
far end is blocked by a 0.40 m choke (S), a winding route to a hidden room plus a
5 m dead end (W), and a room split by a fence with gaps (E). Tests back-and-forth
between equally good frontiers, and every earlier risk at once."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from worldlib import World, write_launch  # noqa: E402

NAME = 'hub_mix'
U = 1.03


def c(i):
    return 0.53 + U * i


SPAWN = (round(c(6), 2), round(c(5), 2), 0.0)
GRID = [            # 13 x 12 cells, top row (r11) first
    '#.#.......#..',   # r11  NW dead end | N hall | NE strip
    '#.#.......#..',   # r10
    '#.#.......#..',   # r9
    '#.#.......#..',   # r8
    '#.####.####..',   # r7   hub north exit c6
    '#..##...#....',   # r6
    '##...........',   # r5   west exit c4, east exit c8
    '#..##...#....',   # r4
    '#.####.##....',   # r3   hub south exit c6 | fence (placed separately) c9-12
    '..####.##....',   # r2   SW room | S corridor north wall (choke at c2)
    '.......##....',   # r1   SW room + S corridor
    '..#######....',   # r0
]
w = World(NAME, U * 13 + 0.03, U * 12 + 0.03, seed=55)
w.grid(GRID, offsets={(2, 2): (0.0, -0.66)})      # S corridor choke: 0.40 m
# E room fence along row 3: gaps 0.38 m (blocked) and 0.68 m (tight but passable)
for lo in (9.33, 10.71):
    w.box(lo + 0.5, c(3))
w.box(13.39 - 0.5, c(3))
# N hall contents
w.sphere(c(4) + 0.3, c(9))
w.cylinder(c(8), c(10))
w.box(c(6), c(10) + 0.25, 30)
# E room / NE strip contents
w.cylinder(c(11) + 0.3, c(5) + 0.4)
w.cylinder(12.75, c(10))
w.sphere(c(10) + 0.5, c(1))

if __name__ == '__main__':
    here = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), NAME)
    os.makedirs(here, exist_ok=True)
    w.write(os.path.join(here, NAME + '.world'), '6.7 -6 14 0 0.95 1.5707')
    write_launch(os.path.join(here, NAME + '.launch.py'), NAME, NAME + '.world',
                 NAME + '.launch.py', SPAWN,
                 presets='Start in the hidden south-west room:  x_pose:=1.0 y_pose:=1.0 yaw:=1.57')
    print('wrote', NAME, len(w.boxes), 'boxes')
