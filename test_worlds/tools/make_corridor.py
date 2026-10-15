"""corridor_trap: a corridor that looks open but is blocked by a 0.38 m choke (test_1 style),
a 5 m dead-end corridor, and a zig-zag route that is the only real way through."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from worldlib import World, write_launch  # noqa: E402

NAME = 'corridor_trap'
SPAWN = (1.2, 1.2, 0.0)
U = 1.03                       # one grid cell = one box + 3 cm


def c(i):
    return 0.53 + U * i        # centre of grid column / row i


# grid 13 x 8 cells; '#' = box, '.' = free. Row 7 is the top.
GRID = [
    '....######...',   # r7  H | M north wall | E
    '......#..#.#.',   # r6  H | door H->M (c4) | M zig-zag | E (+ box island)
    '....#.#......',   # r5  H | M | door M->E (c9) | E
    '....#...##...',   # r4  H | M | E
    '##.#######.#.',   # r3  S north wall (door c2) | C north wall (choke at c6) | E (+ island)
    '.............',   # r2  S | C corridor | E
    '....######...',   # r1  S | C south wall = D north wall (shifted down 0.2 m) | E
    '....#........',   # r0  S | D dead-end corridor | E
]
w = World(NAME, U * 13 + 0.03, U * 8 + 0.03, seed=22)
for r_idx, line in enumerate(reversed(GRID)):
    for c_idx, ch in enumerate(line):
        if ch != '#':
            continue
        x, y = c(c_idx), c(r_idx)
        if r_idx == 1 and 5 <= c_idx <= 9:
            y -= 0.20          # C is 1.26 m wide, D only 0.86 m: the two do not look alike
        if r_idx == 3 and c_idx == 6:
            y -= 0.88          # the choke: leaves 0.38 m, lidar sees right through it
        if r_idx == 3 and c_idx == 1:
            x += 0.20          # door S -> H 0.86 m
        if r_idx == 7 and c_idx == 4:
            y -= 0.20          # door H -> M 0.86 m
        w.box(x, y)
w.cylinder(2.075, 6.195)       # in hall H
w.sphere(c(11), c(1))          # in room E
SPAWN_PRESETS = ('Start at the closed end of the dead-end corridor, facing the wall:\n'
                 '    x_pose:=5.6 y_pose:=0.43 yaw:=3.14\n'
                 'Start in front of the choke:  x_pose:=4.6 y_pose:=2.5 yaw:=0.0')

if __name__ == '__main__':
    here = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), NAME)
    os.makedirs(here, exist_ok=True)
    w.write(os.path.join(here, NAME + '.world'), '6.7 -5 12 0 0.95 1.5707')
    write_launch(os.path.join(here, NAME + '.launch.py'), NAME, NAME + '.world',
                 NAME + '.launch.py', SPAWN, presets=SPAWN_PRESETS)
    print('wrote', NAME, len(w.boxes), 'boxes')
