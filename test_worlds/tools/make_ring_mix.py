"""ring_mix: two big rooms joined top and bottom by slalom passages around a central
room, so the whole map is one loop (SLAM loop closure, driving distance vs straight
line). The east room is split by a fence (one 0.85 m gap, one 0.21 m), the central
room is a dead end behind a 0.84 m door, and a pocket behind a 0.41 m slot cannot be
entered. Box clusters, a sphere and cylinders fill the big rooms."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from worldlib import World, write_launch  # noqa: E402

NAME = 'ring_mix'
U = 1.03


def c(i):
    return 0.53 + U * i


SPAWN = (1.0, 1.0, 0.0)
GRID = [               # 14 x 10 cells, top row (r9) first
    '......#.......',   # r9   W room | T slalom | E room
    '........#.....',   # r8
    '....###.##....',   # r7   central room north wall, door c7
    '....#....#....',   # r6   central room
    '....#....#....',   # r5   (E fence placed separately)
    '....#....#....',   # r4
    '....#....#....',   # r3
    '....######....',   # r2
    '......#......#',   # r1   B slalom | pocket roof c13 (shifted left)
    '........#...#.',   # r0   B slalom | pocket west wall c12
]
w = World(NAME, U * 14 + 0.03, U * 10 + 0.03, seed=66)
w.grid(GRID, offsets={(8, 7): (-0.22, 0.0),      # central room door 0.84 m
                      (6, 1): (0.0, -0.20),      # B slalom squeeze 0.86 m
                      (13, 1): (-0.38, 0.0)})    # pocket roof: leaves a 0.41 m slot
# E room fence (row 5): 0.85 m gap, then a 0.21 m gap the lidar sees through
for lo in (10.36, 12.21, 13.42):
    w.box(lo + 0.5, c(5))
# W room
w.cluster(1.7, 6.2, 10, [(0, 0), (1, 0), (0, 1)])
w.sphere(2.7, 2.7)
w.cylinder(1.0, 9.1)
# E room
w.cylinder(12.4, 8.3)
w.box(11.7, 2.6, 25)
# central room
w.box(c(6) + 0.5, c(4) + 0.5, 45)

if __name__ == '__main__':
    here = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), NAME)
    os.makedirs(here, exist_ok=True)
    w.write(os.path.join(here, NAME + '.world'), '7.2 -5.5 13 0 0.95 1.5707')
    write_launch(os.path.join(here, NAME + '.launch.py'), NAME, NAME + '.world',
                 NAME + '.launch.py', SPAWN,
                 presets='Start in the central dead-end room:  x_pose:=6.2 y_pose:=3.4 yaw:=1.57')
    print('wrote', NAME, len(w.boxes), 'boxes')
