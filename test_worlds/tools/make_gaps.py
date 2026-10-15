"""gaps_test: obstacles with gaps that the lidar sees through but the robot must not take."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from worldlib import World, write_launch  # noqa: E402

NAME = 'gaps_test'
SPAWN = (1.0, 1.0, 0.0)
w = World(NAME, 12.0, 9.0, seed=11)

# --- the fence at x = 6..7 splits the arena. Gaps from the bottom up:
#     0.30 | 0.60 (Nav2 can just fit) | 0.85 | 0.40 | 0.79
#     lidar sees through all of them, so frontiers appear behind every gap
for lo in (0.03, 1.33, 2.93, 4.78, 6.18, 7.97):
    w.box(6.5, lo + 0.5)

# --- left half: rotated boxes with sharp corners, a sphere, a cylinder
w.box(4.3, 1.5, 45)          # diamond: 0.79 m squeeze to the bottom wall
w.box(2.3, 3.4, 30)
w.box(4.0, 5.0, 15)
w.sphere(1.3, 5.9)
w.cylinder(3.3, 7.5)

# --- right half: a funnel that narrows to ~0.4 m (frontier visible through the throat)
# (two boxes tilted +-20 deg: mouth 1.03 m on the left, throat 0.35 m on the right)
w.box(9.3, 1.30, 20)
w.box(9.3, 2.932, -20)
# a 0.9 m wide, 2 m deep dead-end slot along the top wall
w.box(8.47, 8.47)
w.box(8.47, 7.44)
w.box(10.37, 8.47)
w.box(10.37, 7.44)
w.sphere(11.2, 5.2)
w.cylinder(8.2, 5.4)

if __name__ == '__main__':
    here = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), NAME)
    os.makedirs(here, exist_ok=True)
    w.write(os.path.join(here, NAME + '.world'), '6 -5 12 0 0.95 1.5707')
    write_launch(os.path.join(here, NAME + '.launch.py'), NAME, NAME + '.world',
                 NAME + '.launch.py', SPAWN,
                 presets='Tight start next to the diamond:  x_pose:=3.8 y_pose:=0.45 yaw:=0.0')
    print('wrote', NAME, len(w.boxes), 'boxes')
