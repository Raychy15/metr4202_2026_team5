"""open_test: a 14 x 12 m arena with scattered box clusters, spheres and cylinders,
like metr4202.world / env1.world but with every spot within ~2 m of an obstacle."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from worldlib import World, write_launch  # noqa: E402

NAME = 'open_test'
SPAWN = (1.5, 6.0, 0.0)
w = World(NAME, 14.0, 12.0, seed=33)
w.cluster(2.6, 9.4, 12, [(0, 0), (1, 0), (0, -1), (0, -2)])          # L
w.cluster(7.4, 10.0, -8, [(-1, 0), (0, 0), (1, 0), (0, -1)])         # T
w.cluster(11.6, 8.6, 20, [(-1, 0), (0, -1), (1, 0), (-1, 1), (1, 1)])  # U (1.06 m pocket)
w.cluster(6.6, 5.6, 0, [(-1, 1), (0, 1), (0, 0), (1, -1), (0, -1)])  # S / Z
w.cluster(11.4, 2.6, 0, [(-1, -1), (1, -1), (0, 0), (-1, 1), (1, 1)], pitch=1.06)  # X (env1)
w.box(3.0, 2.6, 30)
w.box(8.4, 1.6, 45)
w.sphere(4.4, 7.4)
w.sphere(9.3, 6.0)
w.sphere(1.6, 12.0 - 1.3)
w.cylinder(5.2, 2.0)
w.cylinder(13.0, 5.6)
w.cylinder(9.6, 10.9)
w.cylinder(1.2, 3.9)

if __name__ == '__main__':
    here = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), NAME)
    os.makedirs(here, exist_ok=True)
    w.write(os.path.join(here, NAME + '.world'), '7 -7 16 0 0.95 1.5707')
    write_launch(os.path.join(here, NAME + '.launch.py'), NAME, NAME + '.world',
                 NAME + '.launch.py', SPAWN,
                 presets='Start in the middle of the open floor:  x_pose:=7.0 y_pose:=8.0 yaw:=0.0')
    print('wrote', NAME, len(w.boxes), 'boxes')
