"""
Generate narrow_test_v4.world.

Built the same way as the course worlds (metr4202.world / env1.world):
1 m unit boxes, unit cylinders and unit spheres (radius 0.5), all dynamic
1 kg models, with small random offsets / rotations between boxes. If the robot
hits something it moves, exactly like in the assessment.

Changes from v3, from what broke SLAM in the v3 run:
* no thin walls: neighbouring corridors are separated by >= 1 m of solid box,
  so a scan in one corridor no longer fits the next corridor over
* no repeated look-alike lanes (the v3 serpentine) and no regular pillar grid
* every corridor is short (<= ~4 m straight) or has a corner / bump in view
* everything is within ~2.6 m of an obstacle, so the 3.5 m lidar maps it all

Tight-space tests kept: 0.80 m pinch door, 0.85-0.9 m corridor with two
90 degree bends, 0.9 m dead-end alley (4.4 m long), hairpin U-turn around a
wall end (0.98 m gap), sphere/cylinder slalom, a box sticking into a lane, and
a pocket behind a 0.45 m slot that the robot cannot enter (must give up).

Interior: x 0..14 m, y 0..11 m. Spawn (1.2, 1.2).
"""
import math
import os
import random

SPAWN = (1.2, 1.2)
W, H = 14.0, 11.0
P = 1.03            # box pitch: 1 m boxes, ~3 cm gaps (course worlds: 0-5 cm)
JITTER_XY = 0.006
JITTER_DEG = 0.6


def row(y, x_first, n, pitch=P):
    return [(x_first + i * pitch, y) for i in range(n)]


def col(x, y_first, n, pitch=P):
    return [(x, y_first + i * pitch) for i in range(n)]


BOXES = []
# ---- outer wall (inner faces at x=0, x=14, y=0, y=11); gaps only leak outwards
BOXES += row(-0.5, -0.5, 15, pitch=(W + 1.0) / 14)
BOXES += row(H + 0.5, -0.5, 15, pitch=(W + 1.0) / 14)
BOXES += col(-0.5, 0.52, 10, pitch=(H - 1.04) / 9)
BOXES += col(W + 0.5, 0.52, 10, pitch=(H - 1.04) / 9)

# ---- A: start room x 0..2.88, y 0..2.88
BOXES += [(0.53, 3.38), (2.35, 3.38), (3.38, 3.38)]  # north wall, 0.82 m door x 1.03..1.85
BOXES += [(3.38, 2.35), (3.38, 0.55)]                # east wall, 0.80 m pinch door y 1.05..1.85

# ---- S: corridor east (0.80 m), north (0.90 m), east into O (0.81 m): two 90 deg bends
BOXES += [(4.41, 0.55), (5.44, 0.55)]                # leg 1 south side
BOXES += [(4.41, 2.35), (4.41, 3.38)]                # leg 1 north side (+ P's south-east corner)
BOXES += [(5.44, 2.35), (5.44, 3.38), (5.44, 4.41)]  # leg 2 west side (= P's east wall)
BOXES += col(7.34, 0.55, 4)                          # leg 2 east side (= O's west wall)
BOXES += [(6.47, 5.45), (7.50, 5.45)]                # top of leg 2 (= bottom of J)

# ---- P: hall north of A, x 0..4.94, y 3.88..9.1: sphere, cylinder and a box
BOXES += [(5.44, 5.44), (5.44, 7.29), (5.44, 8.32)]  # P's east wall, 0.85 m door y 5.94..6.79
BOXES += row(9.6, 0.53, 5)                           # P's north wall (= T's south side)
BOXES += [(1.40, 7.60)]
SPHERES = [(1.55, 5.30)]
CYLINDERS = [(3.45, 6.35)]

# ---- T: 0.9 m wide, 5.1 m long dead-end alley along the top (y 10.1..11)

# ---- O: room x 7.84..14, y 0..5.3 with a sphere and a cylinder
BOXES += [(8.53, 5.80), (9.56, 5.80), (10.59, 6.10), (11.62, 5.80), (12.65, 5.80)]
#                                     ^ sticks 0.3 m up into the lane above (landmark + pinch)
#        0.85 m gap x 13.15..14 at the north-east corner joins O to the lane
SPHERES += [(10.30, 2.60)]
CYLINDERS += [(12.30, 3.90)]
# pocket Q (x 12.55..14, y 0..1.07) is entered only through a 0.35 m slot
BOXES += [(12.05, 0.55), (13.15, 1.57)]

# ---- H: hairpin. Lane y 6.3..7.7 runs east (1.4 m, 1.1 m at the bump), then a
#      U-turn round the tongue end through a 0.98 m gap (x 13.02..14)
BOXES += row(8.2, 8.40, 5)                           # tongue x 7.9..13.02

# ---- R: two small rooms above the tongue (y 8.7..11), only reachable via H
BOXES += [(8.40, 9.23), (8.40, 10.26)]               # closes R from J
BOXES += [(10.90, 10.35)]                            # rooms joined under this box (1.15 m)

# ---- J: hall x 5.94..7.9, y 5.95..11 joins P (door), T (top left) and the lane


def generate(seed=4202):
    rnd = random.Random(seed)
    boxes = []
    for (x, y) in BOXES:
        boxes.append((x + rnd.uniform(-JITTER_XY, JITTER_XY),
                      y + rnd.uniform(-JITTER_XY, JITTER_XY),
                      math.radians(rnd.uniform(-JITTER_DEG, JITTER_DEG))))
    return boxes


def inertial():
    return ('<inertial><mass>1</mass><inertia><ixx>0.166667</ixx><ixy>0</ixy><ixz>0</ixz>'
            '<iyy>0.166667</iyy><iyz>0</iyz><izz>0.166667</izz></inertia></inertial>')


MAT = ('<material><script><uri>file://media/materials/scripts/gazebo.material</uri>'
       '<name>Gazebo/Grey</name></script></material>')


def model(name, x, y, yaw, geom):
    return ("    <model name='%s'>\n      <pose>%.4f %.4f 0.5 0 0 %.5f</pose>\n"
            "      <link name='link'>%s\n"
            "        <collision name='collision'><geometry>%s</geometry><max_contacts>10</max_contacts></collision>\n"
            "        <visual name='visual'><geometry>%s</geometry>%s</visual>\n"
            "      </link>\n    </model>\n" % (name, x, y, yaw, inertial(), geom, geom, MAT))


def world_xml():
    out = ['<?xml version="1.0"?>\n<sdf version="1.6">\n  <world name="narrow_test_v4">\n',
           '    <include><uri>model://ground_plane</uri></include>\n',
           '    <include><uri>model://sun</uri></include>\n',
           '    <physics type="ode"><real_time_update_rate>1000.0</real_time_update_rate>'
           '<max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor>'
           '<ode><solver><type>quick</type><iters>150</iters><precon_iters>0</precon_iters>'
           '<sor>1.400000</sor><use_dynamic_moi_rescaling>1</use_dynamic_moi_rescaling></solver>'
           '<constraints><cfm>0.00001</cfm><erp>0.2</erp><contact_max_correcting_vel>2000.000000'
           '</contact_max_correcting_vel><contact_surface_layer>0.01000</contact_surface_layer>'
           '</constraints></ode></physics>\n',
           '    <scene><shadows>false</shadows></scene>\n',
           '    <gui fullscreen="0"><camera name="user_camera">'
           '<pose>7 -6 13 0 0.95 1.5707</pose><view_controller>orbit</view_controller>'
           '<projection_type>perspective</projection_type></camera></gui>\n']
    for i, (x, y, yaw) in enumerate(generate()):
        out.append(model('unit_box_%d' % i, x, y, yaw, '<box><size>1 1 1</size></box>'))
    for i, (x, y) in enumerate(SPHERES):
        out.append(model('unit_sphere_%d' % i, x, y, 0.0, '<sphere><radius>0.5</radius></sphere>'))
    for i, (x, y) in enumerate(CYLINDERS):
        out.append(model('unit_cylinder_%d' % i, x, y, 0.0,
                         '<cylinder><radius>0.5</radius><length>1</length></cylinder>'))
    out.append('  </world>\n</sdf>\n')
    return ''.join(out)


if __name__ == '__main__':
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, 'narrow_test_v4.world')
    with open(path, 'w') as f:
        f.write(world_xml())
    print('wrote', path, '-', len(BOXES), 'boxes,', len(SPHERES), 'spheres,',
          len(CYLINDERS), 'cylinders')
