"""
Small helper for building course-style Gazebo worlds:
1 m unit boxes, unit cylinders and unit spheres (radius 0.5), all dynamic 1 kg
models like metr4202.world / env1.world, plus an outer wall of boxes.
"""
import math
import random

INERTIAL = ('<inertial><mass>1</mass><inertia><ixx>0.166667</ixx><ixy>0</ixy><ixz>0</ixz>'
            '<iyy>0.166667</iyy><iyz>0</iyz><izz>0.166667</izz></inertia></inertial>')
MAT = ('<material><script><uri>file://media/materials/scripts/gazebo.material</uri>'
       '<name>Gazebo/Grey</name></script></material>')
PHYSICS = ('    <physics type="ode"><real_time_update_rate>1000.0</real_time_update_rate>'
           '<max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor>'
           '<ode><solver><type>quick</type><iters>150</iters><precon_iters>0</precon_iters>'
           '<sor>1.400000</sor><use_dynamic_moi_rescaling>1</use_dynamic_moi_rescaling></solver>'
           '<constraints><cfm>0.00001</cfm><erp>0.2</erp><contact_max_correcting_vel>2000.000000'
           '</contact_max_correcting_vel><contact_surface_layer>0.01000</contact_surface_layer>'
           '</constraints></ode></physics>\n')
PITCH = 1.03          # box pitch along a wall: 1 m boxes, ~3 cm gaps


class World:
    def __init__(self, name, width, height, seed=4202, jitter_xy=0.006, jitter_deg=0.6):
        self.name = name
        self.W, self.H = width, height
        self.rnd = random.Random(seed)
        self.jxy, self.jdeg = jitter_xy, jitter_deg
        self.boxes = []        # (x, y, yaw_rad)
        self.spheres = []
        self.cylinders = []
        self._outer()

    # ---------------------------------------------------------------- shapes
    def box(self, x, y, yaw_deg=None):
        """yaw_deg=None -> axis aligned with the usual small random jitter."""
        if yaw_deg is None:
            x += self.rnd.uniform(-self.jxy, self.jxy)
            y += self.rnd.uniform(-self.jxy, self.jxy)
            yaw = math.radians(self.rnd.uniform(-self.jdeg, self.jdeg))
        else:
            yaw = math.radians(yaw_deg)
        self.boxes.append((x, y, yaw))

    def row(self, y, x_first, n, pitch=PITCH):
        for i in range(n):
            self.box(x_first + i * pitch, y)

    def col(self, x, y_first, n, pitch=PITCH):
        for i in range(n):
            self.box(x, y_first + i * pitch)

    def cluster(self, cx, cy, yaw_deg, cells, pitch=PITCH):
        """Rigid group of boxes on a grid (cells = (i, j) offsets), rotated as one."""
        t = math.radians(yaw_deg)
        for i, j in cells:
            dx, dy = i * pitch, j * pitch
            self.boxes.append((cx + dx * math.cos(t) - dy * math.sin(t),
                               cy + dx * math.sin(t) + dy * math.cos(t), t))

    def grid(self, rows, offsets=None, pitch=PITCH, origin=0.53):
        """Boxes from an ASCII grid ('#' = box), top row first. Cell (c, r)
        has its centre at (origin + c * pitch, origin + r * pitch).
        offsets {(c, r): (dx, dy)} shift single boxes (narrow doors, chokes)."""
        offsets = offsets or {}
        for r_idx, line in enumerate(reversed(rows)):
            for c_idx, ch in enumerate(line):
                if ch == '#':
                    dx, dy = offsets.get((c_idx, r_idx), (0.0, 0.0))
                    self.box(origin + pitch * c_idx + dx, origin + pitch * r_idx + dy)

    def sphere(self, x, y):
        self.spheres.append((x, y))

    def cylinder(self, x, y):
        self.cylinders.append((x, y))

    def _outer(self):
        W, H = self.W, self.H
        nx = int(math.floor((W + 1.0) / 1.03)) + 1      # pitch >= 1.03: no overlaps
        ny = int(math.floor((H - 1.04) / 1.03)) + 1
        for y in (-0.5, H + 0.5):
            for i in range(nx):
                self.box(-0.5 + i * (W + 1.0) / (nx - 1), y)
        for x in (-0.5, W + 0.5):
            for i in range(ny):
                self.box(x, 0.52 + i * (H - 1.04) / (ny - 1))

    # ---------------------------------------------------------------- output
    @staticmethod
    def _model(name, x, y, yaw, geom):
        return ("    <model name='%s'>\n      <pose>%.4f %.4f 0.5 0 0 %.5f</pose>\n"
                "      <link name='link'>%s\n"
                "        <collision name='collision'><geometry>%s</geometry>"
                "<max_contacts>10</max_contacts></collision>\n"
                "        <visual name='visual'><geometry>%s</geometry>%s</visual>\n"
                "      </link>\n    </model>\n" % (name, x, y, yaw, INERTIAL, geom, geom, MAT))

    def xml(self, camera):
        out = ['<?xml version="1.0"?>\n<sdf version="1.6">\n  <world name="%s">\n' % self.name,
               '    <include><uri>model://ground_plane</uri></include>\n',
               '    <include><uri>model://sun</uri></include>\n', PHYSICS,
               '    <scene><shadows>false</shadows></scene>\n',
               '    <gui fullscreen="0"><camera name="user_camera"><pose>%s</pose>'
               '<view_controller>orbit</view_controller><projection_type>perspective'
               '</projection_type></camera></gui>\n' % camera]
        for i, (x, y, yaw) in enumerate(self.boxes):
            out.append(self._model('unit_box_%d' % i, x, y, yaw, '<box><size>1 1 1</size></box>'))
        for i, (x, y) in enumerate(self.spheres):
            out.append(self._model('unit_sphere_%d' % i, x, y, 0.0,
                                   '<sphere><radius>0.5</radius></sphere>'))
        for i, (x, y) in enumerate(self.cylinders):
            out.append(self._model('unit_cylinder_%d' % i, x, y, 0.0,
                                   '<cylinder><radius>0.5</radius><length>1</length></cylinder>'))
        out.append('  </world>\n</sdf>\n')
        return ''.join(out)

    def write(self, path, camera):
        with open(path, 'w') as f:
            f.write(self.xml(camera))
        return path


LAUNCH = '''"""
Gazebo + TurtleBot3 (waffle_pi) in {world}.

Keep this file next to {world} and run (replaces README step 1):
    export TURTLEBOT3_MODEL=waffle_pi
    ros2 launch ~/test_worlds/{folder}/{launch}

Start somewhere else:  x_pose:=X y_pose:=Y yaw:=RADIANS
{presets}
SLAM, Nav2, exploration and RViz are launched exactly as in the README.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    here = os.path.dirname(os.path.abspath(__file__))
    world = os.path.join(here, '{world}')
    model = os.environ.get('TURTLEBOT3_MODEL', 'waffle_pi')
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    gazebo_ros = get_package_share_directory('gazebo_ros')
    sdf = os.path.join(tb3, 'models', 'turtlebot3_' + model, 'model.sdf')

    return LaunchDescription([
        DeclareLaunchArgument('x_pose', default_value='{x}'),
        DeclareLaunchArgument('y_pose', default_value='{y}'),
        DeclareLaunchArgument('yaw', default_value='{yaw}'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={{'world': world}}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3, 'launch', 'robot_state_publisher.launch.py')),
            launch_arguments={{'use_sim_time': 'true'}}.items()),
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-entity', model, '-file', sdf,
                        '-x', LaunchConfiguration('x_pose'),
                        '-y', LaunchConfiguration('y_pose'),
                        '-z', '0.01',
                        '-Y', LaunchConfiguration('yaw')]),
    ])
'''


def write_launch(path, folder, world_file, launch_file, spawn, presets=''):
    with open(path, 'w') as f:
        f.write(LAUNCH.format(world=world_file, folder=folder, launch=launch_file,
                              x=spawn[0], y=spawn[1], yaw=spawn[2], presets=presets))
