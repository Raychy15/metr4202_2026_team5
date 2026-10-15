# Test worlds for high-risk failures

Four Gazebo worlds, each aimed at one way the exploration can fail in the assessment.
They are built like the course worlds (`metr4202.world`, `env1.world`): 1 m unit boxes,
unit cylinders and unit spheres, all 1 kg and free to move. If the robot hits something,
the box moves, just as it would in the assessment.

Every world has a layout picture (`*_layout.png`) in its folder. Light blue marks where the
robot's centre can go with Nav2's 0.22 m radius.

## Install

```bash
cd ~ && unzip -o ~/Downloads/test_worlds_v1.zip     # creates ~/test_worlds/...
```

## Run

Use the README's 5 terminals as usual, except that Terminal 1 becomes one of:

```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch ~/test_worlds/gaps_test/gaps_test.launch.py
ros2 launch ~/test_worlds/corridor_trap/corridor_trap.launch.py
ros2 launch ~/test_worlds/open_test/open_test.launch.py
ros2 launch ~/test_worlds/loop_test/loop_test.launch.py
```

To start the robot somewhere else, add `x_pose:=X y_pose:=Y yaw:=RADIANS`. Each launch file
lists useful start points at its top.

Between runs, shut everything down cleanly:

```bash
pkill -9 gzserver; pkill -9 gzclient; ros2 daemon stop
```

## The worlds

| World | Risk it tests | What failure looks like | Pass |
| --- | --- | --- | --- |
| `gaps_test` (12 x 9 m) | Tester 2: goals placed where the robot cannot safely go. A row of boxes splits the arena with gaps of 0.30, 0.60, 0.85, 0.40 and 0.79 m. The lidar sees through all of them, so frontiers appear behind every gap. Also: boxes rotated 15-45 deg with sharp corners, a 0.8 m squeeze past a diamond, a funnel that narrows to 0.35 m, and a 0.9 m dead-end slot. | The robot drives at the 0.30 or 0.40 gap or into the funnel throat, a box moves, or the bump guard fires repeatedly. | It crosses only through the 0.85 or 0.79 gap (0.60 is allowed but tight), nothing moves, and both halves are mapped. |
| `corridor_trap` (13.4 x 8.3 m) | Tester 1: stuck in a corridor. The direct corridor (1.26 m) is blocked by a 0.38 m choke that the lidar sees through. The only real way round is a door, a hall, a 0.86 m door and a zig-zag with three 90 deg bends. There is also a 5 m long, 0.86 m wide dead-end corridor. | The robot keeps returning to the choke, sits still for minutes, or gets wedged at the end of the dead end. | The choke frontier is blacklisted after at most 3 tries (log: `attempt 3 of 3`). The robot takes the zig-zag, turns round in the dead end, and finishes. |
| `open_test` (14 x 12 m) | Efficiency in an open arena like `metr4202.world`: scattered L, T, U, S and X box clusters, 3 spheres and 4 cylinders. Squeezes of about 0.6-0.7 m sit between a diamond, a cylinder and the walls. | Trips back and forth across the arena, a long time with little new map, or SLAM drifting along the long outer walls. | Explores mostly area by area, with no obvious back-and-forth. Note the total time so later versions can be compared. |
| `loop_test` (12.4 x 9.3 m) | Rooms on the far side of a 1 m wall look close in a straight line but are 8-10 m away by driving. Six rooms form a ring, so SLAM closes a loop when the robot comes back round. | The robot drives at the central wall to reach the room behind it; the map jumps or doubles walls when the loop closes. | It goes round through the doors. Any `SLAM correction` lines are followed by a clean map, and the robot carries on. |

`narrow_test_v4` (sent separately) remains the tight-spaces test: 0.8 m doors, bends, a
hairpin U-turn and an unreachable pocket.

## Start points worth trying

| World | Start | Why |
| --- | --- | --- |
| `gaps_test` | `x_pose:=3.8 y_pose:=0.45 yaw:=0.0` | Starts with the body about 0.35 m from a sharp box corner, testing the first move and the bump guard. |
| `corridor_trap` | `x_pose:=5.6 y_pose:=0.43 yaw:=3.14` | Starts at the closed end of the dead-end corridor facing the wall, testing start-up with nothing ahead. |
| `corridor_trap` | `x_pose:=4.6 y_pose:=2.5 yaw:=0.0` | Starts facing the choke, the most tempting frontier. |
| `open_test` | `x_pose:=7.0 y_pose:=8.0 yaw:=0.0` | Starts in the middle of the open floor. |
| `loop_test` | `x_pose:=6.7 y_pose:=3.2 yaw:=1.57` | Starts in the middle room facing the central wall, where the room behind it is the closest frontier in a straight line. |

## What to record for each run

- Time to finish, or to stop making progress.
- Any `Bump guard`, `SLAM correction` or `Blacklisted frontier` lines, with a screenshot of
  RViz at that moment.
- Whether any box moved in Gazebo (a sign of contact).
- A final screenshot of the map.

## Regenerating

`tools/make_*.py` rebuilds each world from `tools/worldlib.py`, and `tools/check_*.py` re-runs the checks:
overlaps, reachability, gap widths and the farthest point from an obstacle. They need
numpy, scipy and matplotlib, and are only needed if you want to change a world.
