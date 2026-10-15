# Combined test worlds

Three worlds that mix the elements of the earlier test worlds. They are built the same way
as the course worlds: 1 m boxes, cylinders and spheres, all 1 kg and free to move, so a
collision shows up as a box that moved.

Each folder holds:

- the world and its launch file
- `*_layout.png`: the map, with light blue where the robot's centre can go
- `*_offline_run_v9.png`: how frontier_selector v9 explored it in a 2D simulator (no
  Gazebo, no SLAM drift), as a reference to compare your Gazebo run against. Red dots are
  frontier goals, orange crosses are short drives, and the path is coloured from dark
  (early) to yellow (late).

## Install and run

```bash
cd ~ && unzip -o ~/Downloads/test_worlds_v2.zip     # adds to ~/test_worlds/
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch ~/test_worlds/hub_mix/hub_mix.launch.py        # README Terminal 1
```

The other four terminals are exactly as in the README. To start somewhere else, add
`x_pose:=X y_pose:=Y yaw:=RADIANS`; each launch file lists a suggested alternative start.

## The worlds

| World | What it combines | What would count as failing | Offline run (v9) |
| --- | --- | --- | --- |
| `hub_mix` (13.4 x 12.4 m) | Starts in a small central room with four exits at the same distance, which is a back-and-forth trap. N: open hall with a sphere, cylinder and tilted box. S: corridor ending in a 0.40 m choke with a room visible behind it. W: zig-zag to that hidden room, plus a 5 m dead end. E: room split by a fence (0.38 m blocked, 0.68 m tight), and a strip past a cylinder (0.88 m squeeze). | Going back and forth between exits, repeated trips to the choke, or never reaching the hidden south-west room or the room behind the fence. | 100 % in 821 s, no contacts |
| `ring_mix` (14.5 x 10.3 m) | Two big rooms joined top and bottom by slalom passages, so the map is one loop. A dead-end central room behind a 0.84 m door. A fence with one 0.85 m gap, and a pocket behind a 0.41 m slot that cannot be entered. | Stuck trying the pocket, missing the room below the fence, or the map jumping when the loop closes. | 100 % in 780 s, no contacts |
| `tight_mix` (15.5 x 11.4 m) | Tight left half: 0.80-0.84 m doors, an S-bend whose straight-on shortcut is a 0.38 m choke, a hall with a sphere, and a 5 m dead-end alley. Open right half: a fence with one usable 0.85 m gap, box clusters, a U pocket, spheres and cylinders. | Hitting walls in the S-bend or doors, trying to squeeze through the choke, or missing the room behind the fence. | 100 % in 840 s, no contacts |

## What to record

- Total time, and whether it ends with `Exploration complete`.
- Any `Bump guard`, `SLAM correction`, `No progress for 45 s` or `Blacklisted frontier`
  lines, with an RViz screenshot.
- Whether any box moved in Gazebo.
- If it goes back and forth: roughly where, and the Terminal 4 lines from that stretch.

## Regenerating

`tools/make_*.py` rebuilds a world and `tools/check_*.py` re-checks it (overlaps,
reachability, gap widths). Both need numpy, scipy and matplotlib.
