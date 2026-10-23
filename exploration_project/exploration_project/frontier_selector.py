import rclpy
import time
import math

from rclpy.node import Node

#from nav2_msgs.msg import BehaviorTreeLog
from geometry_msgs.msg import PoseStamped, Twist
from action_msgs.srv import CancelGoal
from std_msgs.msg import Float32MultiArray
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data



class FrontierNavigator(Node):

    def __init__(self):
        super().__init__('frontier_navigator')  # node name


        # Subscribe to frontier candidates from Ray
        self.frontier_subscription = self.create_subscription(
            Float32MultiArray,
            'frontier_candidates',
            self.frontier_callback,
            10
        )

        # Get robot position from TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )


        self.marker_timer = self.create_timer(
            0.5,
            self.publish_active_goal_marker
        )

        self.map_msg = None
        self.unknown_check_radius = 0.5
        self.minimum_unknown_cells = 5

        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Laser scan, used to check that a short drive is clear
        self.last_scan = None
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        # Bump guard: if the laser shows something within guard_margin of the
        # robot's body while it is driving, cancel the Nav2 goal, back
        # straight off slowly and choose a different frontier
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cancel_client = self.create_client(
            CancelGoal, 'navigate_to_pose/_action/cancel_goal')
        self.guard_margin = 0.07       # m between body outline and obstacle
        self.guard_clear = 0.25        # m that must be free to back off that way
        self.backoff_speed = 0.06      # m/s
        self.backoff_time = 2.0        # s
        self.backoff_vx = 0.0
        self.backoff_start = None
        self.backoff_until = None
        self.last_guard_time = -1e9
        self.last_guard_position = None
        self._extent_key = None
        self._extent = None
        self.backoff_timer = self.create_timer(0.1, self.backoff_step)

        # Create a subscriber to the behavior_tree_log topic
        # The constructor inputs are (message type, topic name, associated
        # callback function, message queue length)
        # self.subscription = self.create_subscription(
        #     BehaviorTreeLog,
        #     'behavior_tree_log',
        #     self.bt_log_callback,
        #     10)
        # self.subscription  # prevent unused variable warning

        # Create a publisher
        # The constructor inputs are (message type, topic name, message queue
        # length)
        self.publisher_ = self.create_publisher(
            PoseStamped,
            'goal_pose',
            10)


        # RViz waypoint visualisation
        self.frontier_marker_pub = self.create_publisher(
            MarkerArray,
            '/frontier_markers',
            10
        )

        self.active_goal_marker_pub = self.create_publisher(
            Marker,
            '/active_frontier_goal',
            10
        )
        # Keep track of the number of waypoints sent
        self.waypoint_counter = 0

        # Store the most recent frontier candidates
        self.frontiers = []
        # Failed goals are cooled down, not permanently discarded.
        # Entries are (x, y, failure_time).
        self.failed_frontiers = []
        self.failed_frontier_cooldown = 60.0

        # Visited goals are also cooled down, not permanently discarded.
        # Entries are (x, y, visit_time)
        self.visited_frontiers = []
        self.visited_radius = 0.5
        self.visited_frontier_cooldown = 120.0

        # Exploration efficiency: keep working on one map sector instead of
        # hopping between nearby-looking frontiers on opposite sides of walls.
        # The sectors are fixed 3 m squares in the map coordinate frame.
        self.sector_size = 3.0
        self.active_sector = None
        # When a sector has no eligible candidates, favour other sectors for
        # a while. This is a preference, not a permanent exploration ban.
        self.sector_cooldowns = {}  # (sector_x, sector_y) -> monotonic expiry
        self.sector_revisit_cooldown = 45.0

        # Hard-coded sector completion rule. A cell is "mapped" if SLAM
        # reports it as free OR occupied (value >= 0), rather than unknown (-1).
        # Count the entire fixed-size sector, including parts outside /map, so
        # a partially published map does not falsely give 100% completion.
        self.sector_target_coverage = 0.80  # Change to 0.70 / 0.90 to tune.

        # Escape conditions: an inaccessible/partly-walled-off sector may
        # never reach 80%. Do not lock the robot there indefinitely.
        self.sector_empty_grace = 12.0  # No usable frontier in active sector.
        self.sector_stall_timeout = 75.0  # No >=1 percentage-point gain.
        self.sector_progress_increment = 0.01
        self.sector_empty_since = None
        self.sector_last_progress_time = None
        self.sector_progress_baseline = None
        self.sector_progress_key = None
        self.wall_crossing_penalty = 2.5  # metres of scoring penalty
        self.local_radius = 2.0

        # start-up deadlock -> slam_toolbox only adds new scans to the
        # map after the robot moves/turns, so a stationary robot never gets a
        # new frontie so changing to make it so after no_frontier_wait seconds, 
        # robot turn on the spot which makes map update
        # if still nothing clear list and attempt again
        self.no_frontier_since = time.monotonic()
        self.no_frontier_wait = 5.0
        self.spinning = False
        self.spin_start_time = None
        self.spin_duration = 8.0
        self.unstick_attempts = 0
        self.max_spins_before_reset = 2

        #slam_toolbox only adds a scan after the robot has DRIVEN about
        # 0.5 m, a short drive into open space is used instead
        self.nudge_distance = 0.8
        # longer short drive tried first: covers corridors in fewer steps
        self.long_nudge_distance = 1.5
        self.nudge_goal = None
        self.last_nudge_angle = None
        self.last_nudge_from = None

        #  a goal closer than this to the robot counts as already there
        self.min_goal_distance = 0.5

        #drop the active goal if the updated map puts it closer than
        # this to a wall
        self.goal_wall_clearance = 0.27

        # Track whether a goal was already set
        self.goal_active = False

        # Current goal
        self.current_goal = None

        # Current robot position
        self.robot_position = None

        # Position robot can return to if it gets stuck
        self.previous_position = None

        # is robot going back to previous position (got stuck)
        self.recovering = False

        # Time recovery started
        self.recovery_start_time = None

        # Give robot time to return to previous position
        self.recovery_time = 15.0

        # Used to check whether robot has become stuck
        self.last_robot_position = None

        # Time robot was last seen moving
        self.last_movement_time = None

        # Minimum movement needed to count as moving
        self.minimum_movement = 0.10

        # How long robot can stay in same area before recovery
        self.stuck_timeout = 12.0

         # Check for a stuck goal once per second
        self.timer = self.create_timer(
            1.0,
            self.check_goal_timeout
        )

    def map_callback(self, msg):
        self.map_msg = msg


    def has_unexplored_area(self, x, y):
        if self.map_msg is None:
            return False

        info = self.map_msg.info
        width = info.width
        height = info.height
        resolution = info.resolution

        if resolution <= 0:
            return False

        origin_x = info.origin.position.x
        origin_y = info.origin.position.y

        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)

        radius = max(
            1, int(self.unknown_check_radius / resolution)
        )

        unknown_count = 0

        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):

                if dx * dx + dy * dy > radius * radius:
                    continue

                nx = grid_x + dx
                ny = grid_y + dy

                if not (0 <= nx < width and 0 <= ny < height):
                    continue

                index = ny * width + nx

                if self.map_msg.data[index] == -1:
                    unknown_count += 1

                    if unknown_count >= self.minimum_unknown_cells:
                        return True

        return False


    def get_robot_position(self):

        try:

            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            return (x, y)

        except Exception as e:
            #error
            self.get_logger().info(
                'Could not get robot position: %s'
                % str(e)
            )

            return None


    def frontier_callback(self, msg: Float32MultiArray):
        #gather all data on frontiers
        
        self.frontiers = []

        data = msg.data

        # Data has location in x,y and size of cluster
        for i in range(0, len(data), 3):

            x = data[i]
            y = data[i + 1]
            size = data[i + 2]

            self.frontiers.append(
                (x, y, size)
            )

        self.get_logger().info(
            'Received %d frontier candidates'
            % len(self.frontiers)
        )

        self.publish_frontier_markers()

        # a wall can appear next to a goal that looked open when it was chosen. Drop that goal (and
        # blacklist it) instead of driving the robot into the wall
        if (self.goal_active and self.current_goal is not None
                and self.wall_distance(*self.current_goal) < self.goal_wall_clearance):
            self.get_logger().warning(
                'Goal (%.2f, %.2f) is now too close to a wall in the updated map - '
                'choosing again' % self.current_goal)
            self.blacklist_current_goal()
            self.goal_active = False
            self.current_goal = None
            self.last_robot_position = None
            self.last_movement_time = None

        # Send a goal if one is not already active and robot is not in recovery
        if (not self.goal_active and not self.recovering and not self.spinning
                and self.backoff_until is None and len(self.frontiers) > 0):
            self.send_waypoint()


    def frontier_sector(self, x, y):
        """Return a stable (column, row) sector ID for map-frame coordinates.

        math.floor, rather than int, also handles negative map coordinates.
        """
        return (math.floor(x / self.sector_size),
                math.floor(y / self.sector_size))

    def map_line_is_clear(self, x1, y1, x2, y2):
        """Whether the direct line crosses only known free occupancy cells.

        This is a quick ranking heuristic, NOT a collision or path check.
        Nav2 remains responsible for planning around walls and inflations.
        """
        if self.map_msg is None:
            return False
        info = self.map_msg.info
        if info.resolution <= 0:
            return False
        length = math.hypot(x2 - x1, y2 - y1)
        steps = max(1, math.ceil(length / info.resolution))
        for i in range(steps + 1):
            t = i / steps
            wx = x1 + t * (x2 - x1)
            wy = y1 + t * (y2 - y1)
            gx = math.floor((wx - info.origin.position.x) / info.resolution)
            gy = math.floor((wy - info.origin.position.y) / info.resolution)
            if not (0 <= gx < info.width and 0 <= gy < info.height):
                return False
            if self.map_msg.data[gy * info.width + gx] != 0:
                return False
        return True

    def sector_coverage(self, sector):
        """Fraction of this fixed square that has been observed by SLAM.

        Includes known free (0), known obstacles (e.g. 100), and any other
        nonnegative occupancy estimate. Unknown (-1) and cells outside the
        currently published map count as unexplored. Cell centres determine
        sector membership; map origin is assumed axis-aligned as in the
        detector. This is *map coverage*, not navigable floor coverage.
        """
        if sector is None or self.map_msg is None:
            return None

        info = self.map_msg.info
        resolution = info.resolution
        if resolution <= 0 or info.width <= 0 or info.height <= 0:
            return None

        # Half-open world-space sector [x0,x1) x [y0,y1).
        x0, y0 = (sector[0] * self.sector_size,
                  sector[1] * self.sector_size)
        x1, y1 = x0 + self.sector_size, y0 + self.sector_size
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y

        # Cell centre = origin + (grid_index + 0.5) * resolution.
        gx0 = math.ceil((x0 - origin_x) / resolution - 0.5)
        gx1 = math.ceil((x1 - origin_x) / resolution - 0.5)
        gy0 = math.ceil((y0 - origin_y) / resolution - 0.5)
        gy1 = math.ceil((y1 - origin_y) / resolution - 0.5)
        total = (gx1 - gx0) * (gy1 - gy0)
        if total <= 0:
            return None

        # Only scan actual map cells, but keep the full-sector denominator.
        left = max(0, gx0)
        right = min(info.width, gx1)
        bottom = max(0, gy0)
        top = min(info.height, gy1)
        known = 0
        if left < right and bottom < top:
            for gy in range(bottom, top):
                row = gy * info.width
                for gx in range(left, right):
                    if self.map_msg.data[row + gx] >= 0:
                        known += 1

        return known / total

    def note_sector_progress(self, now, coverage):
        """Track last meaningful coverage increase for the active sector."""
        if self.sector_progress_key != self.active_sector:
            self.sector_progress_key = self.active_sector
            self.sector_progress_baseline = coverage
            self.sector_last_progress_time = now
        elif (coverage is not None and
              (self.sector_progress_baseline is None or
               coverage >= self.sector_progress_baseline +
               self.sector_progress_increment)):
            self.sector_progress_baseline = coverage
            self.sector_last_progress_time = now

    def select_active_sector(self, candidates, robot_x, robot_y, now):
        """Commit to a sector until it is >=80% mapped or becomes unproductive.

        The percentage is measured from /map (known cells / all sector cells).
        If an active sector still has suitable frontiers and its coverage is
        below the target, normally stay there regardless of scores elsewhere.
        Switch to another *unfinished* sector when the target is met, or use
        the no-frontier/stagnation escape conditions to avoid deadlock.
        """
        groups = {}
        for candidate in candidates:
            key = self.frontier_sector(candidate[0], candidate[1])
            groups.setdefault(key, []).append(candidate)

        self.sector_cooldowns = {
            key: until for key, until in self.sector_cooldowns.items()
            if until > now
        }

        current_candidates = groups.get(self.active_sector, [])
        current_coverage = self.sector_coverage(self.active_sector)
        if self.active_sector is not None:
            self.note_sector_progress(now, current_coverage)

        target_met = (current_coverage is not None and
                      current_coverage >= self.sector_target_coverage)
        if current_candidates:
            self.sector_empty_since = None
        elif self.active_sector is not None and self.sector_empty_since is None:
            self.sector_empty_since = now

        stalled = (current_coverage is not None and not target_met and
                   self.sector_last_progress_time is not None and
                   now - self.sector_last_progress_time >=
                   self.sector_stall_timeout)

        # Prefer exploring below-target sectors before revisiting mapped ones.
        other_sectors = [k for k in groups if k != self.active_sector]
        other_coverage = {k: self.sector_coverage(k) for k in other_sectors}
        incomplete = [k for k in other_sectors
                      if (other_coverage[k] is None or
                          other_coverage[k] < self.sector_target_coverage)]

        # A useful sector is locked until it reaches the target. The lock can
        # be released early only if map coverage has stopped increasing.
        if current_candidates and (not incomplete or
                                   (not target_met and not stalled)):
            return current_candidates

        # No eligible active-sector candidates: wait briefly for a fresh scan
        # or cooldown expiry. The caller can run its existing recovery logic.
        if (self.active_sector is not None and not current_candidates and
            not target_met and self.sector_empty_since is not None and
            now - self.sector_empty_since < self.sector_empty_grace):
            return []

        # If no other useful sector exists, continue exploring the current one.
        if current_candidates and not incomplete:
            return current_candidates
        if not other_sectors and current_candidates:
            return current_candidates
        if not groups:
            return []

        old_sector = self.active_sector
        if old_sector is not None:
            self.sector_cooldowns[old_sector] = now + self.sector_revisit_cooldown

        # On completion/stall, prefer sectors not yet at the target. If no
        # unfinished sector is available, use any other eligible one.
        options = incomplete if incomplete else other_sectors
        if not options:
            return current_candidates
        fresh_options = [k for k in options if k not in self.sector_cooldowns]
        if fresh_options:
            options = fresh_options

        def sector_cost(key):
            return min(
                math.hypot(f[0] - robot_x, f[1] - robot_y)
                - 0.02 * f[2]
                + (0.0 if self.map_line_is_clear(
                    robot_x, robot_y, f[0], f[1]
                ) else self.wall_crossing_penalty)
                for f in groups[key]
            )

        self.active_sector = min(options, key=sector_cost)
        self.sector_empty_since = None
        new_coverage = self.sector_coverage(self.active_sector)
        self.note_sector_progress(now, new_coverage)
        explanation = ('target reached' if target_met else
                       'mapping stalled' if stalled else
                       'no eligible frontier' if old_sector is not None else
                       'initial sector')
        self.get_logger().info(
            f'Exploration sector changed: {old_sector} -> '
            f'{self.active_sector} ({explanation}); '
            f'previous coverage: '
            f'{"unknown" if current_coverage is None else f"{current_coverage:.1%}"}; '
            f'new coverage: '
            f'{"unknown" if new_coverage is None else f"{new_coverage:.1%}"}; '
            f'target: {self.sector_target_coverage:.0%}'
        )
        return groups[self.active_sector]

    def choose_frontier(self):
        
        if not self.frontiers:
            self.get_logger().warning('No frontier candidates received')
            return None

        robot_position = self.get_robot_position()

        # Old failed waypoints can be reconsidered after the map or robot moves.
        now = time.monotonic()
        before = len(self.failed_frontiers)
        self.failed_frontiers = [
            (fx, fy, failed_at)
            for fx, fy, failed_at in self.failed_frontiers
            if now - failed_at < self.failed_frontier_cooldown
        ]
        if before != len(self.failed_frontiers):
            self.get_logger().info(
                f'Expired {before - len(self.failed_frontiers)} failed-frontier cooldown(s)'
            )

        # Visited spots can be reconsidered after their cooldown, so a frontier
        # that was not fully mapped on the first visit is not lost for good
        self.visited_frontiers = [
            (vx, vy, visited_at)
            for vx, vy, visited_at in self.visited_frontiers
            if now - visited_at < self.visited_frontier_cooldown
        ]

        # Keep only frontiers that are not blacklisted
        available_frontiers = []

        
        # Sort all available frontiers by size (largest first)
        #sorted_frontiers = sorted(self.frontiers, key=lambda f: f[2], reverse=True)
        
        for frontier in self.frontiers:

            x, y, size = frontier

            # Ignore frontiers near locations already visited
            already_visited = any(
                math.hypot(x - vx, y - vy) < self.visited_radius
                for vx, vy, visited_at in self.visited_frontiers
            )

            if already_visited:
                self.get_logger().info(
                    f'Rejected frontier ({x:.2f}, {y:.2f}): already visited'
                )
                continue

            is_blacklisted = False

            for failed_x, failed_y, failed_at in self.failed_frontiers:
                distance_to_failed = (
                    (x - failed_x) ** 2 +
                    (y - failed_y) ** 2
                ) ** 0.5

                if distance_to_failed < 0.3:
                    is_blacklisted = True
                    break

            if is_blacklisted:
                self.get_logger().info(
                    f'Rejected frontier ({x:.2f}, {y:.2f}): blacklisted'
                )
                continue

             # Ignore frontiers without enough unexplored area
            if not self.has_unexplored_area(x, y):
                self.get_logger().info(
                    f'Rejected frontier ({x:.2f}, {y:.2f}): '
                    'insufficient unexplored area'
                )
                continue

            available_frontiers.append(frontier)

        self.get_logger().info(
            f'Frontiers: {len(self.frontiers)} received, '
            f'{len(available_frontiers)} available, '
            f'{len(self.failed_frontiers)} blacklisted positions'
        )

        if not available_frontiers:
            # Also start the empty-sector clock when every candidate is
            # rejected globally, not just when other sectors have candidates.
            if self.active_sector is not None and self.sector_empty_since is None:
                self.sector_empty_since = now
            self.get_logger().warning(
                'No suitable frontier passed the filters'
            )
            return None

        # If robot position is unavailable, use largest frontier
        if robot_position is None:
            self.get_logger().warning(
                'Robot position unavailable - using largest frontier'
            )
            return max(available_frontiers, key=lambda f: f[2])

        robot_x, robot_y = robot_position

        # Stay in the same sector across successive goals. The sector only
        # changes once no candidate in it survives the existing filters.
        available_frontiers = self.select_active_sector(
            available_frontiers, robot_x, robot_y, now
        )
        if not available_frontiers:
            self.get_logger().info(
                'Waiting for fresh mapping in the active sector before switching'
            )
            return None

        # Prefer a directly free corridor when available. Otherwise keep the
        # blocked candidates: Nav2 may still plan a safe route around a wall.
        direct_frontiers = [
            f for f in available_frontiers
            if self.map_line_is_clear(robot_x, robot_y, f[0], f[1])
        ]
        if direct_frontiers:
            available_frontiers = direct_frontiers

        # Keep the original 2 m local preference, but within the active sector
        # and after preferring frontiers not obviously separated by walls.
        local_radius = self.local_radius
        local_frontiers = [
            frontier for frontier in available_frontiers
            if math.hypot(
                frontier[0] - robot_x,
                frontier[1] - robot_y
            ) <= local_radius
        ]

        if local_frontiers:
            available_frontiers = local_frontiers

            self.get_logger().info(
                f'Prioritising {len(local_frontiers)} '
                f'frontiers within {local_radius:.1f} m'
            )
        else:
            self.get_logger().info(
                'No suitable local frontiers - considering all remaining frontiers'
            )

        best_frontier = None
        best_score = float('inf')

        # Weights for distance and frontier size
        distance_weight = 1.0
        size_weight = 0.02

        for frontier in available_frontiers:
            x, y, size = frontier

            distance = (
                (x - robot_x) ** 2 +
                (y - robot_y) ** 2
            ) ** 0.5

            score = (
                distance_weight * distance
                - size_weight * size
            )

            self.get_logger().debug(
                f'Candidate ({x:.2f}, {y:.2f}): '
                f'distance={distance:.2f}, '
                f'size={size}, cost={score:.2f}'
            )

            if score < best_score:
                best_score = score
                best_frontier = frontier

        if best_frontier is None:
            self.get_logger().warning('No suitable frontier')
            return None

        self.get_logger().info(
            f'Selected frontier {best_frontier} in sector {self.active_sector}, '
            f'score: {best_score:.2f}'
        )

        return best_frontier



            # # If the robot just attempted this general area, skip it to prevent infinite loops
            # if self.current_goal is not None:
            #     dist = ((frontier[0] - self.current_goal[0])**2 + (frontier[1] - self.current_goal[1])**2)**0.5
            #     if dist < 0.5:  # If within 50cm of the last goal, ignore it and try the next largest
            #         continue
            
            # return frontier
            
        # No suitable frontier


    def send_waypoint(self):

        if len(self.frontiers) == 0:
            if self.no_frontier_since is None:
                self.no_frontier_since = time.monotonic()
            return
        
        # Keep track of the number of waypoints sent
        self.waypoint_counter += 1

        # Choose which frontier to visit
        frontier = self.choose_frontier()

        if frontier is None:
            self.get_logger().info('No suitable frontier')
            if self.no_frontier_since is None:
                self.no_frontier_since = time.monotonic()
            return

        # found a frontier: reset the start-up/deadlock fallback
        #self.no_frontier_since = None
        #self.unstick_attempts = 0

        x, y, size = frontier

        # A nearby frontier is not necessarily explored yet. Nudge to obtain
        # a new scan, but only mark it visited after genuinely reaching a goal.
        here = self.get_robot_position()
        if here is not None and math.hypot(x - here[0], y - here[1]) < self.min_goal_distance:
            self.get_logger().info(
                'Frontier (%.2f, %.2f) is close - trying a scan-producing nudge'
                % (x, y))
            # mark it visited so the next choice is a different frontier
            self.visited_frontiers.append((x, y, time.monotonic()))
            toward = math.atan2(y - here[1], x - here[0])
            if not self.start_nudge(here, toward):
                # Avoid retrying this same too-close goal every second.
                self.failed_frontiers.append((x, y, time.monotonic()))
                self.get_logger().warning(
                    'Nearby frontier has no safe nudge route - cooling it down'
                )
                if self.no_frontier_since is None:
                    self.no_frontier_since = time.monotonic()
            return

        goal = PoseStamped()

        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()

        goal.pose.position.x = x
        goal.pose.position.y = y

        # # No particular heading required
        goal.pose.orientation.w = 1.0

        # Get current robot position in map frame
        self.robot_position = self.get_robot_position()

        # Save where the robot was before attempting this frontier
        if self.robot_position is not None:
            self.previous_position = self.robot_position

        # self.publisher_.publish(goal)

        # self.goal_active = True
        # self.current_goal = (x, y)

        # # Start checking movement from current robot position
        # self.last_robot_position = self.robot_position

        # if self.robot_position is None:
        #     self.get_logger().info(
        #         'Cannot send goal - robot position unavailable'
        #     )
        #     return
        # Get current robot position in map frame
        self.robot_position = self.get_robot_position()

        if self.robot_position is None:
            self.get_logger().info(
                'Cannot send goal - robot position unavailable'
            )
            if self.no_frontier_since is None:
                self.no_frontier_since = time.monotonic()
            return

        # Save position before attempting frontier
        self.previous_position = self.robot_position

        # Publish frontier goal
        self.publisher_.publish(goal)

        self.goal_active = True
        self.current_goal = (x, y)

        # Reset timer only after sending a goal
        self.no_frontier_since = None
        self.unstick_attempts = 0

        self.publish_active_goal_marker()

        # Start checking movement
        self.last_robot_position = self.robot_position


        self.last_movement_time = time.monotonic()

        # # Create Nav2 goal
        # goal = NavigateToPose.Goal()

        # goal.pose.header.frame_id = 'map'
        # goal.pose.header.stamp = self.get_clock().now().to_msg()

        # goal.pose.pose.position.x = x
        # goal.pose.pose.position.y = y


        self.get_logger().info(
            'Sent frontier goal: '
            'x=%.2f, y=%.2f, size=%.0f'
            % (x, y, size)
        )


    #removing for test
    # def bt_log_callback(self, msg: BehaviorTreeLog):


    #     # Check behaviour tree for completed navigation
    #     for event in msg.event_log:

    #         if (event.node_name == 'NavigateRecovery'
    #                 and event.current_status == 'IDLE'):

    #             # Only do this if we actually have an active goal
    #             if not self.goal_active:
    #                 continue

    #             self.get_logger().info(
    #                 'Navigation finished - selecting new frontier'
    #             )

    #             self.goal_active = False
    #             self.current_goal = None
    #             self.goal_start_time = None
    #             # Latest map/frontiers will be used
    #             # if len(self.frontiers) > 0:
    #             #     self.send_waypoint()


    def check_goal_timeout(self):

        # bump guard is backing the robot off: wait for it to finish
        if self.backoff_until is not None:
            return

        # finish an in-place turn, then look for frontiers again
        if self.spinning:
            elapsed = time.monotonic() - self.spin_start_time
            position = self.get_robot_position()
            arrived = (self.nudge_goal is not None and position is not None
                       and elapsed > 2.0
                       and math.hypot(position[0] - self.nudge_goal[0],
                                      position[1] - self.nudge_goal[1]) < 0.3)
            if elapsed < self.spin_duration and not arrived:
                return
            self.spinning = False
            self.nudge_goal = None
            self.get_logger().info('Turn finished - selecting frontier again')
            self.send_waypoint()
            return

        # nothing to do for a while -> unstick instead of waiting forever
        if (not self.goal_active and not self.recovering
                and self.no_frontier_since is not None
                and time.monotonic() - self.no_frontier_since >= self.no_frontier_wait):
            self.unstick()
            return

        # Retry when there is no active goal
        if not self.goal_active and not self.recovering:
            if self.no_frontier_since is None:
                self.no_frontier_since = time.monotonic()

            if self.frontiers:
                self.send_waypoint()

            return

        # Check whether recovery period has finished
        if self.recovering:

            current_position = self.get_robot_position()

            if current_position is None:
                return

            

            if current_position is not None and self.previous_position is not None:

                x, y = current_position
                target_x, target_y = self.previous_position

                # Distance from robot to recovery position
                distance = (
                    (x - target_x) ** 2
                    + (y - target_y) ** 2
                ) ** 0.5

                # Robot has returned close enough
                if distance < 0.20:

                    self.get_logger().info(
                        'Recovery position reached - resuming exploration'
                    )

                    self.recovering = False
                    self.recovery_start_time = None

                    self.last_robot_position = None
                    self.last_movement_time = None

                    return

            # Check if recovery is taking too long
            elapsed_recovery = (
                time.monotonic()
                - self.recovery_start_time
            )

            if elapsed_recovery >= self.recovery_time:

                self.get_logger().warning(
                    'Recovery timed out - select new frontier'
                )
                self.recovering = False
                self.recovery_start_time = None

                self.goal_active = False
                self.current_goal = None

                self.last_robot_position = None
                self.last_robot_yaw = None #remove if i dont add code for it later
                self.last_movement_time = None

                # Stop repeating the warning every second
                #self.recovery_start_time = time.monotonic()

            return

            
        #check normal nav

        # Nothing to check if there is no active goal
        if not self.goal_active:
            return
        
        # Get current robot position in map frame
        current_position = self.get_robot_position()

        if current_position is None:
            return

        if self.current_goal is not None:
            distance_to_goal = math.hypot(
                current_position[0] - self.current_goal[0],
                current_position[1] - self.current_goal[1]
            )

            if distance_to_goal < 0.3:
                self.visited_frontiers.append(
                    (self.current_goal[0], self.current_goal[1], time.monotonic()))

                self.get_logger().info(
                    'Frontier reached - marking as visited'
                )

                self.goal_active = False
                self.current_goal = None
                self.last_robot_position = None
                self.last_movement_time = None

                # Start timer for finding the next frontier
                self.no_frontier_since = time.monotonic()
                return
        
        
        # First position measurement
        if self.last_robot_position is None:

            self.last_robot_position = current_position
            self.last_movement_time = time.monotonic()

            return
        
        current_x, current_y = current_position
        last_x, last_y = self.last_robot_position


        # Calculate how far robot has moved
        distance_moved = (
            (current_x - last_x) ** 2
            + (current_y - last_y) ** 2
        ) ** 0.5


        # Robot has moved enough
        if distance_moved >= self.minimum_movement:

            self.last_robot_position = current_position
            self.last_movement_time = time.monotonic()

            self.get_logger().info(
                'Robot moving - %.2f m since last check'
                % distance_moved
            )

            return


        # Robot has not moved enough
        if self.last_movement_time is None:
            self.last_movement_time = time.monotonic()
            return


        time_stuck = (
            time.monotonic()
            - self.last_movement_time
        )


        self.get_logger().info(
            'Robot movement low for %.1f seconds'
            % time_stuck
        )


        # Robot has remained in same area too long
        if time_stuck >= self.stuck_timeout:

            self.get_logger().info(
                'Robot appears stuck - starting recovery'
            )

            # Remember this frontier caused a problem
            self.blacklist_current_goal()

            self.goal_active = False
            self.current_goal = None

            self.last_robot_position = None
            self.last_movement_time = None


            # Try the existing LiDAR-checked short movement rather than
            # returning to the previous location and crossing explored space.
            position = self.get_robot_position()
            if position is None or not self.start_nudge(position, None):
                # no clear short drive (e.g. pressed against something):
                # go back to where this goal was started from instead of
                # sending the next goal from the stuck spot
                self.get_logger().warning(
                    'No safe short recovery movement - returning to previous position'
                )
                self.start_recovery()

            # Start timer for finding another frontier
            self.no_frontier_since = time.monotonic()

            self.get_logger().info(
                'Navigation stalled - selecting a different frontier'
            )


    def unstick(self):
        # called when no frontier has passed the filters for a while
        self.no_frontier_since = None
        self.unstick_attempts += 1

        if self.unstick_attempts > self.max_spins_before_reset:
            # turning did not help: the 'visited' frontiers are the only ones
            # left, so allow them again
            self.get_logger().warning(
                'Still no frontier after turning - keeping visited/failed frontiers')
            self.unstick_attempts = 1

        position = self.get_robot_position()
        if position is None:
            self.no_frontier_since = time.monotonic()
            return
        yaw = self.get_robot_yaw()

        #drive 0.8 m into open space so SLAM adds a new scan
        if self.start_nudge(position, None):
            return

        # only turn on the spot if nothing is close enough for the
        # robot's corners to hit while turning
        if self.nearest_obstacle() < 0.32:
            self.get_logger().warning(
                'No suitable frontier and no room to drive or turn - waiting')
            self.no_frontier_since = time.monotonic()
            return

        # Goal at the robot's own position, facing the opposite way
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = position[0]
        goal.pose.position.y = position[1]
        new_yaw = yaw + math.pi
        goal.pose.orientation.z = math.sin(new_yaw / 2.0)
        goal.pose.orientation.w = math.cos(new_yaw / 2.0)
        self.publisher_.publish(goal)

        self.spinning = True
        self.spin_start_time = time.monotonic()
        self.get_logger().warning(
            'No suitable frontier - turning on the spot to update the map '
            '(attempt %d)' % self.unstick_attempts)


    def get_robot_yaw(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            q = t.transform.rotation
            return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        except Exception:
            return 0.0


    def start_recovery(self):

        if self.previous_position is None:

            self.get_logger().info(
                'No previous position available for recovery'
            )

            return

        x, y = self.previous_position

        recovery_goal = PoseStamped()

        recovery_goal.header.frame_id = 'map'
        recovery_goal.header.stamp = self.get_clock().now().to_msg()

        recovery_goal.pose.position.x = x
        recovery_goal.pose.position.y = y
        recovery_goal.pose.orientation.w = 1.0

        self.publisher_.publish(recovery_goal)

        self.recovering = True
        self.recovery_start_time = time.monotonic()

        self.get_logger().info(
            'Recovery - returning to x=%.2f, y=%.2f'
            % (x, y)
        )



    def blacklist_current_goal(self):

        if self.current_goal is not None:

            self.failed_frontiers.append(
                (self.current_goal[0], self.current_goal[1], time.monotonic())
            )

            self.get_logger().info(
                'Blacklisted frontier: x=%.2f, y=%.2f'
                % self.current_goal
            )


    def scan_callback(self, msg):
        #keep the latest laser scan
        self.last_scan = msg

        # bump guard: only while driving and not already backing off
        if self.backoff_until is not None:
            return
        if not (self.goal_active or self.spinning or self.recovering):
            return
        now = time.monotonic()
        if now - self.last_guard_time < 5.0:
            return
        extent = self.body_extents(msg)
        closest_gap = None
        closest_angle = 0.0
        gaps = []
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if math.isfinite(r) and r > msg.range_min:
                gap = r - extent[i]
            else:
                gap = float('inf')
            gaps.append((angle, gap))
            if closest_gap is None or gap < closest_gap:
                closest_gap, closest_angle = gap, angle
        if closest_gap is None or closest_gap >= self.guard_margin:
            return

        # the robot can only move straight forward or straight back: pick
        # the one away from the obstacle, if a 70 deg cone that way is clear
        def way_clear(heading):
            for angle, gap in gaps:
                diff = abs(math.atan2(math.sin(angle - heading), math.cos(angle - heading)))
                if diff < math.radians(35) and gap < self.guard_clear:
                    return False
            return True
        order = [math.pi, 0.0] if math.cos(closest_angle) > 0 else [0.0, math.pi]
        self.backoff_vx = 0.0
        for heading in order:
            if way_clear(heading):
                self.backoff_vx = self.backoff_speed if heading == 0.0 else -self.backoff_speed
                break
        if self.backoff_vx == 0.0:
            # neither way fully clear: use the roomier of the two if it still
            # has at least 0.10 m free (the back-off stops early if it closes)
            def room(heading):
                return min([gap for angle, gap in gaps
                            if abs(math.atan2(math.sin(angle - heading),
                                              math.cos(angle - heading))) < math.radians(35)]
                           or [0.0])
            best = max((0.0, math.pi), key=room)
            if room(best) > 0.10:
                self.backoff_vx = self.backoff_speed if best == 0.0 else -self.backoff_speed

        # a repeat stop at the same spot is about where the robot is, not
        # about the goal, so only blacklist the goal the first time
        position = self.get_robot_position()
        repeat = (position is not None and self.last_guard_position is not None
                  and math.hypot(position[0] - self.last_guard_position[0],
                                 position[1] - self.last_guard_position[1]) < 0.3)
        self.last_guard_position = position

        self.last_guard_time = now
        self.get_logger().warning(
            'Bump guard: obstacle %.2f m from the body at %.0f deg - stopping and %s'
            % (closest_gap, math.degrees(closest_angle),
               'moving forward' if self.backoff_vx > 0 else
               ('backing up' if self.backoff_vx < 0 else 'holding still')))
        self.cancel_navigation()
        if not repeat:
            self.blacklist_current_goal()
        self.goal_active = False
        self.current_goal = None
        self.spinning = False
        self.nudge_goal = None
        self.recovering = False
        self.recovery_start_time = None
        self.last_robot_position = None
        self.last_movement_time = None
        self.backoff_start = now + 0.3           # give Nav2 time to stop
        self.backoff_until = self.backoff_start + self.backoff_time


    def body_extents(self, msg):
        # distance from the lidar to the edge of the waffle_pi body along
        # every beam: 0.265 m square body centred on the lidar, wheels out to
        # 0.153 m at x = 0.031..0.097 (lidar frame). Worked out once
        key = (len(msg.ranges), round(msg.angle_min, 4), round(msg.angle_increment, 6))
        if key == self._extent_key:
            return self._extent
        extent = []
        for i in range(len(msg.ranges)):
            angle = msg.angle_min + i * msg.angle_increment
            c, s = math.cos(angle), math.sin(angle)
            t = 0.0
            while t < 0.3:
                x, y = c * t, s * t
                inside = ((abs(x) <= 0.1325 and abs(y) <= 0.1325) or
                          (0.031 <= x <= 0.097 and abs(y) <= 0.153))
                if not inside:
                    break
                t += 0.002
            extent.append(t)
        self._extent_key, self._extent = key, extent
        return extent


    def cancel_navigation(self):
        # cancel all Nav2 navigate_to_pose goals (an empty request = all)
        if self.cancel_client.service_is_ready():
            self.cancel_client.call_async(CancelGoal.Request())
            return
        # fallback: a goal at the robot's own pose and heading
        position = self.get_robot_position()
        if position is not None:
            yaw = self.get_robot_yaw()
            stop = PoseStamped()
            stop.header.frame_id = 'map'
            stop.header.stamp = self.get_clock().now().to_msg()
            stop.pose.position.x = position[0]
            stop.pose.position.y = position[1]
            stop.pose.orientation.z = math.sin(yaw / 2.0)
            stop.pose.orientation.w = math.cos(yaw / 2.0)
            self.publisher_.publish(stop)


    def backoff_step(self):
        # runs every 0.1 s: slow straight back-off after a bump-guard stop
        if self.backoff_until is None:
            return
        now = time.monotonic()
        if now < self.backoff_start:
            return
        twist = Twist()
        if now < self.backoff_until:
            # stop early if the way we are moving closes up
            if self.backoff_vx != 0.0 and self.last_scan is not None:
                scan = self.last_scan
                extent = self.body_extents(scan)
                heading = 0.0 if self.backoff_vx > 0 else math.pi
                for i, r in enumerate(scan.ranges):
                    angle = scan.angle_min + i * scan.angle_increment
                    diff = abs(math.atan2(math.sin(angle - heading), math.cos(angle - heading)))
                    if (diff < math.radians(35) and math.isfinite(r)
                            and r > scan.range_min and r - extent[i] < 0.05):
                        self.backoff_vx = 0.0
                        break
            twist.linear.x = self.backoff_vx
            self.cmd_pub.publish(twist)
            return
        self.cmd_pub.publish(twist)            # zero velocity
        self.backoff_until = None
        self.backoff_start = None
        self.get_logger().info('Bump guard: clear - choosing a new frontier')
        self.no_frontier_since = time.monotonic()
        self.send_waypoint()


    def laser_points(self):
        # obstacles the laser sees, in the robot frame
        scan = self.last_scan
        if scan is None:
            return None
        points = []
        for i, r in enumerate(scan.ranges):
            if math.isfinite(r) and scan.range_min < r < scan.range_max:
                b = scan.angle_min + i * scan.angle_increment
                points.append((r * math.cos(b) - 0.064, r * math.sin(b)))
        return points


    def nearest_obstacle(self):
        #distance from base_link to the closest laser return
        points = self.laser_points()
        if not points:
            return 0.0 if points is None else 9.0
        return min(math.hypot(px, py) for px, py in points)


    def start_nudge(self, position, toward):
        #short drive so SLAM adds a scan. The direction
        # must be clear in the laser: end point 0.35 m from every return and
        # the straight line to it 0.25 m. 'toward' = preferred direction
        points = self.laser_points()
        if points is None:
            return False
        yaw = self.get_robot_yaw()

        # do not drive straight back the way the last drive came; if it did
        # not get anywhere, do not repeat its direction either
        avoid = []
        if self.last_nudge_angle is not None and toward is None:
            avoid.append(self.last_nudge_angle + math.pi)
            if (self.last_nudge_from is not None and math.hypot(
                    position[0] - self.last_nudge_from[0],
                    position[1] - self.last_nudge_from[1]) < 0.2):
                avoid.append(self.last_nudge_angle)

        # The lock applies only to exploration nudges with an active sector,
        # not emergency bump-guard backing. If the sector has had no frontiers
        # for the grace period, permit an escape instead of trapping the robot.
        coverage = self.sector_coverage(self.active_sector)
        locked = (self.active_sector is not None and
                  coverage is not None and
                  coverage < self.sector_target_coverage and
                  self.frontier_sector(*position) == self.active_sector and
                  (self.sector_empty_since is None or
                   time.monotonic() - self.sector_empty_since <
                   self.sector_empty_grace))

        best = None
        # try a long drive first (fast progress along corridors), then the
        # usual 0.8 m and 0.6 m. Every length is checked against the laser
        for distance in (self.long_nudge_distance, self.nudge_distance, 0.6):
            for k in range(36):
                world_a = yaw - math.pi + k * (2.0 * math.pi / 36)
                if any(abs(math.atan2(math.sin(world_a - b), math.cos(world_a - b))) < 0.6
                       for b in avoid):
                    continue
                if toward is not None and abs(math.atan2(
                        math.sin(world_a - toward), math.cos(world_a - toward))) > math.radians(60):
                    continue
                a = world_a - yaw
                if not self.drive_is_clear(points, distance * math.cos(a), distance * math.sin(a)):
                    continue
                x = position[0] + distance * math.cos(world_a)
                y = position[1] + distance * math.sin(world_a)
                if locked and self.frontier_sector(x, y) != self.active_sector:
                    continue
                if self.wall_distance(x, y) < 0.35:
                    continue
                score = self.wall_distance(x, y)
                if toward is not None:
                    score += math.cos(world_a - toward)
                if best is None or score > best[0]:
                    best = (score, x, y, world_a)
            if best is not None:
                break
        if best is None and avoid:
            # the only open way is back the way the last drive came (e.g. a
            # dead end): allow it rather than getting stuck turning on the spot
            self.last_nudge_angle = None
            return self.start_nudge(position, toward)
        if best is None:
            return False

        score, x, y, angle = best
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = math.sin(angle / 2.0)
        goal.pose.orientation.w = math.cos(angle / 2.0)
        self.publisher_.publish(goal)

        self.last_nudge_angle = angle
        self.last_nudge_from = position
        self.nudge_goal = (x, y)
        # same state as the turn on the spot: wait spin_duration, then
        # choose a frontier again
        self.spinning = True
        self.spin_start_time = time.monotonic()
        self.get_logger().warning(
            'Driving %.1f m to (%.2f, %.2f) so SLAM adds a new scan'
            % (math.hypot(x - position[0], y - position[1]), x, y))
        return True


    def drive_is_clear(self, points, tx, ty):
        # True if no laser point is within 0.35 m of the end point
        #or within 0.25 m of the straight line to it (robot frame).
        # Points level with or behind the robot are ignored for the line
        length2 = tx * tx + ty * ty
        for px, py in points:
            if (px - tx) ** 2 + (py - ty) ** 2 < 0.35 ** 2:
                return False
            t = (px * tx + py * ty) / length2
            if t <= 0.0:
                continue
            t = min(t, 1.0)
            if (px - t * tx) ** 2 + (py - t * ty) ** 2 < 0.25 ** 2:
                return False
        return True


    def wall_distance(self, x, y, max_distance=0.5):
        # distance to the nearest wall cell in the map
        if self.map_msg is None:
            return max_distance
        info = self.map_msg.info
        resolution = info.resolution
        grid_x = int((x - info.origin.position.x) / resolution)
        grid_y = int((y - info.origin.position.y) / resolution)
        radius = int(max_distance / resolution)
        best = max_distance
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                nx = grid_x + dx
                ny = grid_y + dy
                if not (0 <= nx < info.width and 0 <= ny < info.height):
                    continue
                if self.map_msg.data[ny * info.width + nx] > 50:
                    best = min(best, math.hypot(dx, dy) * resolution)
        return best


    def publish_frontier_markers(self):
        markers = MarkerArray()

        # Clear old markers before drawing the new candidates
        clear = Marker()
        clear.header.frame_id = 'map'
        clear.header.stamp = self.get_clock().now().to_msg()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for i, (x, y, size) in enumerate(self.frontiers):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()

            marker.ns = 'frontiers'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.12
            marker.scale.y = 0.12
            marker.scale.z = 0.12

            # Blue = detected frontier
            marker.color.r = 0.0
            marker.color.g = 0.4
            marker.color.b = 1.0
            marker.color.a = 0.9

            markers.markers.append(marker)

        self.frontier_marker_pub.publish(markers)


    def publish_active_goal_marker(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'active_goal'
        marker.id = 0

        if self.current_goal is None or not self.goal_active:
            marker.action = Marker.DELETE
        else:
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = float(self.current_goal[0])
            marker.pose.position.y = float(self.current_goal[1])
            marker.pose.position.z = 0.2
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.scale.z = 0.25

            # Green = active goal
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

        self.active_goal_marker_pub.publish(marker)


    
def main(args=None):
    rclpy.init(args=args)

    # Create a new WaypointCycler node
    frontier_navigator = FrontierNavigator()

    # Execute the node
    rclpy.spin(frontier_navigator)


if __name__ == '__main__':
    main()
