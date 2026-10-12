import rclpy
import time
import math



from rclpy.node import Node



#from nav2_msgs.msg import BehaviorTreeLog

from geometry_msgs.msg import PoseStamped

from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import OccupancyGrid

from tf2_ros import Buffer, TransformListener











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





        # Create a publisher
        # The constructor inputs are (message type, topic name, message queue
        # length)

        self.publisher_ = self.create_publisher(
            PoseStamped,
            'goal_pose',
            10)

        # Keep track of the number of waypoints sent

        self.waypoint_counter = 0

        # Store the most recent frontier candidates

        self.frontiers = []

        self.failed_frontiers = []
        self.completed_frontiers = []
        self.blacklist_duration = 120.0
        self.map_msg = None
        self.map_subscription = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 10
        )

        # Track whether a goal was already set
        self.goal_active = False
        self.current_goal = None
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
        self.last_robot_yaw = None
        self.minimum_rotation = math.radians(10)
        self.goal_tolerance = 0.30

        # Minimum movement needed to count as moving
        self.minimum_movement = 0.15

        # How long robot can stay in same area before recovery
        self.stuck_timeout = 8.0



         # Check for a stuck goal once per second
        self.timer = self.create_timer(
            1.0,
            self.check_goal_timeout
        )





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





    def map_callback(self, msg):
        self.map_msg = msg

    def get_robot_yaw(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
            q = transform.transform.rotation
            return math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
        except Exception:
            return None

    def safe_goal_near(self, x, y):
        # Find a known, obstacle-clear map cell near the frontier.
        # If no map is available yet, don't send an unchecked goal.
        msg = self.map_msg
        if msg is None:
            return None
        info = msg.info
        res = info.resolution
        if res <= 0:
            return None
        ox, oy = info.origin.position.x, info.origin.position.y
        q = info.origin.orientation
        theta = math.atan2(2 * (q.w * q.z + q.x * q.y),
                           1 - 2 * (q.y * q.y + q.z * q.z))
        c, sn = math.cos(theta), math.sin(theta)
        dx, dy = x - ox, y - oy
        gx = int(math.floor((c * dx + sn * dy) / res))
        gy = int(math.floor((-sn * dx + c * dy) / res))
        w, h = info.width, info.height
        cells = msg.data
        clearance = max(1, math.ceil(0.20 / res))
        radius = max(1, math.ceil(0.70 / res))
        candidates = []
        for iy in range(max(clearance, gy - radius),
                        min(h - clearance, gy + radius + 1)):
            for ix in range(max(clearance, gx - radius),
                            min(w - clearance, gx + radius + 1)):
                if (ix - gx)**2 + (iy - gy)**2 > radius**2:
                    continue
                safe = True
                for yy in range(iy - clearance, iy + clearance + 1):
                    for xx in range(ix - clearance, ix + clearance + 1):
                        if (xx - ix)**2 + (yy - iy)**2 > clearance**2:
                            continue
                        value = cells[yy * w + xx]
                        if value < 0 or value > 20:
                            safe = False
                            break
                    if not safe:
                        break
                if safe:
                    mx, my = (ix + 0.5) * res, (iy + 0.5) * res
                    wx = ox + c * mx - sn * my
                    wy = oy + sn * mx + c * my
                    candidates.append(((wx-x)**2 + (wy-y)**2, wx, wy))
        if not candidates:
            return None
        _, wx, wy = min(candidates)
        return wx, wy

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



        # Send a goal if one is not already active and robot is not in recovery
        if (not self.goal_active and not self.recovering and len(self.frontiers) > 0):
            self.send_waypoint()





    #might change later depending on how tests go

    def choose_frontier(self):
        if not self.frontiers:
            return None

        robot_position = self.get_robot_position()
        if robot_position is None:
            self.get_logger().warning('Robot position unavailable')
            return None

        now = time.monotonic()
        self.failed_frontiers = [
            (fx, fy, until) for fx, fy, until in self.failed_frontiers
            if until > now
        ]
        robot_x, robot_y = robot_position
        best_frontier = None
        best_score = float('-inf')
        safe_count = 0

        for x, y, size in self.frontiers:
            if any(math.hypot(x-fx, y-fy) < 0.65
                   for fx, fy, until in self.failed_frontiers):
                continue
            if any(math.hypot(x-vx, y-vy) < 0.60
                   for vx, vy in self.completed_frontiers):
                continue

            safe_goal = self.safe_goal_near(x, y)
            if safe_goal is None:
                continue
            safe_count += 1
            goal_x, goal_y = safe_goal
            distance = math.hypot(goal_x-robot_x, goal_y-robot_y)
            # Prefer useful nearby frontiers without returning to visited ones.
            score = size / (distance + 1.0)
            if score > best_score:
                best_score = score
                best_frontier = (goal_x, goal_y, size)

        self.get_logger().info(
            f'Frontiers: {len(self.frontiers)} received, '
            f'{safe_count} safe/unvisited candidates, '
            f'{len(self.failed_frontiers)} temporarily blacklisted'
        )
        if best_frontier is not None:
            self.get_logger().info(
                f'Selected frontier: {best_frontier}, score: {best_score:.2f}'
            )
        return best_frontier


    def send_waypoint(self):

        if len(self.frontiers) == 0:
            return



        # Keep track of the number of waypoints sent
        self.waypoint_counter += 1

        # Choose which frontier to visit
        frontier = self.choose_frontier()



        if frontier is None:
            self.get_logger().info('No suitable frontier')
            return



        x, y, size = frontier

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

            return



        # Save position before attempting frontier

        self.previous_position = self.robot_position



        # Publish frontier goal

        self.publisher_.publish(goal)



        self.goal_active = True

        self.current_goal = (x, y)



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



        # Check whether recovery period has finished

        if self.recovering:



            current_position = self.get_robot_position()



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

                    'Recovery timed out - selecting another frontier'

                )

                self.recovering = False
                self.recovery_start_time = None
                self.goal_active = False
                self.current_goal = None
                self.last_robot_position = None
                self.last_robot_yaw = None
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

        # Mark reached destinations as visited so we do not revisit them.
        if self.current_goal is not None:
            gx, gy = self.current_goal
            if math.hypot(current_position[0]-gx, current_position[1]-gy) <= self.goal_tolerance:
                self.get_logger().info('Frontier reached successfully')
                self.completed_frontiers.append(self.current_goal)
                self.goal_active = False
                self.current_goal = None
                self.last_robot_position = None
                self.last_robot_yaw = None
                self.last_movement_time = None
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





        # Turning in place is progress too.
        current_yaw = self.get_robot_yaw()
        rotation_moved = 0.0
        if current_yaw is not None and self.last_robot_yaw is not None:
            difference = current_yaw - self.last_robot_yaw
            rotation_moved = abs(math.atan2(math.sin(difference), math.cos(difference)))

        # Robot has moved enough

        if (distance_moved >= self.minimum_movement
                or rotation_moved >= self.minimum_rotation):

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



            # Return towards previous safe position
            self.start_recovery()



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
                self.current_goal
            )

            self.get_logger().info(
                'Blacklisted frontier: x=%.2f, y=%.2f'
                % self.current_goal
            )



def main(args=None):

    rclpy.init(args=args)



    # Create a new WaypointCycler node
    frontier_navigator = FrontierNavigator()



    # Execute the node
    rclpy.spin(frontier_navigator)





if __name__ == '__main__':
    main()
