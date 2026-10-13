import rclpy
import time
import math

from rclpy.node import Node

#from nav2_msgs.msg import BehaviorTreeLog
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid




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
        self.minimum_unknown_cells = 10

        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

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
        self.failed_frontiers = []

        self.visited_frontiers = []
        self.visited_radius = 0.5

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
        self.stuck_timeout = 20.0

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

        # Send a goal if one is not already active and robot is not in recovery
        if (not self.goal_active and not self.recovering and len(self.frontiers) > 0):
            self.send_waypoint()


    #might change later depending on how tests go
    def choose_frontier(self):
        
        if not self.frontiers:
            self.get_logger().warning('No frontier candidates received')
            return None

        robot_position = self.get_robot_position()

        # Keep only frontiers that are not blacklisted
        available_frontiers = []

        
        # Sort all available frontiers by size (largest first)
        #sorted_frontiers = sorted(self.frontiers, key=lambda f: f[2], reverse=True)
        
        for frontier in self.frontiers:

            x, y, size = frontier

            # Ignore frontiers near locations already visited
            already_visited = any(
                math.hypot(x - vx, y - vy) < self.visited_radius
                for vx, vy in self.visited_frontiers
            )

            if already_visited:
                self.get_logger().info(
                    f'Rejected frontier ({x:.2f}, {y:.2f}): already visited'
                )
                continue

            is_blacklisted = False

            for failed_x, failed_y in self.failed_frontiers:
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

        # NEW: Prefer nearby frontiers before travelling across the map
        local_radius = 2.0

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

            self.get_logger().info(
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
            f'Selected frontier: {best_frontier}, '
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
                self.visited_frontiers.append(self.current_goal)

                self.get_logger().info(
                    'Frontier reached - marking as visited'
                )

                self.goal_active = False
                self.current_goal = None
                self.last_robot_position = None
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

            # Return towards previous safe position
            #self.start_recovery()
            # Stop tracking the failed goal and allow a new frontier
            self.recovering = False
            self.recovery_start_time = None

            self.get_logger().info(
                'Navigation stalled - selecting a different frontier'
            )


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