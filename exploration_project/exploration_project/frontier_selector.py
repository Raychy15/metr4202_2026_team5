import rclpy
from rclpy.node import Node

#from nav2_msgs.msg import BehaviorTreeLog
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray



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

        # Keep track of the number of waypoints sent
        self.waypoint_counter = 0

        # Store the most recent frontier candidates
        self.frontiers = []
        self.failed_frontiers = []

        # Track whether a goal was already set
        self.goal_active = False

        # Current goal
        self.current_goal = None

        # Keep track of when current goal was sent
        self.goal_start_time = None

        # Maximum time allowed for one frontier
        self.goal_timeout = 30.0

         # Check for a stuck goal once per second
        self.timer = self.create_timer(
            1.0,
            self.check_goal_timeout
        )


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

        # Send a goal if one is not already active
        if not self.goal_active and len(self.frontiers) > 0:
            self.send_waypoint()


    #might change later depending on how tests go
    def choose_frontier(self):
        # Sort all available frontiers by size (largest first)
        sorted_frontiers = sorted(self.frontiers, key=lambda f: f[2], reverse=True)
        
        for frontier in sorted_frontiers:

            x, y, size = frontier

            failed = False
            #need to check frontiers close to failed ones to try prevent loop
            for failed_x, failed_y in self.failed_frontiers:

                distance = (
                    (x - failed_x) ** 2
                    + (y - failed_y) ** 2
                ) ** 0.5

                #ignore anything within 0.5m of previous failed fronter
                if distance < 0.5:
                    failed = True
                    break

            #use frontier that has not failed
            if not failed:
                return frontier

        return None


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

        self.publisher_.publish(goal)

        self.goal_active = True
        self.current_goal = (x, y)

        # Store the time the goal was sent
        self.goal_start_time = self.get_clock().now()

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

        # Nothing to check if there is no active goal
        if not self.goal_active:
            return

        if self.goal_start_time is None:
            return

        elapsed = (
            self.get_clock().now() -
            self.goal_start_time
        ).nanoseconds / 1e9

        # Robot has spent too long trying the same frontier
        if elapsed > self.goal_timeout:

            self.get_logger().info(
                'Frontier timed out - blacklisting it'
            )

            self.blacklist_current_goal()

            self.goal_active = False
            self.current_goal = None
            self.goal_start_time = None

            # # Try a different frontier
            # if len(self.frontiers) > 0:
            #     self.send_waypoint()



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