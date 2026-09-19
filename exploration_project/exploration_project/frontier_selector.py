import rclpy
from rclpy.node import Node

#from nav2_msgs.msg import BehaviorTreeLog
#from geometry_msgs.msg import PoseStamped
#from std_msgs.msg import Float32MultiArray

from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
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

        # Use Nav2 NavigateToPose action instead of publishing /goal_pose
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
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
        # self.publisher_ = self.create_publisher(
        #     PoseStamped,
        #     'goal_pose',
        #     10)

        # Keep track of the number of waypoints sent
        self.waypoint_counter = 0

        # Store the most recent frontier candidates
        self.frontiers = []
        self.failed_frontiers = []

        # Track whether a goal was already set
        self.goal_active = False

        # Current goal
        self.current_goal = None

        # # Manually input the two waypoints
        # # Best practice would be to put these in a config file that's read in
        # # as params
        # p0 = PoseStamped()
        # p0.header.frame_id = 'map'
        # p0.pose.position.x = 1.7
        # p0.pose.position.y = -0.5
        # p0.pose.orientation.w = 1.0

        # p1 = PoseStamped()
        # p1.header.frame_id = 'map'
        # p1.pose.position.x = -0.6
        # p1.pose.position.y = 1.8
        # p1.pose.orientation.w = 1.0

        # # Store the waypoints as a member variable
        # self.waypoints = [p0, p1]

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

        # goal = PoseStamped()

        # goal.header.frame_id = 'map'
        # goal.header.stamp = self.get_clock().now().to_msg()

        # goal.pose.position.x = x
        # goal.pose.position.y = y

        # # No particular heading required
        # goal.pose.orientation.w = 1.0

        #self.publisher_.publish(goal)

        self.goal_active = True
        self.current_goal = (x, y)

        # Create Nav2 goal
        goal = NavigateToPose.Goal()

        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y

        # No particular heading required
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            'Sent frontier goal: '
            'x=%.2f, y=%.2f, size=%.0f'
            % (x, y, size)
        )

        # Wait for Nav2 action server
        if not self.nav_client.wait_for_server(timeout_sec=2.0):

            self.get_logger().info(
                'Nav2 action server not available'
            )

            self.goal_active = False
            return

        # Send goal to Nav2
        future = self.nav_client.send_goal_async(goal)

        future.add_done_callback(
            self.goal_response_callback
        )


    def goal_response_callback(self, future):

        goal_handle = future.result()

        # Nav2 rejected the goal
        if not goal_handle.accepted:

            self.get_logger().info(
                'Frontier goal was rejected'
            )

            self.blacklist_current_goal()

            self.goal_active = False
            self.current_goal = None
            self.send_waypoint()

            return

        self.get_logger().info(
            'Frontier goal accepted'
        )

        # Wait for navigation to finish
        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
        self.navigation_result_callback
        )


    def navigation_result_callback(self, future):

        result = future.result()

        # ROS action status:
        # 4 = succeeded
        # 5 = cancelled
        # 6 = aborted

        if result.status == 4:

            self.get_logger().info(
                'Frontier reached successfully'
            )

        else:

            self.get_logger().info(
                'Could not reach frontier - blacklisting it'
            )

            self.blacklist_current_goal()

        # Robot is no longer navigating
        self.goal_active = False
        self.current_goal = None

        # Try the next available frontier
        if len(self.frontiers) > 0:
            self.send_waypoint()


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