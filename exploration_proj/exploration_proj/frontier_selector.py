import rclpy
from rclpy.node import Node

from nav2_msgs.msg import BehaviorTreeLog
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
        self.subscription = self.create_subscription(
            BehaviorTreeLog,
            'behavior_tree_log',
            self.bt_log_callback,
            10)
        self.subscription  # prevent unused variable warning

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



    def bt_log_callback(self, msg: BehaviorTreeLog):
        # Check the current state of the behaviour tree
        # When the robot has reached its waypoint, the top level node state
        # is 'NavigateRecovery' and the event status is 'IDLE'
        # If the behaviour tree is in this state, send the next waypoint
        for event in msg.event_log:
            if (event.node_name == 'NavigateRecovery'
                    and event.current_status == 'IDLE'):
                # Current navigation done
                self.goal_active = False

                # Send another frontier if available
                if len(self.frontiers) > 0:
                    self.send_waypoint()

    #might change later depending on how tests go
    def choose_frontier(self):

        # choose the largest frontier cluster

        chosen_frontier = self.frontiers[0]

        for frontier in self.frontiers:

            if frontier[2] > chosen_frontier[2]:
                chosen_frontier = frontier

        return chosen_frontier


    def send_waypoint(self):

        if len(self.frontiers) == 0:
            return
        
        # Keep track of the number of waypoints sent
        self.waypoint_counter += 1

        # Choose which frontier to visit
        x, y, size = self.choose_frontier()     

        goal = PoseStamped()

        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()

        goal.pose.position.x = x
        goal.pose.position.y = y

        # No particular heading required
        goal.pose.orientation.w = 1.0

        self.publisher_.publish(goal)

        self.goal_active = True
        self.current_goal = (x, y)

        self.get_logger().info(
            'Sent frontier goal: '
            'x=%.2f, y=%.2f, size=%.0f'
            % (x, y, size)
        )


def main(args=None):
    rclpy.init(args=args)

    # Create a new WaypointCycler node
    frontier_navigator = FrontierNavigator()

    # Execute the node
    rclpy.spin(frontier_navigator)


if __name__ == '__main__':
    main()