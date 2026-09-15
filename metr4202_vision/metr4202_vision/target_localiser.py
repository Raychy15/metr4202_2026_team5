#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from ros2_aruco_interfaces.msg import ArucoMarkers
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
import tf2_geometry_msgs

class TargetLocalizer(Node):
    def __init__(self):
        super().__init__('target_localizer')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.create_subscription(ArucoMarkers, '/aruco_markers', self.pose_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/detected_targets', 10)

    def pose_callback(self, msg):
        marker_array = MarkerArray()
        for pose, marker_id in zip(msg.poses, msg.marker_ids):
            try:
                # Transform from camera to odom frame
                transform = self.tf_buffer.lookup_transform('map', msg.header.frame_id, rclpy.time.Time())
                
                # FIX 1: Pass the raw 'pose' directly into the transform
                global_pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
                
                # Format for RViz Visualization
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.ns, marker.id, marker.type = 'aruco', marker_id, Marker.CUBE
                
                # FIX 2: Assign the transformed pose directly (no .pose needed)
                marker.pose = global_pose 
                marker.scale.x = marker.scale.y = 0.1 # 100mm target size
                marker.scale.z = 0.01
                marker.color.g = marker.color.a = 1.0 # Solid Green
                marker_array.markers.append(marker)
                
                # FIX 3: Read the coordinates from the raw pose
                self.get_logger().info(f'Target {marker_id} mapped at X:{global_pose.position.x:.2f} Y:{global_pose.position.y:.2f}')
                
            except TransformException as e:
                self.get_logger().warn(f'Missing TF tree: {e}')
        
        if marker_array.markers:
            self.marker_pub.publish(marker_array)
            
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TargetLocalizer())
    rclpy.shutdown()

if __name__ == '__main__':
    main()