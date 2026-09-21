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

        # Permanent memory for target markers
        self.saved_markers = []
        self.marker_id_counter = 0

    def pose_callback(self, msg):
        for pose, marker_id in zip(msg.poses, msg.marker_ids):
            try:
                # Transform from camera to map frame
                transform = self.tf_buffer.lookup_transform('map', msg.header.frame_id, rclpy.time.Time())
                global_pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
                
                # 1. Check if we already found this target (within 0.5 meters)
                is_duplicate = False
                for saved_marker in self.saved_markers:
                    dist = ((saved_marker.pose.position.x - global_pose.position.x)**2 + 
                            (saved_marker.pose.position.y - global_pose.position.y)**2)**0.5
                    if dist < 0.5:
                        is_duplicate = True
                        break
                
                # 2. If it is a new target, create and save it
                if not is_duplicate:
                    marker = Marker()
                    marker.header.frame_id = 'map'
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = 'aruco_targets'
                    marker.id = self.marker_id_counter
                    self.marker_id_counter += 1
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD
                    
                    marker.pose = global_pose 
                    marker.scale.x = marker.scale.y = 0.1 # 100mm target size
                    marker.scale.z = 0.01
                    marker.color.g = marker.color.a = 1.0 # Solid Green
                    
                    self.saved_markers.append(marker)
                    self.get_logger().info(f'New Target {marker_id} saved at X:{global_pose.position.x:.2f} Y:{global_pose.position.y:.2f}. Total targets: {len(self.saved_markers)}')
                    
            except TransformException as e:
                self.get_logger().warn(f'Missing TF tree: {e}')
        
        # 3. Always publish the full list of saved markers
        if self.saved_markers:
            marker_array_msg = MarkerArray()
            marker_array_msg.markers = self.saved_markers
            self.marker_pub.publish(marker_array_msg)
            
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TargetLocalizer())
    rclpy.shutdown()

if __name__ == '__main__':
    main()