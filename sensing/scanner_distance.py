import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import numpy as np
import math

class LaserMarkerNode(Node):
    def __init__(self):
        super().__init__('laser_marker_node')

        # Subscribe to laser scan
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)

        # Publisher for RViz marker
        self.marker_pub = self.create_publisher(Marker, 'closest_obstacle_marker', 10)

    def listener_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)

        # Keep only finite values
        finite_indices = np.isfinite(ranges)
        if not np.any(finite_indices):
            self.get_logger().warn('No valid ranges!')
            return

        finite_ranges = ranges[finite_indices]
        min_index = np.argmin(finite_ranges)
        min_distance = finite_ranges[min_index]

        # Recover the actual index in the original array
        global_min_index = np.where(finite_indices)[0][min_index]

        # Compute angle of closest obstacle
        angle = msg.angle_min + global_min_index * msg.angle_increment

        # Convert polar to Cartesian (laser frame)
        x = min_distance * math.cos(angle)
        y = min_distance * math.sin(angle)

        # Print info
        self.get_logger().info(f'Closest obstacle: {min_distance:.2f} m at angle {math.degrees(angle):.1f}°')

        # Create a marker
        marker = Marker()
        marker.header.frame_id = msg.header.frame_id  # usually "laser" or "base_scan"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "closest_obstacle"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Publish marker
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = LaserMarkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
