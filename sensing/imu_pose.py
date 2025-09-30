import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped

class ImuToPose(Node):
    def __init__(self):
        super().__init__('imu_to_pose')
        self.sub = self.create_subscription(Imu, '/imu', self.callback, 10)
        self.pub = self.create_publisher(PoseStamped, '/imu_pose', 10)

    def callback(self, msg: Imu):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.orientation = msg.orientation
        self.pub.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    node = ImuToPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
