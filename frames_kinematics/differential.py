#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
import math
import tf_transformations


class DiffDriveTF(Node):
    def __init__(self):
        super().__init__('diffdrive_tf')

        # Robot parameters
        self.declare_parameter('wheel_radius', 0.05)     # wheel radius [m]
        self.declare_parameter('track', 0.3)             # distance between wheels [m]
        self.declare_parameter('dt', 0.05)

        self.r = self.get_parameter('wheel_radius').value
        self.track = self.get_parameter('track').value
        self.dt = self.get_parameter('dt').value

        # Subscribers and TF broadcaster
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.br = TransformBroadcaster(self)

        # Robot pose (in "map" frame)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Wheel angular states (for animation)
        self.theta_l = 0.0
        self.theta_r = 0.0

        # Commanded velocities
        self.v = 0.0     # forward velocity
        self.omega = 0.0 # angular velocity (yaw rate)

        # Periodic update
        self.timer = self.create_timer(self.dt, self.update)

    def cmd_callback(self, msg: Twist):
        self.v = msg.linear.x
        self.omega = msg.angular.z

    def update(self):
        # Differential drive kinematics
        v_l = self.v - (self.track / 2.0) * self.omega
        v_r = self.v + (self.track / 2.0) * self.omega

        # Integrate pose
        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt
        self.yaw += self.omega * self.dt

        now = self.get_clock().now().to_msg()

        # --- base_link ---
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        q = tf_transformations.quaternion_from_euler(0, 0, self.yaw)
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        self.br.sendTransform(t)

        # --- left & right wheels ---
        w_l = v_l / self.r
        w_r = v_r / self.r
        self.theta_l += w_l * self.dt
        self.theta_r += w_r * self.dt

        for name, yoff, theta in [
            ('left_wheel',  self.track/2, self.theta_l),
            ('right_wheel', -self.track/2, self.theta_r)
        ]:
            tw = TransformStamped()
            tw.header.stamp = now
            tw.header.frame_id = 'base_link'
            tw.child_frame_id = name
            tw.transform.translation.x = 0.0
            tw.transform.translation.y = yoff
            qroll = tf_transformations.quaternion_from_euler(0, theta, 0)  # wheel roll about y-axis
            tw.transform.rotation.x, tw.transform.rotation.y, tw.transform.rotation.z, tw.transform.rotation.w = qroll
            self.br.sendTransform(tw)


def main():
    rclpy.init()
    node = DiffDriveTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
