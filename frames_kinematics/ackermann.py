#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import tf_transformations


class AckermannTF(Node):
    def __init__(self):
        super().__init__('ackermann_tf')
        self.declare_parameter('wheelbase', 0.3)        # front–rear axle distance [m]
        self.declare_parameter('track', 0.2)            # left–right wheel distance [m]
        self.declare_parameter('wheel_radius', 0.05)    # wheel radius [m]
        self.declare_parameter('dt', 0.05)
        self.declare_parameter('max_steer', math.radians(35.0))  # ±35° steering

        self.L = self.get_parameter('wheelbase').value
        self.track = self.get_parameter('track').value
        self.r = self.get_parameter('wheel_radius').value
        self.dt = self.get_parameter('dt').value
        self.max_steer = self.get_parameter('max_steer').value

        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.br = TransformBroadcaster(self)

        # robot pose
        self.x = self.y = self.yaw = 0.0
        self.v = 0.0
        self.delta = 0.0

        # wheel rolling states
        self.theta_rl = self.theta_rr = 0.0
        self.theta_fl = self.theta_fr = 0.0

        self.timer = self.create_timer(self.dt, self.update)

    def cmd_callback(self, msg: Twist):
        self.v = msg.linear.x
        requested = msg.angular.z
        self.delta = max(-self.max_steer, min(self.max_steer, requested))

    def update(self):
        # kinematics
        if abs(self.delta) > 1e-3:
            # todo
            R = 
            omega = 
        else:
            R, omega = float('inf'), 0.0

        # integrate pose
        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt
        self.yaw += omega * self.dt

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

        # --- rear wheels (roll only about y) ---
        w_rear = self.v / self.r
        self.theta_rl += w_rear * self.dt
        self.theta_rr += w_rear * self.dt

        for name, yoff, theta in [
            ('rear_left_wheel',  self.track/2, self.theta_rl),
            ('rear_right_wheel', -self.track/2, self.theta_rr)
        ]:
            tw = TransformStamped()
            tw.header.stamp = now
            tw.header.frame_id = 'base_link'
            tw.child_frame_id = name
            tw.transform.translation.x = 0.0
            tw.transform.translation.y = yoff
            qroll = tf_transformations.quaternion_from_euler(0, theta, 0)  # roll about y
            tw.transform.rotation.x, tw.transform.rotation.y, tw.transform.rotation.z, tw.transform.rotation.w = qroll
            self.br.sendTransform(tw)

        # --- front wheels (steering about z + roll about y) ---
        if abs(self.delta) > 1e-3:
            # path radii for left/right front wheels
            R_left = math.sqrt((R - self.track/2)**2 + self.L**2)
            R_right = math.sqrt((R + self.track/2)**2 + self.L**2)

            # use absolute omega, preserve direction from v
            v_left = abs(omega) * R_left * math.copysign(1.0, self.v)
            v_right = abs(omega) * R_right * math.copysign(1.0, self.v)
        else:
            v_left = v_right = self.v

        # todo

        w_left =
        w_right =
        self.theta_fl +=
        self.theta_fr +=

        for name, yoff, theta in [
            ('front_left_wheel',  self.track/2, self.theta_fl),
            ('front_right_wheel', -self.track/2, self.theta_fr)
        ]:
            tw = TransformStamped()
            tw.header.stamp = now
            tw.header.frame_id = 'base_link'
            tw.child_frame_id = name
            tw.transform.translation.x = self.L
            tw.transform.translation.y = yoff

            # steering about z (base frame)
            q_steer = tf_transformations.quaternion_from_euler(0, 0, self.delta)
            # rolling about y (local wheel axis)
            q_roll = tf_transformations.quaternion_from_euler(0, theta, 0)
            # combine: steer first, then roll in steered frame
            q_combined = tf_transformations.quaternion_multiply(q_steer, q_roll)

            tw.transform.rotation.x, tw.transform.rotation.y, tw.transform.rotation.z, tw.transform.rotation.w = q_combined
            self.br.sendTransform(tw)


def main():
    rclpy.init()
    node = AckermannTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
