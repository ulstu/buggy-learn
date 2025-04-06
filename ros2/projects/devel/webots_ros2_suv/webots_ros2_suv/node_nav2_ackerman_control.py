#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Twist
import sys, select, termios, tty
from numpy import clip
import traceback
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


control_keys_bindings = {
    '\x77': ['w', (0.1, 0.0)], 
    '\x61': ['a', (0.0, -0.05)], 
    '\x73': ['s', (-0.1, 0.0)], 
    '\x64': ['d', (0.0, 0.05)], 
    '\x20': ['space', (0.0, 0.0)], 
    '\x09': ['tab', (0.0, 0.0)], 
}

ROBOT_MAX_SPEED = 0.5  # См/с
ROBOT_MAX_STEERING_ANGLE = 1.0  # Радианы


class NodeNav2AckermannControl(Node):
    def __init__(self):
        super().__init__('node_nav2_ackerman_control')

        self.current_speed = 0.0
        self.current_steering_angle = 0.0

        max_speed = ROBOT_MAX_SPEED
        max_steering_angle = ROBOT_MAX_STEERING_ANGLE

        self.speed_range = [-float(max_speed), float(max_speed)]
        self.steering_angle_range = [-float(max_steering_angle), float(max_steering_angle)]

        for key in control_keys_bindings:
            control_keys_bindings[key][1] = (
                control_keys_bindings[key][1][0] * float(max_speed) / 5, 
                control_keys_bindings[key][1][1] * float(max_steering_angle) / 5
            )

        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, 
            history=HistoryPolicy.KEEP_LAST, 
            depth=1, 
        )

        self.nav2_listener = self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self.nav2_listener_callback,
            10)

        self.message = AckermannDrive()
        self.cmd_ackermann_publisher = self.create_publisher(AckermannDrive, '/cmd_ackermann', cmd_qos)

    def nav2_listener_callback(self, msg):
        self.message.speed = msg.linear.x
        self.message.steering_angle = min(1.0, max(-1.0, -msg.angular.z * 50))
        self.cmd_ackermann_publisher.publish(self.message)


def main(args=None):
    try:
        rclpy.init(args=args)

        node = NodeNav2AckermannControl()
        rclpy.spin(node)
        node.destroy_node()

        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(''.join(traceback.TracebackException.from_exception(e).format()))
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
