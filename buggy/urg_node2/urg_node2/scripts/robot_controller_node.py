#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg  import Twist
from rclpy.node import Node
import traceback
import threading
import socket
import json
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import Imu
import time


UDP_SERVER_IP = '192.168.2.11'
UDP_SERVER_PORT = 9090
RECV_BUFFER_SIZE = 1024


class RobotControllerNode(Node):
    def __init__(self):
        try:
            super().__init__('robot_controller_node')

            self.__imu_message = Imu()
            self.__imu_message.header.frame_id = 'base_link'

            self.__last_cmd_vel_callback_time = None

            self.__drive_command = AckermannDrive()
            self.__sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            self.__imu_publisher = self.create_publisher(Imu, '/imu', 10)
            threading.Thread(target=self.__udp_data_receiver).start()

            self.create_subscription(Twist, '/cmd_vel', self.__cmd_vel_callback, 1)
            self.create_subscription(AckermannDrive, '/cmd_ackermann', self.__cmd_ackermann_callback, 1)

            self._logger.info('Successfully launched!')
        except Exception as e:
            self._logger.error(''.join(traceback.TracebackException.from_exception(e).format()))

    def __udp_data_receiver(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('0.0.0.0', UDP_SERVER_PORT))

        while True:
            current_step_time = time.time()

            if self.__last_cmd_vel_callback_time is not None and current_step_time - self.__last_cmd_vel_callback_time > 3:
                self.__stop()

            data = sock.recv(RECV_BUFFER_SIZE)

            if data:
                data_dict = json.loads(data)

                self.__imu_message.header.stamp.sec = data_dict['params'][0]['header']['stamp']['secs']
                self.__imu_message.header.stamp.nanosec = data_dict['params'][0]['header']['stamp']['nsecs']

                self.__imu_message.linear_acceleration.x = data_dict['params'][0]['linear_acceleration']['x']
                self.__imu_message.linear_acceleration.y = data_dict['params'][0]['linear_acceleration']['y']
                self.__imu_message.linear_acceleration.z = data_dict['params'][0]['linear_acceleration']['z']
                # self.__imu_message.linear_acceleration_covariance = data_dict['params'][0]['linear_acceleration_covariance']

                self.__imu_message.angular_velocity.x = data_dict['params'][0]['angular_velocity']['x']
                self.__imu_message.angular_velocity.y = data_dict['params'][0]['angular_velocity']['y']
                self.__imu_message.angular_velocity.z = data_dict['params'][0]['angular_velocity']['z']
                # self.__imu_message.angular_velocity_covariance = data_dict['params'][0]['angular_velocity_covariance']

                self.__imu_message.orientation.x = data_dict['params'][0]['orientation']['x']
                self.__imu_message.orientation.y = data_dict['params'][0]['orientation']['y']
                self.__imu_message.orientation.z = data_dict['params'][0]['orientation']['z']
                self.__imu_message.orientation.w = data_dict['params'][0]['orientation']['w']
                # self.__imu_message.orientation_covariance = data_dict['params'][0]['orientation_covariance']

                self.__imu_publisher.publish(self.__imu_message)

    def __stop(self):
        self.__drive_command.speed = 0.0
        self.__drive_command.steering_angle = 0.0

        self.__cmd_ackermann_callback(self.__drive_command)

    def __cmd_vel_callback(self, message):
        self.__last_cmd_vel_callback_time = time.time()

        self.__drive_command.speed = message.linear.x * 17.5
        self.__drive_command.steering_angle = -message.angular.z * 500

        if self.__drive_command.speed > 0:
            self.__drive_command.speed = 8.0

        if self.__drive_command.steering_angle < -30.0:
            self.__drive_command.steering_angle = -30.0
        elif self.__drive_command.steering_angle > 30.0:
            self.__drive_command.steering_angle = 30.0

        self.__cmd_ackermann_callback(self.__drive_command)

    def __cmd_ackermann_callback(self, message):
        speed = message.speed
        steering_angle = message.steering_angle

        jsonrpc_message = {
            'jsonrpc': '2.0', 
            'method': 'set_control_values', 
            'params': [speed, steering_angle], 
        }

        jsonrpc_message_str = json.dumps(jsonrpc_message)
        self.__sock.sendto(jsonrpc_message_str.encode(), (UDP_SERVER_IP, UDP_SERVER_PORT))


def main(args=None):
    try:
        rclpy.init(args=args)
        node = RobotControllerNode()
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(''.join(traceback.TracebackException.from_exception(e).format()))
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
