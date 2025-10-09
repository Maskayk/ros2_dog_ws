#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
import time

class DogLegMover(Node):
    def __init__(self):
        super().__init__('dog_leg_mover')

        # Список всех суставов
        self.joints = [
            'front_left_hip_joint',
            'front_left_thigh_joint',
            'front_left_shin_joint',
            'front_right_hip_joint',
            'front_right_thigh_joint',
            'front_right_shin_joint',
            'rear_left_hip_joint',
            'rear_left_thigh_joint',
            'rear_left_shin_joint',
            'rear_right_hip_joint',
            'rear_right_thigh_joint',
            'rear_right_shin_joint'
        ]

        # Создаем publishers для каждого сустава
        self.publishers = {}
        for joint in self.joints:
            self.publishers[joint] = self.create_publisher(Float64, f'/{joint}/command', 10)

        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz
        self.t = 0.0

    def timer_callback(self):
        # Простая синусоидальная волна для движения лап
        for i, joint in enumerate(self.joints):
            angle = 0.5 * math.sin(self.t + i)  # амплитуда 0.5 рад
            msg = Float64()
            msg.data = angle
            self.publishers[joint].publish(msg)
        self.t += 0.02

def main(args=None):
    rclpy.init(args=args)
    node = DogLegMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
