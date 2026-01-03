#!/usr/bin/env python3
"""
Простой скрипт: каждую 0.1s запрашивает сервис /gazebo/set_model_state
и сдвигает модель 'dog' по X, создавая прямолинейное движение.
Работает с классическим Gazebo (gzserver) и ros2-gazebo bridge.
"""

import rclpy
import time
from rclpy.node import Node

# Сервис/тип зависят от версии gazebo_ros:
from gazebo_msgs.srv import SetEntityState  # try this first
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.msg import EntityState

class MoveDog(Node):
    def __init__(self):
        super().__init__('move_dog_node')
        # Попробуем несколько возможных имён сервисов — возьмём первое доступное
        self.srv_name_candidates = [
            '/gazebo/set_entity_state',     # sometimes present
            '/gazebo/set_model_state',      # common in ROS1/ROS2 bridges
            '/set_model_state',             # fallback
        ]
        self.srv_client = None
        self.srv_name = None
        for name in self.srv_name_candidates:
            client = self.create_client(SetEntityState, name)
            if client.wait_for_service(timeout_sec=1.0):
                self.srv_client = client
                self.srv_name = name
                self.get_logger().info(f'Using gazebo service: {name}')
                break
            else:
                # destroy client and try next
                client.destroy()
        if self.srv_client is None:
            self.get_logger().error('No gazebo set_model/set_entity service found. Exiting.')
            raise SystemExit(1)

        self.timer = self.create_timer(0.1, self.tick)
        self.t = 0.0
        self.x = 0.0

    def tick(self):
        # increment forward position
        self.x += 0.01  # meters per tick: adjust to change speed
        # build request
        req = SetEntityState.Request()
        state = EntityState()
        state.name = 'dog'
        p = Pose()
        p.position.x = float(self.x)
        p.position.y = 0.0
        p.position.z = 0.0
        p.orientation.x = 0.0
        p.orientation.y = 0.0
        p.orientation.z = 0.0
        p.orientation.w = 1.0
        state.pose = p
        # zero twist
        t = Twist()
        t.linear.x = 0.0
        t.linear.y = 0.0
        t.linear.z = 0.0
        t.angular.x = 0.0
        t.angular.y = 0.0
        t.angular.z = 0.0
        state.twist = t
        # other fields (reference_frame optional)
        state.reference_frame = 'world'

        req.state = state

        fut = self.srv_client.call_async(req)
        # optional: add future callback or just ignore
        # avoid flooding logs
        self.t += 0.1

def main(args=None):
    rclpy.init(args=args)
    try:
        node = MoveDog()
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
