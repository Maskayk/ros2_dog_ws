import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

class StandNode(Node):
    def __init__(self):
        super().__init__('stand_node')
        self.publisher = self.create_publisher(Float64MultiArray, '/dog_leg_controller/commands', 10)
        time.sleep(1.0)
        msg = Float64MultiArray()
        # углы стоячей позы, пример:
        msg.data = [0.3, -0.6, 0.3, -0.6, 0.3, -0.6, 0.3, -0.6]
        self.publisher.publish(msg)
        self.get_logger().info('🐶 Standing pose published.')

def main(args=None):
    rclpy.init(args=args)
    node = StandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
