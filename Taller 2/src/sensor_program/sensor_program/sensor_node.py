import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.publisher_ = self.create_publisher(String, 'sensor_data', 10)
        self.timer = self.create_timer(1.0, self.publish_data)

    def publish_data(self):
        msg = String()
        temperatura = random.randint(20, 30)
        msg.data = f'Temperatura: {temperatura} C'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publicando: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()