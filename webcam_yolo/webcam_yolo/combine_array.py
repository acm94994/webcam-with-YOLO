import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class ArrayCombiner(Node):
    def __init__(self):
        super().__init__('combine')
        self.coords_subscription = self.create_subscription(
            Float32MultiArray,
            'centre_coords',
            self.coords_callback,
            10
        )
        self.random_subscription = self.create_subscription(
            Float32MultiArray,
            'random_array',
            self.random_callback,
            10
        )
        self.coords_data = None
        self.random_data = None

    def coords_callback(self, msg):
        self.coords_data = msg.data
        self.combine_and_print()

    def random_callback(self, msg):
        self.random_data = msg.data
        self.combine_and_print()

    def combine_and_print(self):
        if self.coords_data is not None and self.random_data is not None:
            combined_array = self.coords_data + self.random_data
            self.get_logger().info(f'Combined array: {combined_array}')

def main(args=None):
    rclpy.init(args=args)
    node = ArrayCombiner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()