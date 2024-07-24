import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import random

class RandomNumberPublisher(Node):
    def __init__(self):
        super().__init__('rng')
        self.publisher = self.create_publisher(Float32MultiArray, 'random_array', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.rng_callback)

    def rng_callback(self):
        x = random.randint(1,100)
        y = random.randint(1,100)
        array = [x,y]
        msg = Float32MultiArray(data=array)
        self.publisher.publish(msg)
        info = "Publishing: " + str(array)
        self.get_logger().info(info)

def main(args=None):
    rclpy.init(args=args)
    node = RandomNumberPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
