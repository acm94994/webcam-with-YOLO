import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import torch
from ultralytics import YOLO

class YoloV8Subscriber(Node):
    def __init__(self):
        super().__init__('yolov8_subscriber')
        self.center_publisher = self.create_publisher(Float32MultiArray, 'centre_coords', 10)
        self.subscription = self.create_subscription(Image, 'webcam_image', self.listener_callback, 10)
        self.br = CvBridge()
        self.model = YOLO('yolov8n.pt')
        self.frame = None  # Initialize the frame variable to store incoming images

        # Create a timer to process and publish at a fixed interval
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.process_and_publish)

    def listener_callback(self, msg):
        self.frame = self.br.imgmsg_to_cv2(msg, "bgr8")

    def process_and_publish(self):
        if self.frame is None:
            return
        
        frame = self.frame
        results = self.model(frame)
        bottle_class_id = 39

        # Visualize and display the results
        for result in results:
            for box in result.boxes:
                cls = int(box.cls[0])
                if cls == bottle_class_id:  # Check if the detected object is a bottle
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, x2, y1, y2 = int(x1), int(x2), int(y1), int(y2)
                    conf = box.conf[0]
                    label = f'Bottle {conf:.2f}'

                    # Ensure coordinates are integers
                    x1, y1, x2, y2 = map(int, (x1, y1, x2, y2))

                    # Draw rectangle and label
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                    cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                    x0 = int(0.5 * (x1 + x2))
                    y0 = int(0.5 * (y1 + y2))
                    centres = [x0, y0]
                    center_msg = Float32MultiArray(data=centres)
                    self.center_publisher.publish(center_msg)

        cv2.imshow("YOLOv8 Detection", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YoloV8Subscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
