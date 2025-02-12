import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2 as cv

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Image()

        cv.imread("../assets/800px-Road_in_Norway.jpg")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        msg.height = 480
        msg.width = 640
        msg.encoding = 'rgb8'
        msg.step = msg.width * 3  # 3 bytes per pixel for rgb8
        msg.data = np.zeros((msg.height, msg.width, 3), dtype=np.uint8).tobytes()  # Dummy black image
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing an image')

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
