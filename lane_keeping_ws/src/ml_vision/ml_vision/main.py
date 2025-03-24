import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from lane_msgs.msg import LanePositions
from rclpy.clock import Clock
from tensorflow.keras.preprocessing.image import load_img, img_to_array #LOL

class VisionNode(Node):
    def __init__(self): 
        super().__init__("vision_node")
        self.subscription = self.create_subscription(Image, "image_raw", self.image_callback, 10)
        self.lane_pos_publisher = self.create_publisher(LanePositions, "lane_position", 10)
        self.mask_img_publisher = self.create_publisher(LanePositions, "mask_img", 10)

    def image_callback(self, msg):
        #TODO: model prediction on raw image
        #       - obtain mask and publish it 
        #       - extract lane postion 
        print()

    def publish_lane_position(self, left_lane_points, right_lane_points):
        msg = LanePositions()
        msg.header.stamp = Clock().now()
        msg.left_lane = left_lane_points
        msg.right_lane = right_lane_points
        self.lane_pos_publisher.publish(msg)

def main():
    rclpy.init()
    vision_node = VisionNode()
    rclpy.spin(vision_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
