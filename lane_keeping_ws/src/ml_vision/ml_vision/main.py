import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from lane_msgs.msg import LanePositions

class VisionNode(Node):
    def __init__(self): 
        super().__init__("vision_node")
        self.subscription = self.create_subscription(Image, "image_raw", self.image_callback, 10)
        self.publisher = self.create_publisher(LanePosition, "lane_position", 10)

    def image_callback(self, msg):
        #TODO: model prediction on raw image

    def publish_lane_position(self):
        #TODO: publish lane vectors


def main():
    print("hello world\n")

if __name__ == "__main__":
    main()
