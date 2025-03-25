import os
import numpy as np
import cv2
from tensorflow.keras.models import load_model
from tensorflow.keras.preprocessing.image import load_img, img_to_array

from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from lane_msgs.msg import LanePositions
from rclpy.clock import Clock
from tensorflow.keras.preprocessing.image import load_img, img_to_array #LOL

class VisionNode(Node):
    model = load_model("unet_lane_detection_model.h5")
   
    def __init__(self): 
        super().__init__("vision_node")
        self.subscription = self.create_subscription(Image, "image_raw", self.image_callback, 10)
        self.lane_pos_publisher = self.create_publisher(LanePositions, "lane_position", 10)
        self.mask_img_publisher = self.create_publisher(LanePositions, "mask_img", 10)
        self.bridge = CvBridge()

    def rdp_simplify(self, points):
        """ Apply the Ramer-Douglas-Peucker algorithm to simplify a curve. """
        points = np.array(points, dtype=np.int32)
        return cv2.approxPolyDP(points, 0.5, False)[:, 0, :].tolist()

    def compute_centerline(self, contour):
        """ Compute the centerline by averaging x-coordinates for each unique y. """
        points = contour[:, 0, :]
        unique_y = np.unique(points[:, 1])
        centerline = []

        for y in unique_y:
            x_values = points[points[:, 1] == y, 0]
            avg_x = int(np.mean(x_values))
            centerline.append((avg_x, int(y)))

        return self.rdp_simplify(centerline)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        image_resized = cv2.resize(cv_image, (256, 256))
        image_array = image_resized.astype("float32") / 255.0  # Normalize to [0, 1]
        image_array = np.expand_dims(image_array, axis=0)

        predicted_mask = self.model.predict(image_array)[0]
        predicted_mask = (predicted_mask * 255).astype(np.uint8)

        _, binary_mask = cv2.threshold(predicted_mask, 127, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [cnt for cnt in contours if len(cnt) > 1]

        if len(contours) == 0:
            return None, None

        contours = sorted(contours, key=lambda cnt: np.mean(cnt[:, 0, 0]))

        left_lane = self.compute_centerline(contours[0]) if len(contours) > 0 else None
        right_lane = self.compute_centerline(contours[-1]) if len(contours) > 1 else None
        #TODO: model prediction on raw image
        #       - obtain mask and publish it 
        #       - extract lane postion 
        #print()
        self.publish_lane_position(left_lane, right_lane)

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
