#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
import cv2
import numpy as np
import math
import tf2_ros
from geometry_msgs.msg import PointStamped


class FaceDetector(Node):
    def __init__(self):
        super().__init__('face_detector')
        self.subscription = self.create_subscription(
            Image,
            '/image',  # Use the topic from image_tools
            self.image_callback,
            10)
        self.bridge = CvBridge()

        # Calculate focal length using the 78° FoV and 640-pixel width
        horizontal_fov = 78  # Horizontal field of view in degrees
        image_width = 640  # Resolution width in pixels
        self.focal_length = image_width / (2 * math.tan(math.radians(horizontal_fov / 2)))
        self.cx = 320
        self.cy = 240
        
        # Real-world width of the object in meters
        self.object_real_width = 0.18
        self.obj_depth = None
        self.obj_width = 0.0
        self.obj_center = []

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(f"Image conversion error:{e}")
            return
            
        self.detect_object(frame)
        if self.obj_depth and self.obj_center:
            self.pos_transform(self.obj_center,self.obj_depth)
        
        # Show the result
        cv2.imshow("Blue Object Detection with Depth", frame)
        cv2.waitKey(1)

    def detect_object(self, image):
        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        blue_M = cv2.moments(mask)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if blue_M["m00"] == 0 or contours is None:
            return None,0.0
        
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)
        self.obj_depth = (self.object_real_width * self.focal_length) / w
        cv2.putText(image, f"Depth: {self.obj_depth :.2f} m", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        self.obj_width = w
        blue_cX = int(blue_M["m10"] / blue_M["m00"])
        blue_cY = int(blue_M["m01"] / blue_M["m00"])
        self.obj_center = [blue_cX, blue_cY]

    def pos_transform(self,center,depth):
        # Calculate the 3D coordinates relative to the camera
        x = (center[0] - self.cx) * depth / self.focal_length
        y = (center[1] - self.cy) * depth / self.focal_length
        z = depth

        return [x, y, z]

    
def main(args=None):
    rclpy.init(args=args)
    detector = FaceDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
