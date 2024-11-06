#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
import cv2
import numpy as np
import math
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header



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
        image_width = 320  # Resolution width in pixels
        self.focal_length = image_width / (2 * math.tan(math.radians(horizontal_fov / 2)))
        self.cx = 160
        self.cy = 120
        
        # Real-world width of the object in meters
        self.object_real_width = 0.2
        self.obj_depth = None
        self.obj_width = 0.0
        self.obj_center = []
        self.obj_pos = None
        
        # Initialize TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer,self)
        
        self.obj_publisher = self.create_publisher(JointState, '/mecademic_robot_joint', 10)

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(f"Image conversion error:{e}")
            return
            
        self.detect_object(frame)
        if self.obj_depth and self.obj_center:
            self.get_logger().info(f"Object Center in base frame:{self.obj_center}")
            self.obj_pos = self.pos_transform(self.obj_center,self.obj_depth)
            self.lookat(self.obj_pos)
        
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
    
        # Create a PointStamped message in the camera frame
        point_in_camera_frame = PointStamped()
        point_in_camera_frame.header.frame_id = "camera_frame"
        point_in_camera_frame.header.stamp = self.get_clock().now().to_msg()
        point_in_camera_frame.point.x = x
        point_in_camera_frame.point.y = y
        point_in_camera_frame.point.z = z
        
        try:
            # Transform the point to the robot base frame
            point_in_base_frame = self.tf_buffer.transform(point_in_camera_frame, "meca_base_link", rclpy.duration.Duration(seconds=1.0))

            target_position = np.array([point_in_base_frame.point.x, point_in_base_frame.point.y, point_in_base_frame.point.z])
            self.get_logger().info(f"Object position in base frame:{point_in_base_frame.point}")
            # Now we have the object's position in the robot's base frame
            return target_position

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Transform error: {e}")
            return None

    def lookat(self,target_position):
        try:
            # Lookup the transform from "meca_axis_5_link" (end effector) to "meca_base_link" (base frame)
            end_effector_in_base_frame = self.tf_buffer.lookup_transform(
                "meca_base_link",         # Target frame
                "meca_axis_5_link",       # Source frame (end effector)
                rclpy.time.Time(),        # Time (use the latest available transform)
                timeout=rclpy.duration.Duration(seconds=1.0)  # Timeout
            )
            
            # Extract the translation component of the transform (end-effector position in the base frame)
            end_effector_x = end_effector_in_base_frame.transform.translation.x
            end_effector_y = end_effector_in_base_frame.transform.translation.y
            end_effector_z = end_effector_in_base_frame.transform.translation.z

            # Convert to a NumPy array for easier manipulation
            end_effector_position = np.array([end_effector_x, end_effector_y, end_effector_z])
            self.get_logger().info(f"End effector position in base frame: {end_effector_position}")

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Transform error for end-effector: {e}")
            end_effector_position = None  # Handle the case where the transform is unavailable
            
        if end_effector_position is None:
            return  # Exit the function if the transform failed

        direction = target_position-end_effector_position
        yaw = np.arctan2(direction[1],direction[0])
        
        state_msg = JointState()
        state_msg.header = Header()
        state_msg.name = ['meca_axis_1_joint', 'meca_axis_2_joint', 'meca_axis_3_joint', 
                        'meca_axis_4_joint', 'meca_axis_5_joint', 'meca_axis_6_joint'] 
        state_msg.position = [math.degrees(yaw), 0, 0, 0, 0, 0]
        self.obj_publisher.publish(state_msg)
        self.get_logger().info("Joint state published")
    
    # def publish_joint_state(self,joint_state_msg):
    #     # Update the header timestamp
    #     self.joint_state.header.stamp = self.get_clock().now().to_msg()

    #     # Publish the joint state message
    #     self.publisher_.publish(self.joint_state)
    #     self.get_logger().info("Joint state published")
    
def main(args=None):
    rclpy.init(args=args)
    detector = FaceDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
