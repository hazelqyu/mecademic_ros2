#!/usr/bin/env python3

import rclpy
from typing import Optional
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge,CvBridgeError
import cv2
import numpy as np
# from tensorflow.keras.models import load_model
from deepface import DeepFace
import math
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header,Bool,String
from btree.check_condition import ConditionChecker
from yoloface.face_detector import YoloDetector
import time


class FaceDetectorNode(Node):
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
        self.face_real_width = 0.2
        self.face_depth = None
        self.face_width = 0.0
        self.face_center = []
        self.face_pos = None
        # self.mp_face_detection = mp.solutions.face_detection.FaceDetection(min_detection_confidence=0.3)
        self.yolo_detector = YoloDetector(min_face=50, target_size=640, device='cpu')
        
        # For threshold
        self.pre_yaw = None
        self.pre_pitch = None
        self.pre_joint2_pitch = None
        
        self.target_joint_state = [0,0,0,0,0,0]
        
        # Load the pre-trained FER2013 model (mini-XCEPTION trained on FER2013)
        # self.model = load_model('/home/andrek/ros2_ws/emotion _detection.keras')

        # Define the emotion labels for FER2013 (7 classes)
        # self.emotion_labels = ['Angry', 'Disgust', 'Fear', 'Happy', 'Sad', 'Surprise', 'Neutral']
        self.emotion = None
        
        # Initialize TF buffer and listener
        self.frames = ["camera_frame", "meca_base_link", "meca_axis_1_link", "meca_axis_2_link","meca_axis_3_link",  "meca_axis_5_link"]
        self.frames_available = {frame:False for frame in self.frames}
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer,self)
        # Create a timer to check for all required frames before proceeding
        self.check_frames_timer = self.create_timer(0.1, self.check_frames)
        
        # Configure publishers for btree
        self.is_awake_publisher = self.create_publisher(Bool,'/is_awake',10)
        self.is_detected_publisher = self.create_publisher(Bool,'/is_detected',10)
        self.emotion_publisher = self.create_publisher(String,'/face_emotion',10)
        self.is_alert_publisher = self.create_publisher(Bool,'/is_alert',10)
        self.is_bored_publisher = self.create_publisher(Bool,'/is_bored',10)
        self.face_publisher = self.create_publisher(JointState, '/face_position', 10)
        self.condition_publish_timer = self.create_timer(0.1,self.publish_condition)
        
        self.is_awake = True
        self.is_detected = False
        self.is_alert = False
        self.is_bored = False
        self.emotion = ""
        self.face_count = 0
        self.condition_checker = ConditionChecker()
    
    def publish_condition(self):
        # Make sure publish at least once before next check
        self.is_awake = self.condition_checker.check_awake(self.is_detected)
        self.is_bored = self.condition_checker.check_face_still(self.is_detected, self.face_pos)
        self.is_alert = self.condition_checker.check_face_appear(self.face_count)
        
        is_awake_msg = Bool()
        is_awake_msg.data = self.is_awake
        self.is_awake_publisher.publish(is_awake_msg)
        
        is_detected_msg = Bool()
        is_detected_msg.data = self.is_detected
        self.is_detected_publisher.publish(is_detected_msg)
        
        is_bored_msg = Bool()
        is_bored_msg.data = self.is_bored
        self.is_bored_publisher.publish(is_bored_msg)
        
        is_alert_msg = Bool()
        is_alert_msg.data = self.is_alert
        self.is_alert_publisher.publish(is_alert_msg)
        
        face_emotion_msg = String()
        face_emotion_msg.data = self.emotion
        self.emotion_publisher.publish(face_emotion_msg)
        self.get_logger().info(f"Plant Awake:{self.is_awake}")
    
    def check_frames(self):
        # Check each required frame
        all_frames_available = True
        for frame in self.frames:
            if not self.frames_available[frame]:
                # Check if the transform is available
                if self.tf_buffer.can_transform(frame, "meca_base_link", rclpy.time.Time()):
                    self.frames_available[frame] = True
                    self.get_logger().info(f"{frame} is now available in tf2.")
                else:
                    all_frames_available = False

        # If all frames are available, stop the timer
        if all_frames_available:
            self.get_logger().info("All required frames are now available.")
            self.check_frames_timer.cancel()  # Stop the timer once all frames are available


    def image_callback(self, msg):
        if not all(self.frames_available.values()):
            self.get_logger().info("Waiting for all frames to be available...")
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(f"Image conversion error: {e}")
            return

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self.is_detected = False
        
        # Use YOLO for face detection
        bboxes, _ = self.yolo_detector.predict(rgb_frame, conf_thres=0.3, iou_thres=0.5)
        self.face_count = len(bboxes[0])
        
        closest_face_depth = float('inf')
        closest_face = None
        
        if bboxes[0]:  # If there are detected faces
            for bbox in bboxes[0]:
                x1, y1, x2, y2 = bbox
                w = x2 - x1
                h = y2 - y1
                depth = (self.face_real_width * self.focal_length) / w

                if depth < closest_face_depth:
                    closest_face_depth = depth
                    closest_face = (x1, y1, w, h, depth)

            if closest_face:
                x, y, w, h, self.face_depth = closest_face
                self.face_width = w
                center_x = x + self.face_width // 2
                center_y = y + h // 2
                self.face_center = [center_x, center_y]
                self.is_detected = True

                if self.face_depth and self.face_center:
                    self.get_logger().info(f"Face Center in camera frame: {self.face_center}")
                    self.face_pos = self.pos_transform(self.face_center, self.face_depth)
                    self.lookat(self.face_pos)

                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)
                cv2.putText(frame, f"Depth: {self.face_depth:.2f} m", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                
                # Extract the face from the frame
                face_image = frame[y:y+h, x:x+w]

                # Recognize the emotion using the FER2013 model
                emotion, confidence = self.recognize_expression(face_image)
                
                # only update the emotion when confidence level is greater than 0.5
                self.emotion = emotion if confidence > 0.5 else ""

                # # Only display the emotion and confidence if they are not None
                if self.emotion and confidence is not None:
                    # Draw the bounding box and emotion on the frame
                    cv2.putText(frame, f'{self.emotion} ({confidence:.2f})', (x, y - 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (36, 255, 12), 2)
                else:
                    # Optionally, show a default message if no emotion is detected
                    cv2.putText(frame, 'No emotion detected', (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                
                
        cv2.imshow('YOLO Facial Tracking', frame)
        cv2.waitKey(1)
        
    # def preprocess_face(self,face_image):
    #     if face_image is None or face_image.size == 0:
    #         self.get_logger().error("Received an empty face image.")
    #         return None
    #     face_image = cv2.cvtColor(face_image, cv2.COLOR_BGR2GRAY)  # Convert face image to grayscale
    #     face_image = cv2.resize(face_image, (64, 64))  # Resize to 64x64 (FER2013 input size)
    #     face_image = face_image.astype('float32') / 255  # Normalize pixel values to [0, 1]
    #     face_image = np.expand_dims(face_image, axis=0)  # Add batch dimension
    #     face_image = np.expand_dims(face_image, axis=-1)  # Add channel dimension (grayscale)
    #     return face_image

    # # Function to recognize emotion using the FER2013 model
    # def recognize_expression(self,face_image):
    #     preprocessed_face = self.preprocess_face(face_image)
    #     if preprocessed_face is None:
    #         return None, 0.0  # Or handle the case as needed
    #     predictions = self.model.predict(preprocessed_face)  # Get model predictions
    #     emotion_index = np.argmax(predictions)  # Find the highest confidence index
    #     return self.emotion_labels[emotion_index], np.max(predictions)  # Return emotion label and confidence

    def recognize_expression(self,face_image):
        result = DeepFace.analyze(face_image, actions=['emotion'], enforce_detection=False)
        try:
            first_result = result[0]
            emotion = first_result['dominant_emotion']
            confidence = first_result['emotion'][emotion]
        except Exception as e:
            print(f"Error accessing predictions: {e}")
            emotion = "Unknown"
            confidence = 0.0
        return emotion, confidence
    
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

            return target_position

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Transform error: {e}")
            return None

    def lookat(self,target_position):

        joint1_direction = self.compute_direction("meca_axis_1_link",target_position)
        joint3_direction = self.compute_direction("meca_axis_3_link",target_position)
        joint5_direction = self.compute_direction("meca_axis_5_link",target_position)
        if joint1_direction is None or joint3_direction is None or joint5_direction is None:
            return  # Exit the function if the transform failed
        
        yaw = np.arctan2(joint1_direction[1],joint1_direction[0]) # joint 1 will do the yaw
        joint2_pitch = self.back_forth_movemont(self.face_depth)
        joint3_pitch = np.arctan2(joint3_direction[2],joint3_direction[0])-joint2_pitch # joint 3 will do the pitch
        joint5_pitch = np.arctan2(joint5_direction[2],joint5_direction[0])-joint2_pitch
        
        percentage = 0.5
        
        movement_threshold = 0.1 # This threshold is in radius, about 5.73 degree
        # Only update target_pos when it's greater than movement threshold
        h_movement = abs(yaw - self.pre_yaw) if self.pre_yaw is not None else movement_threshold + 1
        v_movement = abs(joint3_pitch-self.pre_pitch) if self.pre_pitch is not None else movement_threshold + 1
        # depth_movement = abs(joint2_pitch-self.pre_joint2_pitch) if self.pre_joint2_pitch is not None else movement_threshold +1
        
        if h_movement >= movement_threshold or v_movement>=movement_threshold:
            self.target_joint_state = [yaw, -joint2_pitch, -joint3_pitch*percentage, 0, -joint5_pitch*(1-percentage), 0]
            self.pre_yaw = yaw
            self.pre_pitch = joint3_pitch 
            self.publish_joint_state()
    
    def back_forth_movemont(self,depth) -> float:
        joint2_pitch =  1.2-depth
        return max(min(joint2_pitch, 1.2), -1.5)
        
    def compute_direction(self,link_name:str,target_position) -> Optional[np.ndarray]:
        try:
            link_frame = self.tf_buffer.lookup_transform(
                "meca_base_link",         # Target frame
                link_name,       # Source frame
                rclpy.time.Time(),        # Time (use the latest available transform)
                timeout=rclpy.duration.Duration(seconds=1.0)  # Timeout
            )
            
            link_position = np.array([link_frame.transform.translation.x, link_frame.transform.translation.y, link_frame.transform.translation.z])
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Transform error for joints: {e}")
            link_position = None
        
        if link_position is None:
            return None
        
        direction = target_position-link_position
        return direction
    
    def publish_joint_state(self):
        state_msg = JointState()
        state_msg.header = Header()
        state_msg.name = ['meca_axis_1_joint', 'meca_axis_2_joint', 'meca_axis_3_joint', 
                        'meca_axis_4_joint', 'meca_axis_5_joint', 'meca_axis_6_joint']
        state_msg.position = self.target_joint_state
        state_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish the joint state message
        self.face_publisher.publish(state_msg)
        self.get_logger().info("Joint state published")
   
 
def main(args=None):
    rclpy.init(args=args)
    detector = FaceDetectorNode()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
