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
from scipy.spatial.distance import euclidean
from btree.transform_helper import FaceTransformHelper


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
        
        # Real-world width of the object in meters
        self.face_real_width = 0.2

        # self.mp_face_detection = mp.solutions.face_detection.FaceDetection(min_detection_confidence=0.3)
        self.yolo_detector = YoloDetector(min_face=50, target_size=640, device='cpu')
        
        # Tracked faces
        self.previous_faces = []
        self.next_face_id = 0
        self.closest_face = None
        self.newest_face = None
        self.closest_face_pos = None
        self.newest_face_pos = None
        
        # For threshold
        self.pre_yaw = None
        self.pre_pitch = None
        self.pre_joint2_pitch = None
        
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
        self.closest_face_publisher = self.create_publisher(JointState, '/face_position', 10)
        self.newest_face_publisher = self.create_publisher(JointState, '/newest_face_position', 10)
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
        self.is_bored = self.condition_checker.check_face_still(self.is_detected, self.closest_face_pos)
        self.is_alert = self.condition_checker.check_face_appear(self.face_count)
        self.get_logger().info(f"Alert:{self.is_alert},{self.face_count}")
        
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
        
        current_faces = []
        MAX_DISTANCE_THRESHOLD = 50
        closest_face_depth = float('inf')
        
        if bboxes[0]:  # If there are detected faces
            for bbox in bboxes[0]:
                x1, y1, x2, y2 = bbox
                w = x2 - x1
                h = y2 - y1
                center_x = x1 + w // 2
                center_y = y1 + h // 2
                depth = (self.face_real_width * self.focal_length) / h
                
                # Match current face with previously tracked faces
                matched_face = None
                for prev_face in self.previous_faces:
                    prev_center = prev_face['center']
                    distance = euclidean((center_x, center_y), prev_center)
                    if distance < MAX_DISTANCE_THRESHOLD:
                        matched_face = prev_face
                        break

                if matched_face:
                    # Update the matched face's attributes
                    matched_face['bbox'] = (x1, y1, w, h)
                    matched_face['center'] = (center_x, center_y)
                    matched_face['depth'] = depth
                    current_faces.append(matched_face)
                else:
                    # Assign a new ID to the new face
                    new_face = {
                        'id': self.next_face_id,
                        'bbox': (x1, y1, w, h),
                        'center': (center_x, center_y),
                        'depth': depth
                    }
                    self.next_face_id += 1
                    current_faces.append(new_face)
                    self.newest_face = new_face  # Update the newest face

                if depth < closest_face_depth:
                    closest_face_depth = depth
                    self.closest_face = matched_face or new_face

            
            self.previous_faces = current_faces
            
            if self.closest_face:
                (x, y, w, h) = self.closest_face['bbox']
                self.is_detected = True

                closest_face_helper = FaceTransformHelper(self.closest_face,self.tf_buffer,logger=self)
                self.closest_face_pos = closest_face_helper.pos_transform(self.get_clock().now().to_msg())
                closest_face_target_joint_pos = closest_face_helper.compute_target_joint_position(self.closest_face_pos)
                self.publish_joint_state(closest_face_target_joint_pos,self.closest_face_publisher)
                
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.circle(frame, self.closest_face['center'], 5, (0, 255, 0), -1)
                cv2.putText(frame, f"Depth: {self.closest_face['depth']:.2f} m", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                
                face_image = frame[y:y+h, x:x+w]
                emotion, confidence = self.recognize_expression(face_image)
                
                self.emotion = emotion if confidence > 0.5 else ""

                if self.emotion and confidence is not None:
                    cv2.putText(frame, f'{self.emotion} ({confidence:.2f})', (x, y - 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (36, 255, 12), 2)
                else:
                    cv2.putText(frame, 'No emotion detected', (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            if self.newest_face:
                (x, y, w, h) = self.closest_face['bbox']
                newest_face_helper = FaceTransformHelper(self.newest_face,self.tf_buffer,logger=self)
                self.newest_face_pos = newest_face_helper.pos_transform(self.get_clock().now().to_msg())
                newest_face_target_joint_pos = newest_face_helper.compute_target_joint_position(self.newest_face_pos)
                self.publish_joint_state(newest_face_target_joint_pos,self.newest_face_publisher)
                
                
        cv2.imshow('YOLO Facial Tracking', frame)
        cv2.waitKey(1)
        
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
    
    def publish_joint_state(self, target_joint_state, publisher):
        state_msg = JointState()
        state_msg.header = Header()
        state_msg.name = ['meca_axis_1_joint', 'meca_axis_2_joint', 'meca_axis_3_joint', 
                        'meca_axis_4_joint', 'meca_axis_5_joint', 'meca_axis_6_joint']
        state_msg.position = target_joint_state
        state_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish the joint state message
        publisher.publish(state_msg)
        self.get_logger().info("Joint state published")
   
 
def main(args=None):
    rclpy.init(args=args)
    detector = FaceDetectorNode()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
