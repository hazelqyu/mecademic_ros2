import numpy as np
import tf2_ros
from geometry_msgs.msg import PointStamped
from typing import Optional
import math
import rclpy


class FaceTransformHelper:
    def __init__(self,face,tf_buffer,logger,horizontal_fov = 78, image_width = 640,):
        self.face = face
        self.tf_buffer = tf_buffer
        self.logger = logger
        self.focal_length = image_width / (2 * math.tan(math.radians(horizontal_fov / 2)))
        self.cx = 320
        self.cy = 240
        
        self.depth = self.face['depth']
        
        # For threshold
        self.pre_yaw = None
        self.pre_pitch = None
        self.pre_joint2_pitch = None
    
    def get_target_joint_position(self,timestamp):
        transformed_position =  self.pos_transform(timestamp)
        target_joint_position = self.compute_target_joint_position(transformed_position)
        return target_joint_position
        
       
    def pos_transform(self,timestamp):
        center = self.face['center']
        
        x = (center[0] - self.cx) * self.depth / self.focal_length
        y = (center[1] - self.cy) * self.depth / self.focal_length
        z = self.depth

        point_in_camera_frame = PointStamped()
        point_in_camera_frame.header.frame_id = "camera_frame"
        point_in_camera_frame.header.stamp = timestamp
        point_in_camera_frame.point.x = x
        point_in_camera_frame.point.y = y
        point_in_camera_frame.point.z = z

        try:
            point_in_base_frame = self.tf_buffer.transform(point_in_camera_frame, "meca_base_link", rclpy.duration.Duration(seconds=1.0))
            transformed_position = np.array([point_in_base_frame.point.x, point_in_base_frame.point.y, point_in_base_frame.point.z])
            return transformed_position

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.logger.error(f"Transform error for face: {e}")
            return None

    def compute_target_joint_position(self,target_position):

        joint1_direction = self.compute_direction("meca_axis_1_link",target_position)
        joint3_direction = self.compute_direction("meca_axis_3_link",target_position)
        joint5_direction = self.compute_direction("meca_axis_5_link",target_position)
        if joint1_direction is None or joint3_direction is None or joint5_direction is None:
            return  # Exit the function if the transform failed
        
        yaw = np.arctan2(joint1_direction[1],joint1_direction[0]) # joint 1 will do the yaw
        joint2_pitch = self.back_forth_movement(self.depth)
        joint3_pitch = np.arctan2(joint3_direction[2],joint3_direction[0])-joint2_pitch # joint 3 will do the pitch
        joint5_pitch = np.arctan2(joint5_direction[2],joint5_direction[0])-joint2_pitch
        
        percentage = 0.5
        
        movement_threshold = 0.1 # This threshold is in radius, about 5.73 degree
        # Only update target_pos when it's greater than movement threshold
        h_movement = abs(yaw - self.pre_yaw) if self.pre_yaw is not None else movement_threshold + 1
        v_movement = abs(joint3_pitch-self.pre_pitch) if self.pre_pitch is not None else movement_threshold + 1
        # depth_movement = abs(joint2_pitch-self.pre_joint2_pitch) if self.pre_joint2_pitch is not None else movement_threshold +1
        
        if h_movement >= movement_threshold or v_movement>=movement_threshold:
            target_joint_position = [yaw, -joint2_pitch, -joint3_pitch*percentage, 0, -joint5_pitch*(1-percentage), 0]
            self.pre_yaw = yaw
            self.pre_pitch = joint3_pitch 
            return target_joint_position
    
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

    def back_forth_movement(self, depth: float) -> float:
        """Calculate joint2 pitch based on depth."""
        joint2_pitch = 1.2 - depth
        return max(min(joint2_pitch, 1.2), -1.5)
