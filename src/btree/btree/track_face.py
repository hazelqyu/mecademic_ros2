#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


class FaceTrackerNode(Node):
    def __init__(self):
        super().__init__('face_tracker')
        self.face_sub = self.create_subscription(JointState,'/face_position',self.set_target,10)
        self.btree_sub = self.create_subscription(Bool,'/start_tracking',self.publish_command,10)
        self.command_publisher = self.create_publisher(JointState, '/mecademic_robot_joint', 10)
        self.target_face = JointState()
        
    def publish_command(self,msg):
        self.get_logger().info("Publishing target face.")
        self.command_publisher.publish(self.target_face)
    
    def set_target(self,msg):
        self.target_face = msg
        

def main(args=None):
    rclpy.init(args=args)
    tracker = FaceTrackerNode()
    try:
        rclpy.spin(tracker) 
    except KeyboardInterrupt:
        pass
    finally:
        tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()