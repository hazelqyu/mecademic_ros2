#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class FaceTrackerNode(Node):
    def __init__(self):
        super().__init__('face_tracker')
        self.subscriptions = self.create_subscription(
            JointState,
            '/face_position',
            self.publish_command,
            10)
        self.command_publisher = self.create_publisher(JointState, '/mecademic_robot_joint', 10)
    
        
    def publish_command(self,state_msg):
        self.command_publisher.publish(state_msg)

def main(args=None):
    rclpy.init(args=args)
    tracker = FaceTrackerNode()
    rclpy.spin(tracker)
    tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()