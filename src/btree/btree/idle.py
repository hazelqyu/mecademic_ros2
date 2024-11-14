#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class IdleNode(Node):
    def __init__(self):
        super().__init__('face_tracker')
        self.motion_publisher = self.create_publisher(JointState, '/mecademic_robot_joint', 10)

        
    def publish_motion(self):
        state_msg = JointState()
        state_msg.header = Header()
        state_msg.name = ['meca_axis_1_joint', 'meca_axis_2_joint', 'meca_axis_3_joint', 
                        'meca_axis_4_joint', 'meca_axis_5_joint', 'meca_axis_6_joint']
        state_msg.position = [0,0,0,0,0,0]
        state_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish the joint state message
        self.motion_publisher.publish(state_msg)
        self.get_logger().info("Idle")
        
def main(args=None):
    rclpy.init(args=args)
    idle = IdleNode()
    idle.publish_motion()
    idle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()