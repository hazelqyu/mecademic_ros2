#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        
        # Create a publisher for the /mecademic_robot_joint topic
        self.publisher_ = self.create_publisher(JointState, '/mecademic_robot_joint', 10)
        
        # Define joint state message
        self.joint_state = JointState()
        self.joint_state.header = Header()
        self.joint_state.name = ['meca_axis_1_joint', 'meca_axis_2_joint', 'meca_axis_3_joint', 
                                 'meca_axis_4_joint', 'meca_axis_5_joint', 'meca_axis_6_joint']  # Replace with your joint names
        self.joint_state.position = [0, -60, 60, 0, 0, 0]  # Set desired joint positions in radians

        # Set the rate at which to publish (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_joint_state)
    
    def publish_joint_state(self):
        # Update the header timestamp
        self.joint_state.header.stamp = self.get_clock().now().to_msg()

        # Publish the joint state message
        self.publisher_.publish(self.joint_state)
        self.get_logger().info("Joint state published")
    


def main(args=None):
    rclpy.init(args=args)

    # Create and run the node
    joint_state_publisher = JointStatePublisher()
    rclpy.spin(joint_state_publisher)

    # Clean up and shutdown
    joint_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
