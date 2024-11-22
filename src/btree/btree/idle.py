#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_interfaces.msg import SingleJointState
from std_msgs.msg import Header
import math
import time

class IdleNode(Node):
    def __init__(self):
        super().__init__('idle_node')
        self.motion_publisher = self.create_publisher(SingleJointState, '/mecademic_single_joint', 10)

        # Set parameters for sine wave
        self.amplitude = 0.1  # Amplitude in radius
        self.frequency = 0.5  # Frequency in Hz
        self.start_time = time.time()
        self.timer = self.create_timer(1, self.publish_motion)
        self.joint_4_position = self.amplitude
                
    def publish_motion(self):
        t = time.time()-self.start_time
        self.joint_4_position = -self.joint_4_position
        # joint_4_position = self.amplitude * math.sin(2*math.pi*self.frequency*t)
        state_msg = SingleJointState()
        state_msg.header = Header()
        state_msg.name = "meca_axis_4_joint"
        state_msg.idx = 3
        state_msg.position = self.joint_4_position
        state_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish the joint state message
        self.motion_publisher.publish(state_msg)
        self.get_logger().info("Idle")
        
def main(args=None):
    rclpy.init(args=args)
    idle = IdleNode()
    try:
        rclpy.spin(idle)  # Keep the node alive and spinning
    except KeyboardInterrupt:
        pass  # Handle shutdown gracefully on Ctrl+C
    finally:
        idle.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()