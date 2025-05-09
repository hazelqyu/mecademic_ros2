#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Bool,Float64MultiArray
import math
import time
from meca_controller.robot_motion import RobotMotion

class IdleNode(Node):
    def __init__(self):
        super().__init__('idle_node')
        # self.robot = MecademicRobotDriver().robot
        self.btree_sub = self.create_subscription(Bool, '/start_idling', self.start_idling_callback, 10)
        self.command_publisher = self.create_publisher(JointState, '/mecademic_robot_joint', 10)

        self.timer = None # A timer to publish command

    def start_idling_callback(self, msg):
        
        if msg.data == True and not self.timer:
            self.get_logger().info("Starting idle motion.")
            self.timer = self.create_timer(0.05, self.publish_command)

        if msg.data == False:
            if self.timer:
                self.timer.cancel()
                self.timer = None

    def publish_command(self):
        time_now = self.get_clock().now().nanoseconds * 1e-9
        sine_value = 0.05 * math.sin(2 * math.pi * 0.5 * time_now)
        cosine_value = 0.5 * math.sin(2 * math.pi * 0.15 * time_now)

        state_msg = JointState()
        state_msg.header = Header()
        state_msg.name = [
            'meca_axis_1_joint', 'meca_axis_2_joint', 'meca_axis_3_joint',
            'meca_axis_4_joint', 'meca_axis_5_joint', 'meca_axis_6_joint'
        ]
        # idling
        state_msg.position = [cosine_value, -0.2+0.5*sine_value, -0.45+sine_value, 0, sine_value, 0]

        state_msg.header.stamp = self.get_clock().now().to_msg()
        self.command_publisher.publish(state_msg)
        
def main(args=None):
    rclpy.init(args=args)
    idle = IdleNode()
    # idle.test_wave()
    try:
        rclpy.spin(idle)
    except KeyboardInterrupt:
        pass
    finally:
        idle.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
