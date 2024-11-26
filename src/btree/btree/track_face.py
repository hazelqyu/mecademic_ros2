#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


class FaceTrackerNode(Node):
    def __init__(self):
        super().__init__('face_tracker')
        self.face_sub = self.create_subscription(JointState,'/face_position',self.set_target,10)
        self.btree_sub = self.create_subscription(Bool,'/start_tracking',self.start_tracking_callback,10)
        self.command_publisher = self.create_publisher(JointState, '/mecademic_robot_joint', 10)
        self.target_face = None
        self.timer = None
    
    def start_tracking_callback(self,msg):
        if msg.data == True and not self.timer:
            self.get_logger().info("Starting tracking motion.")
            self.timer = self.create_timer(0.05, self.publish_command)

        if msg.data == False:
            if self.timer:
                self.timer.cancel()
                self.timer = None
        
    def publish_command(self):
        self.get_logger().info("Publishing target face.")
        if self.target_face:
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