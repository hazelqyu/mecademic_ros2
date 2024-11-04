#! /usr/bin/env python3

import mecademicpy.robot as mdr
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose,Twist
from sensor_msgs.msg import JointState
import time

class MecademicRobotDriver(Node):
    def __init__(self):
        super().__init__("mecademic_robot_driver")

        self.robot = mdr.Robot()
        self.robot.Connect(address='192.168.0.100', enable_synchronous_mode=True)
        self.robot.ActivateAndHome()
        # Provide all available real-time feedback messages
        self.robot.SetRealTimeMonitoring('all')
        self.robot.WaitIdle(timeout=60)
        print('ready.')
        
        # Set monitoring interval for real time data:
        self.MONITORING_INTERVAL = 0.001 # seconds TODO ADJUST THIS
        self.robot.SetMonitoringInterval(self.MONITORING_INTERVAL)
        DATA_LOGGING_TIME_INTERVAL = .001 # seconds TODO ADJUST THIS
        self.create_timer(DATA_LOGGING_TIME_INTERVAL, self.timed_data_logging_callback)
        
        # TODO: create publisher for feedback data
        
        # Set up subscriber to recieve ROS command
        self.pose_subscriber = self.create_subscription(Pose, "mecademic_robot_pose", self.pose_callback, 10)
        self.joint_subscriber = self.create_subscription(JointState, "mecademic_robot_joint", self.joint_callback, 10)
        self.joint_vel_subscriber = self.create_subscription(Twist,"/cmd_vel",self.set_joint_vel_callback,10)
        
        # TODO: Set up services
        
        # Keep track of error state:
        self.is_in_error = False
        
        # For xbox controller
        # Initialize movement variables
        self.target_velocity = 0.0
        self.scale_factor = 10.0  # Adjust as needed
        self.prev_velocity = 0.0
        self.last_update_time = time.time()
        
        # Set up a timer to mimic the Unity update loop
        self.update_rate = 0.02  # 50 Hz, similar to Unity frame rate
        self.create_timer(self.update_rate, self.update_movement)
        
    def stop(self):
        try:
            self.robot.WaitIdle(60) 
            self.robot.DeactivateRobot()
            self.robot.Disconnect()
            print('stopped')
        except mdr.DisconnectError:
            self.stop_after_disconnect_error()
    
    def stop_after_disconnect_error(self):
        # 1) Reconnect so that we can properly shut down:
        self.robot.Connect(address='192.168.0.100')
        
        # 2) Reset the error, if there is any, and resume robot motion if it is paused (it will be after a DisconnectError):
        self.robot.ResetError() # just in case
        self.robot.ResumeMotion()
        self.stop()
    
    def handle_error(self):
        print('robot is in error state, correcting..')
        self.robot.ResetError()
        self.is_in_error = False
        print('...error state corrected.')
        self.robot.ResumeMotion()
    
    def timed_data_logging_callback(self):
        # TODO:Feedback Data logging: to be implemented later
        robot_status = self.robot.GetStatusRobot()
        self.is_in_error = robot_status.error_status
        if self.is_in_error:
            self.handle_error()
            
    
    def pose_callback(self, pose):
        self.get_logger().info("Received pose message")

    def joint_callback(self, joints):
        self.get_logger().info(f"Received joint message: {joints.position}")
        # Move robot joints based on received joint positions
        self.robot.MoveJoints(joints.position[0], joints.position[1], joints.position[2],
                              joints.position[3], joints.position[4], joints.position[5])
    
    def set_joint_vel_callback(self, joints_vel):
        self.target_velocity = joints_vel.linear.x * self.scale_factor
        
    def update_movement(self):
        # Calculate delta time for scaling the movement
        current_time = time.time()
        delta_time = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # Smooth the movement by blending previous and current velocity
        smoothed_velocity = self.prev_velocity * 0.8 + self.target_velocity * 0.2
        self.prev_velocity = smoothed_velocity
        
        # Move the robot based on smoothed velocity scaled by delta time
        try:
            self.robot.MoveJointsRel(smoothed_velocity * delta_time, 0.0, 0.0, 0.0, 0.0, 0.0)
        except ValueError as e:
            self.get_logger().error(f"Error in MoveJointsVel: {e}")
        
def main(args=None):
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create MecademicRobotDriver node
    driver = MecademicRobotDriver()
    
    # Spin the node to keep it active
    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        # driver.get_logger().info("Shutting down Mecademic Robot Driver")
        print('disconnecting... ')
        driver.stop()
    except mdr.DisconnectError:
        """
        Sometimes CTRL-C triggers a DisconnectError from Mecademicpy before the KeyboardInterrupt exception is caught. This
        DisconnectError is directly because of the KeyboardInterrupt and causes the robot to disconnect, which we do not want, as
        we are trying to shut things down (deactivate). This function reconnects for the purpose of shutting things down completely
        so the robot isn't in a weird state:
        """
        print('disconnecting... ') # To avoid confusing the user, keeping print statements truthful to the broader goal of shutting down.
        driver.stop_after_disconnect_error()

    rclpy.try_shutdown() 

if __name__ == "__main__":
    main()
