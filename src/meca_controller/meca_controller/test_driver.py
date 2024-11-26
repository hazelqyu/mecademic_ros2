#! /usr/bin/env python3

import mecademicpy.robot as mdr
from meca_controller.robot_controller import RobotController
from custom_interfaces.msg import RobotStatus, SingleJointState
from custom_interfaces.srv import ClearMotion, ExecuteMotion
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose,Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Bool
import time
import math


def singleton(cls):
    """A decorator to make a class a singleton."""
    instances = {}  # Dictionary to store the single instance

    def get_instance(*args, **kwargs):
        if cls not in instances:
            instances[cls] = cls(*args, **kwargs)
        return instances[cls]

    return get_instance


@singleton
class MecademicRobotDriver(Node):
    def __init__(self):
        super().__init__("mecademic_robot_driver")

        self.robot = mdr.Robot()
        self.robot.Connect(address='192.168.0.100')
        self.robot.ResumeMotion()
        self.robot.ActivateAndHome()
        # Provide all available real-time feedback messages
        self.robot.SetRealTimeMonitoring('all')
        # self.robot.SetBlending(80)
        self.robot.SetJointVel(25)
        self.controller = RobotController(self.robot)
        self.robot.WaitIdle(timeout=60)
        print('ready.')
        
        # Set monitoring interval for real time data:
        self.MONITORING_INTERVAL = 0.1 # seconds TODO ADJUST THIS
        self.robot.SetMonitoringInterval(self.MONITORING_INTERVAL)
        DATA_LOGGING_TIME_INTERVAL = 0.1 # seconds TODO ADJUST THIS
        self.create_timer(DATA_LOGGING_TIME_INTERVAL, self.timed_data_logging_callback)
        
        # Publisher feedback
        self.feedback_publisher = self.create_publisher(JointState, "/joint_states", 10)
        
        # Timer to update command
        # self.create_timer(0.05,self.send_command)
        
        # Set up subscriber to recieve ROS command
        self.single_joint_state_subscriber = self.create_subscription(SingleJointState, "/mecademic_single_joint", self.update_single_joint_state,10)
        self.joint_subscriber = self.create_subscription(Float64MultiArray, "/mecademic_robot_joint_array", self.joint_array_callback, 10)
        self.joint_subscriber = self.create_subscription(JointState, "/mecademic_robot_joint", self.joint_callback, 10)
        self.joint_rel_subscriber = self.create_subscription(JointState, "/mecademic_robot_joint_rel", self.joint_rel_callback,10)
        self.joint_vel_subscriber = self.create_subscription(Twist,"/cmd_vel",self.set_joint_vel_callback,10)
        
        # create services
        self.state_change_srv = self.create_service(ClearMotion, "/state_change", self.state_change)
        self.motion_srv = self.create_service(ExecuteMotion,'/execute_motion',self.execute_motion_callback)
        
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
        
        self.joint_desired_state=[0,0,0,0,0,0] #radius
        self.joint_current_state = [0,0,0,0,0,0] #radius

        
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
        
        status = RobotStatus()
        status.activation_state = robot_status.activation_state
        status.brakes_engaged = robot_status.brakes_engaged
        status.end_of_block_status = robot_status.end_of_block_status
        status.error_status = robot_status.error_status
        status.estop_state = robot_status.estopState
        status.homing_state = robot_status.homing_state
        status.pause_motion_status = robot_status.pause_motion_status
        status.pstop2_state = robot_status.pstop2State
        status.recovery_mode = robot_status.recovery_mode
        status.simulation_mode = robot_status.simulation_mode
        
        # print(status.end_of_block_status)
        
        self.is_in_error = robot_status.error_status
        if self.is_in_error:
            self.handle_error()
        
        # self.get_logger().info("Publishing joint states")

        data = self.robot.GetRobotRtData(synchronous_update=True) 
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['meca_axis_1_joint','meca_axis_2_joint','meca_axis_3_joint','meca_axis_4_joint','meca_axis_5_joint','meca_axis_6_joint']
        joint_state.position = [
            math.radians(data.rt_joint_pos.data[0]),
            math.radians(data.rt_joint_pos.data[1]),
            math.radians(data.rt_joint_pos.data[2]),
            math.radians(data.rt_joint_pos.data[3]),
            math.radians(data.rt_joint_pos.data[4]),
            math.radians(data.rt_joint_pos.data[5])
        ]
        self.joint_current_state = joint_state.position
        # joint_state.velocity = [data.rt_joint_vel.data[0],data.rt_joint_vel.data[1],data.rt_joint_vel.data[2],
        #                         data.rt_joint_vel.data[3],data.rt_joint_vel.data[4],data.rt_joint_vel.data[5]]
        self.feedback_publisher.publish(joint_state)
    
    # Callback function to update a single joint state
    def update_single_joint_state(self,msg): 
        self.joint_desired_state[msg.idx] = msg.position
        # TODO: relative update
    
    def test_queue(self):
        self.robot.WaitHomed()
        self.robot.MoveJoints(0,0,0,0,0,0)
        # self.robot.MoveLin(200, 0, 300, 0, 90, 0)
        # self.robot.MoveLin(200, 100, 300, 0, 90, 0)
        # self.robot.MoveLin(200, 100, 100, 0, 90, 0)
        # self.robot.MoveLin(200, -100, 100, 0, 90, 0)
        # self.robot.MoveLin(200, -100, 300, 0, 90, 0)
        # self.robot.MoveLin(200, 0, 300, 0, 90, 0)
        # self.robot._set_eob(True)
        
    def send_command(self):
        joint_positions_deg = [math.degrees(pos) for pos in self.joint_desired_state]
        self.controller.move_joints(joint_positions_deg)

    def joint_array_callback(self,array):
        joint_positions_deg = [math.degrees(pos) for pos in array.data]
        
        self.robot.MoveJoints(joint_positions_deg[0], joint_positions_deg[1], joint_positions_deg[2],
                              joint_positions_deg[3], joint_positions_deg[4], joint_positions_deg[5])
    
    def state_change(self,request, response):
        try:
            self.robot.ClearMotion()
            self.robot.WaitMotionCleared()
            response.success = True
            self.get_logger().info("Motion buffer cleared")
        except Exception as e:
            response.success = False
            self.get_logger().error(f"Failed to clear motion buffer: {e}")
        return response

    def execute_motion_callback(self,request,response):
        try:
            self.get_logger().info()(f"Executing motion:{request.motion_name}")
            if request.motion_name == "yawn":
                self.test_yawn()
                response.success = True
            else:
                self.get_logger().warn(f"Unknown motion: {request.motion_name}")
                response.success = False
        except Exception as e:
            self.get_logger().error(f"Motion execution failed: {e}")
            response.success = False

        return response
    
    def joint_callback(self, joints):
        joint_positions_deg = [math.degrees(pos) for pos in joints.position]
        # self.controller.move_joints(joint_positions_deg)
        print(joint_positions_deg)
        self.robot.MoveJoints(joint_positions_deg[0], joint_positions_deg[1], joint_positions_deg[2],
                              joint_positions_deg[3], joint_positions_deg[4], joint_positions_deg[5])
    
    def joint_rel_callback(self,joints_rel):
        # comes in degrees for now
        self.robot.MoveJointsRel(joints_rel.position[0],joints_rel.position[1],joints_rel.position[2],
                                 joints_rel.position[3],joints_rel.position[4],joints_rel.position[5],)
    
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
    
    def test_wave(self):
        while rclpy.ok():
            time_now = self.get_clock().now().nanoseconds * 1e-9
            sine_value = 0.1*math.sin(2*math.pi*0.5*time_now)
            print("sine_value:",sine_value)
            self.robot.MoveJoints(0,math.degrees(sine_value),0,0,-math.degrees(sine_value),0)
            time.sleep(0.08)
    
    def test_yawn(self):
        self.robot.SetJointAcc(10)
        self.robot.SetJointVelLimit(15)
        self.robot.MoveJoints(0, 0, -90, 0, 0, 0)
        self.robot.MoveJoints(0, 0, 0, 0, 0, 0)
        self.robot.WaitIdle(timeout=60)
        self.robot.SetJointVel(25)
    
def main(args=None):
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create MecademicRobotDriver node
    driver = MecademicRobotDriver()
    driver.test_queue()
    # driver.test_wave()
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
