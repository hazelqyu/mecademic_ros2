# robot_controller.py
class RobotController:
    def __init__(self, robot):
        self.robot = robot
        self.current_joint_angles = self.get_current_joint_positions()
        self.checkpoint_counter = 1  # Initialize a checkpoint counter

    def get_current_joint_positions(self):
        """
        Fetch the current real-time joint positions from the robot.
        :return: List of current joint angles [q1, q2, q3, q4, q5, q6]
        """
        response = self.robot.GetRobotRtData(synchronous_update=True)  # Assumes this function returns a list of joint positions
        current_joint_positions = [
            response.rt_joint_pos.data[0],
            response.rt_joint_pos.data[1],
            response.rt_joint_pos.data[2],
            response.rt_joint_pos.data[3],
            response.rt_joint_pos.data[4],
            response.rt_joint_pos.data[5]
        ]
        return current_joint_positions

    def check_limits(self, joint_angles):
        """
        Ensure the joint angles are within their specified limits.
        :param joint_angles: List of joint angles [q1, q2, q3, q4, q5, q6]
        :return: List of limited joint angles
        """
        limited_angles = [max(-175, min(175, joint_angles[0])),  # θ1
                          max(-70, min(90, joint_angles[1])),  # θ2
                          max(-135, min(70, joint_angles[2])),  # θ3
                          max(-170, min(170, joint_angles[3])),  # θ4
                          max(-115, min(115, joint_angles[4])),  # θ5
                          joint_angles[5]]  # θ6 has software limits ±100 turns

        return limited_angles

    def set_checkpoint(self):
        """
        Sets a checkpoint in the robot's motion queue to track the completion of movements.
        """
        self.robot.SetCheckpoint(self.checkpoint_counter)
        print(f"Checkpoint {self.checkpoint_counter} set.")
        self.checkpoint_counter += 1
        if self.checkpoint_counter > 8000:
            self.checkpoint_counter = 1  # Reset checkpoint counter if it exceeds the limit

    def turn_right(self):
        joint_angles = self.check_limits([90, 0, 0, 0, 0, 0])
        self.robot.MoveJoints(*joint_angles)
        self.set_checkpoint()
        self.current_joint_angles = self.get_current_joint_positions()

    def turn_left(self):
        joint_angles = self.check_limits([-90, 0, 0, 0, 0, 0])
        self.robot.MoveJoints(*joint_angles)
        self.set_checkpoint()
        self.current_joint_angles = self.get_current_joint_positions()

    def move_to_home(self):
        joint_angles = self.check_limits([0, 0, 0, 0, 0, 0])
        self.robot.MoveJoints(*joint_angles)
        self.set_checkpoint()
        self.current_joint_angles = self.get_current_joint_positions()

    def move_up(self):
        joint_angles = self.check_limits([
            self.current_joint_angles[0], self.current_joint_angles[1] - 45,
                                          self.current_joint_angles[2] + 45, self.current_joint_angles[3],
            self.current_joint_angles[4], self.current_joint_angles[5]])
        self.robot.MoveJoints(*joint_angles)
        self.set_checkpoint()
        self.current_joint_angles = self.get_current_joint_positions()

    def move_down(self):
        joint_angles = self.check_limits([
            self.current_joint_angles[0], self.current_joint_angles[1] + 45,
                                          self.current_joint_angles[2] - 45, self.current_joint_angles[3],
            self.current_joint_angles[4], self.current_joint_angles[5]])
        self.robot.MoveJoints(*joint_angles)
        self.set_checkpoint()
        self.current_joint_angles = self.get_current_joint_positions()

    def turn_to_degree(self, degree):
        """
        Turns the robot to a specified degree.
        :param degree: float Degree to turn to (-175 to 175)
        """
        if degree < -175 or degree > 175:
            raise ValueError("Degree must be between -175 and 175")

        # Replace q1 with the desired degree while keeping other joints as is
        joint_angles = self.check_limits([
            degree, self.current_joint_angles[1],
            self.current_joint_angles[2], self.current_joint_angles[3],
            self.current_joint_angles[4], self.current_joint_angles[5]])
        self.robot.MoveJoints(*joint_angles)
        self.set_checkpoint()
        self.current_joint_angles = self.get_current_joint_positions()

    def move_joints(self,degrees):
        joint_angles = self.check_limits(degrees)
        self.robot.MoveJoints(*joint_angles)
        self.set_checkpoint()
        self.current_joint_angles = self.get_current_joint_positions()
        
    def move_joints_rel(self, delta_angles):
        """
        Moves the robot joints by the specified relative amounts.
        :param delta_angles: List of relative joint displacements [Δq1, Δq2, Δq3, Δq4, Δq5, Δq6]
        """
        # Calculate new target positions
        new_joint_angles = [
            self.current_joint_angles[0] + delta_angles[0],
            self.current_joint_angles[1] + delta_angles[1],
            self.current_joint_angles[2] + delta_angles[2],
            self.current_joint_angles[3] + delta_angles[3],
            self.current_joint_angles[4] + delta_angles[4],
            self.current_joint_angles[5] + delta_angles[5]
        ]

        # Apply limit checks
        checked_joint_angles = self.check_limits(new_joint_angles)

        # Calculate new delta angles respecting the limits
        new_delta_angles = [
            checked_joint_angles[0] - self.current_joint_angles[0],
            checked_joint_angles[1] - self.current_joint_angles[1],
            checked_joint_angles[2] - self.current_joint_angles[2],
            checked_joint_angles[3] - self.current_joint_angles[3],
            checked_joint_angles[4] - self.current_joint_angles[4],
            checked_joint_angles[5] - self.current_joint_angles[5]
        ]

        # Move joints relatively
        self.robot.MoveJointsRel(*new_delta_angles)
        self.set_checkpoint()
        self.current_joint_angles = self.get_current_joint_positions()

    def check_velocity_limits(self, joint_velocities):
        """
        Ensure the joint velocities are within their specified limits based on robot version.
        :param joint_velocities: List of joint velocities [q̇1, q̇2, q̇3, q̇4, q̇5, q̇6]
        :return: List of limited joint velocities
        """

        velocity_limits = [225, 225, 225, 350, 350, 500] # Meca500 R4
        limited_velocities = [
            max(-velocity_limits[0], min(velocity_limits[0], joint_velocities[0])),
            max(-velocity_limits[1], min(velocity_limits[1], joint_velocities[1])),
            max(-velocity_limits[2], min(velocity_limits[2], joint_velocities[2])),
            max(-velocity_limits[3], min(velocity_limits[3], joint_velocities[3])),
            max(-velocity_limits[4], min(velocity_limits[4], joint_velocities[4])),
            max(-velocity_limits[5], min(velocity_limits[5], joint_velocities[5]))
        ]

        return limited_velocities

    def move_joints_vel(self, joint_velocities):
        """
        Moves the robot joints by the specified velocities.
        :param joint_velocities: List of joint velocities [q̇1, q̇2, q̇3, q̇4, q̇5, q̇6]
        """
        # Check velocity limits
        checked_velocities = self.check_velocity_limits(joint_velocities)

        # Move joints with velocities
        self.robot.MoveJointsVel(*checked_velocities)
        self.set_checkpoint()
