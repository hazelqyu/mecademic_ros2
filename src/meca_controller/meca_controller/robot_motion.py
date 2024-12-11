# robot_motion.py

import math
import time
import random

class RobotMotion:
    def __init__(self, robot, music_player, logger=None):
        self.robot = robot
        self.music_player = music_player
        self.logger = logger
    
    def dance(self):
        if self.music_player.songs[2].is_paused_state():
                self.music_player.unpause_song(2)
        else:
                self.music_player.play_song(2)
        self.robot.SetJointVel(45)
        # self.robot.SetJointAcc(150)
        time_start = time.time()
        duration = 0
        while duration<10:
            time_now = time.time()
            duration = time_now-time_start
            self.robot.MoveJoints(math.degrees(math.cos(2 * math.pi * 0.1 * time_now)),
                                math.degrees(-0.2+0.5*math.sin(2 * math.pi * 0.5 * (time_now-0.5))),
                                math.degrees(-0.45+0.5*math.sin(2 * math.pi * 0.5 * (time_now-1))),
                                math.degrees(2.5*math.sin(2 * math.pi * 0.1 * (time_now-1.5))),
                                math.degrees(0.3*math.sin(2 * math.pi * 0.5 * (time_now-1.5))),
                                0)
            time.sleep(0.08)
        self.robot.WaitIdle(timeout=60)
        self.music_player.pause_song(2)
        
    def yawn(self,current_joint_1=0):
        self.robot.SetJointAcc(15)
        self.robot.SetJointVelLimit(25)
        self.robot.MoveJoints(current_joint_1, 0, -90, 0, -35, 0)
        self.robot.SetJointAcc(7.5)
        self.robot.SetJointVelLimit(15)
        self.robot.MoveJoints(current_joint_1, 20, 20, 0, 30, 0)
        self.robot.WaitIdle(timeout=60)
        self.robot.SetJointAcc(15)
        self.robot.SetJointVelLimit(80)
        
    def alert(self,current_joint_1 = 0, newest_face_deg = [0,0,0,0,0,0]):
        self.robot.SetJointVel(120)
        # self.robot.SetJointAcc(150)
        # TODO: should face the new face
        self.robot.MoveJoints(newest_face_deg[0],newest_face_deg[1],newest_face_deg[2],newest_face_deg[3],newest_face_deg[4],newest_face_deg[5])
        self.robot.MoveJoints(current_joint_1, -60, 20, 0, 0, 0)
        self.robot.WaitIdle()
        time.sleep(0.25)        
        self.robot.MoveJoints(current_joint_1-10, -60, 20, -40, 0, 0)
        self.robot.WaitIdle()
        time.sleep(0.25)
        self.robot.MoveJoints(current_joint_1+10, -60, 20, 40, 0, 0)
        self.robot.WaitIdle()
        time.sleep(0.25)
        self.robot.MoveJoints(current_joint_1, -60, 20, 0, 0, 0)
        self.robot.WaitIdle(timeout=60)
        self.robot.SetJointAcc(15)
        self.robot.SetJointVelLimit(80)
        
    def dash(self,current_joint_1=0):
        self.robot.SetJointAcc(150)
        self.robot.MoveJoints(current_joint_1, -40, 25, 0, 30, 0)
        self.robot.WaitIdle()
        self.robot.SetJointVel(120)
        self.robot.MoveJoints(current_joint_1, 40, -30, 0, -35, 0)
        self.robot.MoveJoints(current_joint_1, -40, 25, 0, 30, 0)
        self.robot.WaitIdle(timeout=60)
        self.robot.SetJointAcc(15)
        self.robot.SetJointVelLimit(80)
    
    def chomp(self,current_joint_1=0):
        self.robot.SetJointVel(45)
        # self.robot.SetJointAcc(150)
        time_start = time.time()
        duration = 0
        while duration<8:
            time_now = time.time()
            duration = time_now-time_start
            self.robot.MoveJoints(current_joint_1,-50,-10,0,30 * math.sin(2*math.pi*2*time_now+ math.pi),0)
            time.sleep(0.08)

    
    def dance2(self):
        if self.music_player.songs[2].is_paused_state():
                self.music_player.unpause_song(2)
        else:
                self.music_player.play_song(2)
        self.robot.SetJointVel(90)
        # self.robot.SetJointAcc(150)
        time_start = time.time()
        duration = 0
        amplitude_j2 = 30  # Amplitude for joint 2
        amplitude_j3 = 40  # Amplitude for joint 3
        amplitude_j5 = 20
        amplitude_j4 = 20
        frequency = 2 
        while duration<10:
            time_now = time.time()
            duration = time_now-time_start
            # Calculate joint positions based on sine wave
            j2 = -10+amplitude_j2 * math.sin(2 * math.pi * frequency * time_now)  # Sine wave for joint 2
            j3 = -10+amplitude_j3 * math.sin(2 * math.pi * frequency * time_now + math.pi)
            j4 = amplitude_j4 * math.sin(2 * math.pi * 4 * time_now + math.pi)
            j5 = amplitude_j5 * math.sin(2*math.pi*frequency*time_now+ math.pi)
            self.robot.MoveJoints(0, j2, j3, j4, j5, 0)
            time.sleep(0.08)
        self.robot.WaitIdle(timeout=60)
        self.music_player.pause_song(2)