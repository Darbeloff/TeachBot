#!/usr/bin/env python

import rospy
import numpy as np
import sensor_msgs.msg as smsg
from std_msgs.msg import Bool, Int32, Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import actionlib

SCARA = [0, -3.14, 0, -3.14, -1.57, 0]
ROTATE_WRIST = [0, -3.14, 0, -3.14, -1.57, -1]
ROTATE_WRIST_BACK = [0, -3.14, 0, -3.14, -1.57, 0]
ZERO = [0, -1.57, 0, -1.57, 0, 0]
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

class PositionClient:

    def __init__(self):
        self.initialized = False 

        rospy.Subscriber('/ur_hardware_interface/robot_program_running', Bool, self.cb_robot_ready)
        rospy.Subscriber('/joint_states', JointState, self.send_command)

        self.client = actionlib.SimpleActionClient('/scaled_pos_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        
    def cb_robot_ready(self, res):
        if not self.initialized:
            self.initialized = res.data
            if self.initialized:
                print "inside ready and initialized"
                rospy.wait_for_service("set_speed_slider")
                print "connected to service"
                try:
                    send_speed = rospy.ServiceProxy("set_speed_slider", Float64)
                    send_speed(0.5)
                    print send_speed.success
                
                except:
                    print "could not connect to service"
                

    def send_command(self, joints):
        print 'press enter'
        pause = raw_input()

        self.client.wait_for_server()

        followJoint_msg = FollowJointTrajectoryGoal()

        traj_msg = JointTrajectory()
        traj_msg.joint_names = JOINT_NAMES

        jointPositions_msg = JointTrajectoryPoint()
        jointPositions_msg.positions = ZERO
        jointPositions_msg.time_from_start = rospy.Duration(3)

        traj_msg.points = [jointPositions_msg,]
        followJoint_msg.trajectory = traj_msg

        self.client.send_goal(followJoint_msg)
        self.client.wait_for_result()

if __name__ == '__main__':
    rospy.init_node('request')
    pc = PositionClient()

    try:              
        rospy.spin()

    except rospy.ROSInterruptException:
        pass




