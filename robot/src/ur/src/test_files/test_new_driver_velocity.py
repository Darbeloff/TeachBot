#!/usr/bin/env python

import rospy
import numpy as np
import sensor_msgs.msg as smsg
from std_msgs.msg import Bool, Int32, Float64, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import actionlib

from ur.msg import *
from ur.srv import *

SCARA = [0, -3.14, 0, -3.14, -1.57, 0]
ROTATE = [0, -3.14, 1.0, -3.14, -1.57, 0]
ZERO = [0, -1.57, 0, -1.57, 0, 0]
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

class VelocityClient:

    def __init__(self):
        # sending to group vel trajectory
        rospy.Subscriber('/joint_states', JointState, self.send_velocities)
        self.publish_velocity = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=1)
                
    def send_velocities(self, joints):
        print 'press enter'
        pause = raw_input()
        velocity_msg = Float64MultiArray()
        velocity_msg.data = [0, -.5, 0, 0, 0, 0]

        self.publish_velocity.publish(velocity_msg)

if __name__ == '__main__':
    rospy.init_node('request')
    pc = VelocityClient()

    try:              
        rospy.spin()

    except rospy.ROSInterruptException:
        pass




