#!/usr/bin/env python

import rospy
import numpy as np
import sensor_msgs.msg as smsg
from std_msgs.msg import Bool, Int32, Float64, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from controller_manager_msgs.srv import SwitchController, ListControllers
from ur_msgs.srv import SetSpeedSliderFraction
import actionlib

from ur.msg import *
from ur.srv import *

SCARA = [0, -3.14, 0, -3.14, -1.57, 0]
ROTATE = [0, -3.14, 1.0, -3.14, -1.57, 0]
ZERO = [0, -1.57, 0, -1.57, 0, 0]
pos_left = [0, -0.78, -0.78, -1.57, 0, 0]
pos_right = [0, -2.35, 0.78, -1.57, 0, 0]
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

class VelocityClient:

    def __init__(self):
        self.moving = False
        # sending to group vel trajectory
        rospy.Subscriber('/joint_states', JointState, self.send_velocities)
        self.publish_velocity = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=1)
                
        rospy.Subscriber('/joint_group_vel_controller/command',Float64MultiArray, self.cb_vel_controller)

        rospy.wait_for_service('/controller_manager/switch_controller')
        self.switch_controller = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)
        rospy.wait_for_service('/controller_manager/list_controllers')
        self.list_controllers = rospy.ServiceProxy('/controller_manager/list_controllers', ListControllers)

        self.set_speed_slider_srv = rospy.ServiceProxy('/ur_hardware_interface/set_speed_slider', SetSpeedSliderFraction)

        try:
            res = self.switch_controller(['joint_group_vel_controller'],['scaled_pos_traj_controller'], 2, True, 10.0)
            # res = self.switch_controller(['joint_group_vel_controller'], ['scaled_pos_traj_controller'], 2)
            if res.ok:
                rospy.loginfo("Switched to joint_group_vel_controller")
        except rospy.ServiceException, e:
            print "Service call failed to switch to scaled_pos_traj_controller"
            rospy.logerr("Service call failed to switch to joint_group_vel_controller")

    def send_velocities(self, data):
        ## Code setting speed slider, only functional when TP set to 'remote control' mode.
        # print 'press enter'
        # user_input = raw_input()
        # try:
        #     user_input = float(user_input)
        # except:
        #     pass
        # if(isinstance(user_input, float)):
        #     if (user_input>=0.02 and user_input<=1):
        #         res = self.set_speed_slider_srv(user_input)
        #         print(res)
        #         if res.success:
        #             print("Success, speed slider set to {}%".format(user_input*100))
        #         else:
        #             print("speed slider service failure")

        velocity_msg = Float64MultiArray()
        velocity_msg.data = [0, 0, -0.0, 0, 0, 0]
        pos_elbow = data.position[2]

        # if (abs(pos_elbow)<0.78 and not self.moving):
        #     velocity_msg.data[2] = 0.
        # else:

            
        self.publish_velocity.publish(velocity_msg)

    def cb_vel_controller(self, msg):
        # print(msg.data)
        pass

if __name__ == '__main__':
    rospy.init_node('request')
    pc = VelocityClient()

    try:              
        rospy.spin()

    except rospy.ROSInterruptException:
        pass




