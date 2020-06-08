#!/usr/bin/env python

'''
The raw force inputs are filtered in the cb_forward_wrench callback function. It
is called when there is a new data coming from topic /wrench.
The control loop that sends velocity commands to the robot is done in the cb_loop
callback function, which is called when a new joint state is received from the
robot.
Both functions above run at 500 Hz because that is the rate at which the robot
publishes joint states and force measurements to respective topics.
'''


import rospy
import numpy as np
import math, random
from scipy import signal
import sensor_msgs.msg as smsg
from std_msgs.msg import Bool, Int32, Float64, Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import WrenchStamped
from ur_msgs.srv import SetSpeedSliderFraction
import actionlib

from ur.msg import *
from ur.srv import *

# Pre-defined joint positions.
ZERO = [0, -1.57, 0, -1.57, 0, 0]
# IMPE_INIT = [0, -1.57, -1.57, -1.57, -1.57, 0]
IMPE_INIT = [1.57, -3.14, 1.57, -1.57, 1.57, 0]
GRIPPER_DOWN = [0, -1.57, 0, 0, -1.57, 0]
GRIPPER_UP = [0, -1.57, 0, 0, 1.57, 0]

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
POS_CONTROLLER = "scaled_pos_traj_controller"
VEL_CONTROLLER = "joint_group_vel_controller"

class ImpedanceControl:
	def __init__(self):
		self.initialized = False
		self.current_controller = POS_CONTROLLER

		self.wrench_order = 59
		self.wrench_i = 0;
		self.collect_wrench_data = True

		self.wrench_x_raw = [0] * self.wrench_order;
		self.wrench_y_raw = [0] * self.wrench_order;
		self.wrench_z_raw = [0] * self.wrench_order;

		self.wrench_x = 0
		self.wrench_y = 0
		self.wrench_z = 0

		self.maf_z = 0

		# butterworth filter element containers
		self.butter_order = 15
		self.wrench_z_in = [0]
		self.wrench_z_out = [0]
		self.b, self.a = signal.butter(self.butter_order, 0.2)
		# w, h = signal.freqz(self.b, self.a)
		# import matplotlib.pyplot as plt
		# fig, ax1 = plt.subplots()
		# ax1.set_title('Digital filter frequency response')
		# ax1.plot(w, 20 * np.log10(abs(h)), 'b')
		# ax1.set_ylabel('Amplitude [dB]', color='b')
		# ax1.set_xlabel('Frequency [rad/sample]')
		# ax2 = ax1.twinx()
		# angles = np.unwrap(np.angle(h))
		# ax2.plot(w, angles, 'g')
		# ax2.set_ylabel('Angle (radians)', color='g')
		# ax2.grid()
		# ax2.axis('tight')
		# plt.show()

		self.ftsensor_angle = 0
		self.vel_order = 7
		self.vel_raw = [0] * self.vel_order
		self.vel_i = 0

		self.joint_position = [0] * 6

		rospy.init_node('impedance')

		self.vel_cmd_pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=1)
		self.butter_fz_pub = rospy.Publisher('/butter_force_z', Float64, queue_size=1)
		self.maf_fz_pub = rospy.Publisher('/maf_force_z', Float64, queue_size=1)
		self.raw_fz_pub = rospy.Publisher('/raw_force_z', Float64, queue_size=1)
		self.elbow_vel_pub = rospy.Publisher('/elbow_vel', Float64, queue_size=1)

		rospy.Subscriber('/ur_hardware_interface/robot_program_running', Bool, self.cb_robot_ready)
		rospy.Subscriber('/joint_states', JointState, self.cb_loop)
		rospy.Subscriber('/wrench', WrenchStamped, self.cb_forward_wrench)
		rospy.Subscriber('/terminal_command', String, self.cb_terminal_command)

		self.cumulative = 0
		self.freq = 100
		self.t_int = 1.0/100
		rospy.Subscriber('/test_loop', Float64, self.cb_loop_TEST)
		self.pub_out = rospy.Publisher('/test_out', Float64, queue_size=1)

		rospy.wait_for_service('/controller_manager/switch_controller')
		self.switch_controller_srv = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)

		self.pos_cmd_act = actionlib.SimpleActionClient('/scaled_pos_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

		self.switch_controller('pos')

		# self.cb_switch_controller('vel')

		self.initialize_position()


	def cb_loop_TEST(self, data):
		self.cumulative += t_int
		sine = math.sin(2*math.PI*self.cumulative)
		self.pub_out.publish(sine)


	def initialize_position(self):

		self.send_position(IMPE_INIT);
		self.switch_controller('vel')


	def cb_forward_wrench(self, data):
		if self.collect_wrench_data:
			# self.collect_wrench_data = False
			### Moving average filter
			i = self.wrench_i
			self.wrench_x_raw[i] = data.wrench.force.x
			self.wrench_y_raw[i] = data.wrench.force.y
			self.wrench_z_raw[i] = data.wrench.force.z
			self.maf_z = sum(self.wrench_z_raw)/self.wrench_order
			self.wrench_i+=1
			if (self.wrench_i==self.wrench_order):
				self.wrench_i = 0
			
			### Butterworth filter
			output = self.b[0] * data.wrench.force.z
			for i in range(len(self.wrench_z_in)):
				output += self.b[i+1] * self.wrench_z_in[i] - self.a[i+1] * self.wrench_z_out[i]
			output /= self.a[0]
			# The most recent data is kept at the front, and data not used by filter is
			# discarded from the end. This is to make indexing in multiplication loop above easier.
			self.wrench_z_in.insert(0, data.wrench.force.z)
			self.wrench_z_out.insert(0, output)
			if len(self.wrench_z_in) > self.butter_order:
				self.wrench_z_in.pop()
				self.wrench_z_out.pop()
			self.butter_fz_pub.publish(output)
			self.maf_fz_pub.publish(self.maf_z)
			self.raw_fz_pub.publish(data.wrench.force.z)
			self.raw_fz_pub.publish(data.wrench.force.z)

		else:
			self.collect_wrench_data = True


	def switch_controller(self, target_controller):
		if (target_controller == VEL_CONTROLLER or target_controller == 'vel'):
			res = self.switch_controller_srv([],[POS_CONTROLLER,VEL_CONTROLLER], 1, True, 10.0)
			res = self.switch_controller_srv([VEL_CONTROLLER],[], 1, True, 10.0)
			self.current_controller = VEL_CONTROLLER
			rospy.loginfo('Switched to velocity controller.')
		elif (target_controller == POS_CONTROLLER or target_controller == 'pos'):
			res = self.switch_controller_srv([],[POS_CONTROLLER,VEL_CONTROLLER], 1, True, 10.0)
			res = self.switch_controller_srv([POS_CONTROLLER],[], 1, True, 10.0)
			self.current_controller = POS_CONTROLLER
			rospy.loginfo('Switched to position controller.')
		else:
			rospy.logwarn('Invalid command!')

	def cb_switch_controller(self, target_controller):
		self.switch_controller(target_controller.data)


	def send_position(self, position):
		if (self.current_controller != POS_CONTROLLER):
			rospy.logwarn('Not in position controller, cannot send position command')
		else:
			self.pos_cmd_act.wait_for_server()

			followJoint_goal = FollowJointTrajectoryGoal()
			jointTraj_msg = JointTrajectory()
			trajPoint_msg = JointTrajectoryPoint()

			# As of now, use if-else cases to check position command. In the future,
			# this will be replaced by numbers parsed from JSON data.
			if (type(position) is list):
				trajPoint_msg.positions = position
			else:	
				if (position == 'IMPE_INIT'):
					trajPoint_msg.positions = IMPE_INIT
				elif (position == 'ZERO'):
					trajPoint_msg.positions = ZERO

			trajPoint_msg.time_from_start = rospy.Duration(3)

			jointTraj_msg.joint_names = JOINT_NAMES
			jointTraj_msg.points = [trajPoint_msg,]

			followJoint_goal.trajectory = jointTraj_msg

			self.pos_cmd_act.send_goal(followJoint_goal)
			self.pos_cmd_act.wait_for_result()

	def cb_send_position(self, position):
		self.send_position(position.data)


	def cb_send_velocity(self, velocity):
		if (self.current_controller != VEL_CONTROLLER):
			rospy.logwarn('Not in velocity controller, cannot send velocity command')
		else:
			# as of now, this method is not active because sending vel cmd is not simple through terminal.
			pass


	def cb_robot_ready(self, res):
		while (not self.initialized):
			self.initialized = res.data
		print("Robot initialized.")


	def cb_loop(self, res):

		F_threshold = 5
		F_cutoff = 2 * F_threshold

		for val in res.position:
			self.joint_position[res.position.index(val)] = val
		base = self.joint_position[2]
		self.joint_position[2] = self.joint_position[0]
		self.joint_position[0] = base

		# Note that for now, this angle is valid only at IMPE_INIT pose.
		# For a more generalized application, you should use forward kinematics.
		self.ftsensor_angle = self.joint_position[2]
		theta_i = math.pi/2
		f_max = 8.7
		bias_z = -math.sin(self.ftsensor_angle - theta_i) * f_max

		# Calculate force measurement in z direction, offset by bias_z

		# self.wrench_x = sum(self.wrench_x_raw)/self.wrench_order
		# self.wrench_y = sum(self.wrench_y_raw)/self.wrench_order
		# self.wrench_z = sum(self.wrench_z_raw)/self.wrench_order - bias_z
		force_input = self.maf_z - bias_z
		temp_force_input = self.maf_z - bias_z

		if (abs(force_input) < F_threshold/2):
			force_input = 0.0
			temp_force_input = 0.0
		
		# 1. Converting position deviation to resistance force
		joint_to_move = [2] # specify one joint from 0-5, 0 being base.
		position_ref = IMPE_INIT[2]
		delta_pos = self.ftsensor_angle - position_ref

		Kpf = 45
		P2F = Kpf * delta_pos
		# Eliminate vibration at zero position.
		if (abs(P2F) < 0.5):
			P2F = 0.0

		# ##### Impedance control implemented here #####
		force_input += P2F

		# 2. Velocity portion that simulates motion caused by pushing
		Kfv = -0.059
		# F2V = Kfv * self.wrench_z
		if abs(force_input) < F_cutoff:
			F2V = Kfv / (2 * F_cutoff) * force_input * abs(force_input)
			temp_f2v = Kfv / (2 * F_cutoff) * temp_force_input * abs(force_input)
		else:
			# F2V = Kfv / (2 * F_cutoff) * F_cutoff ** 2 * abs(self.wrench_z)/self.wrench_z
			if (force_input < 0):
				F2V = Kfv * (force_input + F_threshold)
				temp_f2v = Kfv * (temp_force_input + F_threshold)
			else:
				F2V = Kfv * (force_input - F_threshold)
				temp_f2v = Kfv * (temp_force_input - F_threshold)

		# ####### Change Kpv to put it back to impedance mode #######
		vel_elbow_target = F2V

		if self.vel_i == self.vel_order:
			self.vel_i = 0
		self.vel_raw[self.vel_i] = vel_elbow_target
		self.vel_i += 1
		vel_elbow_target = sum(self.vel_raw)/self.vel_order

		velocity_msg = Float64MultiArray()
		velocity_msg.data = [0, 0, vel_elbow_target, 0, 0, 0]
		self.elbow_vel_pub.publish(vel_elbow_target)
		print(temp_force_input)

		if (self.current_controller == VEL_CONTROLLER):
			self.vel_cmd_pub.publish(velocity_msg)
			pass
		else:
			# print("wrong controller.")
			pass


	def cb_terminal_command(self, data):
		if(data.data == 'pos'):
			self.switch_controller('pos')
		elif (data.data == 'vel'):
			self.switch_controller('vel')
		elif (data.data == 'zero'):
			if (self.current_controller == POS_CONTROLLER):
				self.send_position('ZERO')
			else:
				rospy.logwarn('Not in position controller.')
		elif (data.data == 'impe'):
			if (self.current_controller == POS_CONTROLLER):
				self.send_position('IMPE_INIT')
			else:
				rospy.logwarn('Not in position controller.')
		else:
			rospy.logwarn('command {} not valid (yet).'.format(data.data))


if __name__ == '__main__':
    
    ic = ImpedanceControl()

    try:              
        rospy.spin()

    except rospy.ROSInterruptException:
        pass