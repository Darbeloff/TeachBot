#! /usr/bin/env python
#testing various python functions and ros communication via rosbridge
import os, rospy, math, pexpect
from std_msgs.msg import (
	String,
	Int32
)

# Intera
import intera_interface 
from intera_interface import (
    SimpleClickSmartGripper,
    get_current_gripper_interface,
    Lights,
    Cuff,
    RobotParams
)

# Custom
import go_to
import waypoint
import time
import zeroG
#from module import (Module, Sequence)
from sawyer import Module

'''
file_save = True
module_number = 2
int_msg = Int32()
int_msg.data = 0
grip_msg = Int32()
grip_msg.data = 1
unique_msg = String()
unique_msg.data = ''	
z_table = 0.117974138485
gripper_to_base_length = .08
box1_height = 0.110
box2_height = 0.110
box3_height = 0.110
bin_depth = 0.005
bin_height = 0.085
half_gripper_width = 0.015
gripper_width = 0.03
lift_position = z_table + box1_height - gripper_width
'''
#rostopic pub -r 10 /robot/limb/right/suppress_cuff_interaction std_msgs/Empty

class cartPose(object):
	def __init__(self, pos_x=None, pos_y=None, pos_z=None, ori_x=None, ori_y=None, ori_z=None, ori_w = None):
		self.position_x = pos_x
		self.position_y = pos_y
		self.position_z = pos_z
		self.orientation_x = ori_x
		self.orientation_y = ori_y
		self.orientation_z = ori_z
		self.orientation_w = ori_w

class jointAngles(object):
	def __init__(self, j0 = None, j1 = None, j2 =None, j3=None, j4 = None, j5 = None, j6 = None):
		self.j0 = j0
		self.j1 = j1
		self.j2 = j2
		self.j3 = j3
		self.j4 = j4
		self.j5 = j5
		self.j6 = j6

class listener(Module):
	'''
	def __init__(self):
		#self.interaction_pub = rospy.Publisher('/interaction_topic', Int32, queue_size = 10)
		self.startPose = self.limb.endpoint_pose() 
		self.startPose_container = cartPose()
		self.startPose_arg = go_to.cartesian_pose_arg()
		self.startJointAngles = self.limb.joint_angles()
		self.newJointAngles_arg = go_to.joint_angle_arg()
		self.newCartPose_arg = go_to.cartesian_pose_arg()
		self.newCartPose_arg2 = go_to.cartesian_pose_arg()
		self.newCartPose = cartPose()
		self.newJointAngles = jointAngles()
		self.waypoint_init_container = cartPose()
		self.waypoint_init_arg = go_to.cartesian_pose_arg()

		#Check files and set text file to save to
		if(file_save):
			self.home = os.path.expanduser('~')
			self.dir_path = self.home + '/Learner_Responses/module' + str(module_number) + '/'
			if not os.path.exists(self.dir_path):
				os.makedirs(self.dir_path)

			self.file_number = 0
			self.file_path = self.dir_path + 'test_' + str(self.file_number) + '.txt'
			while(os.path.exists(self.file_path)):
				self.file_number += 1
				self.file_path = self.dir_path + 'test_' + str(self.file_number) + '.txt'
			self.file = open(self.file_path, 'w')
			self.file.write('pretest \r\n' + str(self.old_time) + '\r\n')
			self.file.close()

		Module.__init__(self)
	'''

	def compare_efforts(self, effort1, effort2, tol_effort = 2.8):
		return abs(effort1['right_j6']-effort2['right_j6'])<=tol_effort \
			   and abs(effort1['right_j5']-effort2['right_j5'])<=tol_effort \
			   and abs(effort1['right_j4']-effort2['right_j4'])<=tol_effort \
			   and abs(effort1['right_j3']-effort2['right_j3'])<=tol_effort \
			   and abs(effort1['right_j2']-effort2['right_j2'])<=tol_effort \
			   and abs(effort1['right_j1']-effort2['right_j1'])<=tol_effort \
			   and abs(effort1['right_j0']-effort2['right_j0'])<=tol_effort

	def endpoints_equal(self, pose1, pose2, tol=0.01):
		return abs(pose1['position'].x-pose2['position'].x)<=tol \
			   and abs(pose1['position'].y-pose2['position'].y)<=tol \
			   and abs(pose1['position'].z-pose2['position'].z)<=tol \
			   and abs(pose1['orientation'].x-pose2['orientation'].x)<=tol \
			   and abs(pose1['orientation'].y-pose2['orientation'].y)<=tol \
			   and abs(pose1['orientation'].z-pose2['orientation'].z)<=tol \
			   and abs(pose1['orientation'].w-pose2['orientation'].w)<=tol

	def pose_to_cartclass(self, pose, cartclass):
		cartclass.position_x = pose['position'].x
		cartclass.position_y = pose['position'].y
		cartclass.position_z = pose['position'].z
		cartclass.orientation_x = pose['orientation'].x
		cartclass.orientation_y = pose['orientation'].y
		cartclass.orientation_z = pose['orientation'].z
		cartclass.orientation_w = pose['orientation'].w
		return cartclass

	def cartclass_to_cartarg(self, cartclass, cartarg):
		cartarg = go_to.cartesian_pose_arg(position = [cartclass.position_x, cartclass.position_y, cartclass.position_z],
			orientation = [cartclass.orientation_x, cartclass.orientation_y, cartclass.orientation_z, cartclass.orientation_w])
		return cartarg

	def limbangles_to_angleclass(self, limbangles, angleclass):
		angleclass.j0 = limbangles['right_j0']
		angleclass.j1 = limbangles['right_j1']
		angleclass.j2 = limbangles['right_j2']
		angleclass.j3 = limbangles['right_j3']
		angleclass.j4 = limbangles['right_j4']
		angleclass.j5 = limbangles['right_j5']
		angleclass.j6 = limbangles['right_j6']
		return angleclass

	def angleclass_to_anglearg(self, angleclass, anglearg):
		anglearg = go_to.joint_angle_arg(joint_angles = [angleclass.j0, angleclass.j1, angleclass.j2, angleclass.j3, angleclass.j4, angleclass.j5, angleclass.j6])
		return anglearg

	def navigator_callback(self, data):
		self.new_time = time.time()
		self.time_elapsed = self.new_time - self.old_time
		self.file = open(self.file_path, 'a')
		self.file.write('time stamp: ' + str(self.time_elapsed) + '\r\n' + 'Navigator: ' + str(data.data) + '\r\n')
		self.file.close()
		if(self.learner_interaction and data.data == 'Button \'OK\': OFF'):
			self.learner_interaction = False
			zeroG.constrained(zeroG_all_constraints)
			self.interaction_counter = self.interaction_counter + 1
			self.total_interactions = self.total_interactions + 1
			int_msg.data = self.interaction_counter
			self.interaction_pub.publish(int_msg)
			rospy.sleep(0.1)

	def rx_command(self, data):
		seq = Sequence(data.data)
		if self.VERBOSE: rospy.loginfo("I heard: " + str(seq.idn))

		if seq.idn==0:
			self.limb.go_to_joint_angles(init_joint_arg)
			
			self.addSeq(seq)
			self.pub_cmd(seq.idn)

		elif seq.idn==1:
			#Go to gripping position
			self.limb.go_to_joint_angles(fail_init_joint_arg)
			self.limb.adjustPoseTo('position','z',Module.Z_TABLE+Module.BOX_HEIGHT)

			#Grip
			self.close_gripper()

			#fail and then lift up
			#Open gripper
			rospy.sleep(1)
			self.limb.go_to_cartesian_pose(joint_angles = fail_init_joint_arg)
			self.open_gripper()
			
			self.addSeq(seq)
			self.pub_cmd(seq.idn)

		elif seq.idn==2:
			self.finished = False
			self.subscribe_to_cuff_interaction(lambda self: self.finished)

			if self.check_pickup():
				#did lift the shoe.
				seq.addAction('pickup_success')
				self.addSeq(seq)
				self.pub_cmd(seq.idn)
			else:
				#did not lift the shoe. Efforts are the same.
				self.open_gripper()
				seq.addAction('pickup_fail')
				self.addSeq(seq)
				self.pub_cmd(seq.idn+1)

		elif seq.idn==4:
			self.finished = False
			self.subscribe_to_cuff_interaction(lambda self: self.finished)
			self.open_gripper()

			self.addSeq(seq)
			self.pub_cmd(seq.idn)

		elif seq.idn==5:
			rospy.loginfo('Module Complete.')
			rospy.signal_shutdown('Module Complete.') 

if __name__ == '__main__':
	#variables to be used
	init_joint_arg                = [1.133,-0.678481445312,-0.433721679687,1.33986621094,-0.763953125,-1.132484375,0.959416015625]
	fail_init_joint_arg           = [1.0454140625,-0.629208007812,-1.01547070313,1.05442871094,-2.38241699219,-1.48850390625,-1.18359277344]
	fail_pickup_cart_arg          = [1.08375488281,0.175158203125,-1.53774609375,1.02942480469,-1.45563085938,-1.45510351563,1.86297558594]
	fail_above_pickup_cart_arg    = [1.06155175781,-0.26884765625,-1.4501015625,0.838840820312,-1.88630175781,-1.62122851562,1.9701875]

	joint_buttons                 = [math.pi/2-0.0393427734375,-0.71621875,0.022359375,1.811921875,-0.116017578125,2.77036035156,-4.48708789063]

	success_init_joint_arg        = [1.05569238281,-0.176821289062,-1.62947851563,0.904107421875,-1.66179296875,-1.63658007812,0.266885742187]
	success_pickup_joint_arg      = [1.06658789062,0.264276367188,-1.62918945312,1.05704492187,-1.31784570312,-1.50530078125,0.266266601562]
	success_pickup_cart_arg       = [1.06658789062,0.264276367188,-1.62918945312,1.05704492187,-1.31784570312,-1.50530078125,0.266266601562]
	success_above_pickup_cart_arg = [1.05569238281,-0.176821289062,-1.62947851563,0.904107421875,-1.66179296875,-1.63658007812,0.266885742187]

	above_bin_cart_arg            = [0.935443359375,-0.182680664062,-1.34264160156,1.20748339844,-1.85780664063,-1.45737597656,3.1062734375]
	rotated_above_bin_cart_arg    = [0.938936523438,-0.166234375,-1.35050976563,1.20908984375,-1.8427265625,-1.403859375,1.51214648438]
	into_bin_cart_arg             = [0.943045898437,0.233333984375,-1.50463964844,1.30310449219,-1.3893203125,-1.43581445312,1.4121640625]

	into_second_bin_cart_arg      = [0.561075195313,0.2966484375,-1.81974609375,0.759317382813,-1.23418261719,-1.49326855469,0.08444140625]
	above_second_bin_cart_arg     = [0.5035625,0.0966318359375,-1.882125,0.744194335937,-1.22447363281,-1.62413378906,0.002185546875]

	above_second_box_cart_arg     = [math.pi/2-0.729661132813,0.12510546875,-1.78256054688,1.094546875,-1.35895410156,-1.71990722656,4.66472753906]
	above_third_box_cart_arg      = [0.317999023437,0.0488310546875,-1.646234375,-0.249951171875,-1.51264550781,-1.61024121094,-2.44349414062]
	above_fourth_box_cart_arg     = [math.pi/2-1.482765625,0.17649609375,-2.27079101562,0.363586914062,-0.855272460937,-1.6052578125,1.5651328125]

	zeroG_endpoint_constraints = zeroG.constrained_arg(orientation_z=True, in_endpoint_frame = True)
	zeroG_all_constraints = zeroG.constrained_arg(orientation_x = False, orientation_y = False, orientation_z = False, position_x = False, position_y = False, position_z = False)
	zeroG_ori_constraints = zeroG.constrained_arg(orientation_x=False, orientation_y=False, orientation_z=False, position_x=True, position_y=True, position_z=True)
	zeroG_pos_constraints = zeroG.constrained_arg(orientation_x=True, orientation_y=True, orientation_z=True, position_x=False, position_y=False, position_z=False)
	zeroG_truly_no_constraints = zeroG.constrained_arg(orientation_x=True, orientation_y=True, orientation_z=True, position_x=True, position_y=True, position_z=True)
	zeroG_xyplane = zeroG.constrained_arg(plane_horizontal = True)

	listener()