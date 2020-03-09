#! /usr/bin/env python

## IMPORTS ##
# Basic
import rospy, actionlib, numpy, math
from enum import Enum
from pygame import mixer
import cv2
import apriltag
import serial
import re
# Intera
import intera_interface
from std_msgs.msg import (
	Bool,
	String,
	Int32,
	Float64,
	Float64MultiArray,
	UInt16,
	Empty
)
import sensor_msgs
import intera_core_msgs

# Custom
from limb_plus import LimbPlus
from sawyer.msg import *
from sawyer.srv import *
from cv_bridge import CvBridge, CvBridgeError

from signal import signal, SIGINT
from sys import exit
import threading

class Module():
	FORCE2VELOCITY = {'right_j0': 0.06, 'right_j1': 0.06, 'right_j2': 0.4, 'right_j3': 0.2, 'right_j4': 1, 'right_j5': 0.9, 'right_j6': 2}
	VERBOSE = True
	JOINTS = 7
	BUTTON = {'back': 0, 'show': 1, 'circle': 2, 'square': 3, 'triangle': 4}	# Triangle is the X button. An earlier version of Sawyer had a triangle and Rethink never updated the SDK.
	CUFF_BUTTON = {'upper': 0, 'lower': 1}
	Z_TABLE = 0.11
	BOX_HEIGHT = 0.11
	BIN_HEIGHT = 0.005

	## INITIALIZE ROS AND SAWYER ##
	def __init__(self):
		# Initialize Sawyer
		rospy.init_node('Sawyer_comm_node', anonymous=True)
		intera_interface.HeadDisplay().display_image('logo.png')

		# Limb
		self.limb = LimbPlus()
		self.limb.go_to_joint_angles()

		# Global Vars
		self.audio_duration = 0
		self.finished = False
		self.startPos = 0
		self.devMode = False
		self.allowCuffInteraction = False
		self.modeTimer = None

		self.p_command = lambda self: self.pub_num(180/math.pi*self.limb.joint_angle(shoulder))
		wheel = self.finished
		b_less = lambda self : self.limb.joint_angle(shoulder)<point_b[0]
		b_more = lambda self : self.limb.joint_angle(shoulder)>point_b[0]
		dot_2 = lambda self : self.limb.joint_angle(shoulder)<joint_dot_2[0]
		#endpoint = lambda self : self.endpoints_equal(self.limb.endpoint_pose(),dot_3,tol) or self.finished

		# Custom Control Variables
		self.control = {
			'i': 0,
			'order': 15,
			'effort': [],			# Structured self.control['effort'][tap #, e.g. i][joint, e.g. 'right_j0']
			'position': [],
			'velocity': [],
		}
		zeroVec = self.limb.joint_velocities()
		for joint in zeroVec.keys():
			zeroVec[joint] = 0.0
		for i in range(self.control['order']):
			self.control['effort'].append(zeroVec.copy())
			self.control['position'].append(zeroVec.copy())
			self.control['velocity'].append(zeroVec.copy())

		# Publishing topics
		suppress_cuff_interaction = rospy.Publisher('/robot/limb/right/suppress_cuff_interaction', Empty, queue_size=1)
		self.position_topic = rospy.Publisher('/teachbot/position', JointInfo, queue_size=1)
		self.velocity_topic = rospy.Publisher('/teachbot/velocity', JointInfo, queue_size=1)
		self.effort_topic = rospy.Publisher('/teachbot/effort', JointInfo, queue_size=1)
		self.command_complete_topic = rospy.Publisher('/teachbot/command_complete', Empty, queue_size=1)
		self.endpoint_topic = rospy.Publisher('/teachbot/EndpointInfo', EndpointInfo, queue_size=10)
		self.box_in_bin_topic = rospy.Publisher('/teachbot/box_in_bin', Bool, queue_size=1)

		# Subscribing topics
		rospy.Subscriber('/robot/joint_states', sensor_msgs.msg.JointState, self.forwardJointState)
		rospy.Subscriber('/robot/limb/right/endpoint_state', intera_core_msgs.msg.EndpointState, self.forwardEndpointState)
		rospy.Subscriber('/teachbot/camera', Bool, self.cb_camera)
		rospy.Subscriber('/teachbot/allowCuffInteraction', Bool, self.cb_allowCuffInteraction)
		rospy.Subscriber('/teachbot/button', String, self.cb_Button)

		# Service Servers
		rospy.Service('/teachbot/audio_duration', AudioDuration, self.rx_audio_duration)
		rospy.Service('/teachbot/set_robot_mode', SetRobotMode, self.cb_SetRobotMode)
		rospy.Service('/teachbot/CuffWays', CuffWays, self.cb_CuffWays)

		# Service Clients
		self.DevModeSrv = rospy.ServiceProxy('/teachbot/dev_mode', DevMode)
		self.PlayAudioSrv = rospy.ServiceProxy('/teachbot/PlayAudio', PlayAudio)

		# Actions
		self.CuffInteractionAct = actionlib.SimpleActionServer('/teachbot/CuffInteraction', CuffInteractionAction, execute_cb=self.cb_CuffInteraction, auto_start=True)
		self.GoToJointAnglesAct = actionlib.SimpleActionServer('/teachbot/GoToJointAngles', GoToJointAnglesAction, execute_cb=self.cb_GoToJointAngles, auto_start=True)
		self.InteractionControlAct = actionlib.SimpleActionServer('/teachbot/InteractionControl', InteractionControlAction, execute_cb=self.cb_interaction, auto_start=True)
		self.InteractionControlActCli = actionlib.SimpleActionClient('/teachbot/InteractionControl',InteractionControlAction)
		self.AdjustPoseToAct = actionlib.SimpleActionServer('/teachbot/AdjustPoseTo', AdjustPoseToAction, execute_cb=self.cb_AdjustPoseTo, auto_start=True)
		self.GripperAct = actionlib.SimpleActionServer('/teachbot/Gripper', GripperAction, execute_cb=self.cb_Gripper, auto_start=True)
		self.GoToCartesianPoseAct = actionlib.SimpleActionServer('/teachbot/GoToCartesianPose', GoToCartesianPoseAction, execute_cb=self.cb_GoToCartesianPose, auto_start=True)
		self.MultipleChoiceAct = actionlib.SimpleActionServer('/teachbot/MultipleChoice', MultipleChoiceAction, execute_cb=self.cb_MultipleChoice, auto_start=True)
		self.PickUpBoxAct = actionlib.SimpleActionServer('/teachbot/PickUpBox', PickUpBoxAction, execute_cb=self.cb_PickUpBox, auto_start=True)
		self.AdjustPoseByAct = actionlib.SimpleActionServer('/teachbot/AdjustPoseBy', AdjustPoseByAction, execute_cb=self.cb_AdjustPoseBy, auto_start=True)
		self.WaitAct = actionlib.SimpleActionServer('/teachbot/Wait', WaitAction, execute_cb=self.cb_Wait, auto_start=True)

		# Lights
		self.lights = intera_interface.Lights()
		for light in self.lights.list_all_lights():
			self.lights.set_light_state(light,False)

		# Navigator
		self.navigator = intera_interface.Navigator()
		self.multi_choice_callback_ids = self.BUTTON.copy()
		for key in self.multi_choice_callback_ids:
			self.multi_choice_callback_ids[key] = -1
		self.wheel_callback_id = -1
		self.wheel_state = self.navigator.get_wheel_state('right_wheel')
		self.navigator.register_callback(self.rx_finished, 'right_button_ok')
		self.navigator.register_callback(self.rx_cheat_code, 'head_button_ok')

		# Gripper
		try:
			self.gripper = intera_interface.get_current_gripper_interface()
			if isinstance(self.gripper, intera_interface.SimpleClickSmartGripper):
				if self.gripper.needs_init():
					self.gripper.initialize()
			else:
				if not (self.gripper.is_calibrated() or self.gripper.calibrate() == True):
					raise
			self.open_gripper()
		except OSError as e:
			rospy.logwarn('Failed to get gripper. No gripper attached on the robot. This may result in errors later.')

		# Cuff
		self.cuff = intera_interface.Cuff()
		self.cuff_callback_ids = self.CUFF_BUTTON.copy()
		for key in self.cuff_callback_ids:
			self.cuff_callback_ids[key] = -1

		# Turn off cuff interaction.
		if not self.allowCuffInteraction:
			rospy.Timer(rospy.Duration(0.1), lambda event=None : suppress_cuff_interaction.publish())

		# Initialization complete. Spin.
		rospy.loginfo('Ready.')
		rospy.spin()

	## HELPER FUNCTIONS ##
	# Returns true if two positions equal each other or are at least within a given tolerance
	def endpoints_equal(self, pose1, pose2, tol=0):
		equality = True
		if 'position' in pose1 and 'position' in pose2:
			equality &= abs(pose1['position'].x-pose2['position'].x)<=tol \
			   and abs(pose1['position'].y-pose2['position'].y)<=tol \
			   and abs(pose1['position'].z-pose2['position'].z)<=tol
		if 'orientation' in pose1 and 'orientation' in pose2:
			equality &= abs(pose1['orientation'].x-pose2['orientation'].x)<=tol \
			   and abs(pose1['orientation'].y-pose2['orientation'].y)<=tol \
			   and abs(pose1['orientation'].z-pose2['orientation'].z)<=tol \
			   and abs(pose1['orientation'].w-pose2['orientation'].w)<=tol
		return equality

	def set_joint_imp(self, springs, damping, ref_pos, ref_vel):
		cmd = dict()
		cur_pos = self.joint_angles()
		cur_vel = self.joint_velocities()
		for j,joint in enumerate(cur_pos.keys()):
			cmd[joint] = springs[j]*(ref_pos[joint]-cur_pos[joint])
			cmd[joint] += damping[j]*(ref_vel[joint]-cur_vel[joint])
		self.set_joint_torques(cmd)

	# Returns true iff the current joint position will not collide with the table
	def joint_safety_check(self, resetFn, returnFn):
		#rospy.loginfo('in safety zone')
		pos = self.limb.endpoint_pose()['position']
		if pos.z<0 or pos.x<0.1:
			rospy.loginfo('oops outside safety zone')
			self.limb.position_mode()
			rospy.loginfo('position_mode entered')
			self.PlayAudioSrv('safety1.mp3')
			rospy.loginfo('Audio file played')
			rospy.sleep(6.5)
			resetFn(self)
			rospy.loginfo('reset position done')
			self.PlayAudioSrv('safety2.mp3')
			rospy.loginfo('Audio file 2 played')
			returnFn(self)
			rospy.loginfo('Ready to be moved again')

	# Open gripper
	def open_gripper(self):
		if self.VERBOSE: rospy.loginfo('Gripper open triggered')
		if isinstance(self.gripper, intera_interface.SimpleClickSmartGripper):
			self.gripper.set_ee_signal_value('grip', False)
		else:
			self.gripper.open()

	# Close gripper
	def close_gripper(self):
		if self.VERBOSE: rospy.loginfo('Gripper close triggered')
		if isinstance(self.gripper, intera_interface.SimpleClickSmartGripper):
			self.gripper.set_ee_signal_value('grip', True)
		else:
			self.gripper.close()

	# Lower the gripper, close gripper, raise gripper, check effort, return whether or not an object is being supported 
	def cb_PickUpBox(self, goal, lift_position=Z_TABLE+BOX_HEIGHT+0.1, effort_tol=5):
		success = True
		result_PickUpBox = PickUpBoxResult()

		# Leave the box and lift upward to measure no-load effort
		self.limb.adjustPoseTo('position','z',lift_position)
		rospy.sleep(1)
		effort_nobox = sum(abs(effort) for effort in self.limb.joint_efforts().values())

		# Grab and lift box to measure effort under load
		self.limb.adjustPoseTo('position','z',self.Z_TABLE+self.BOX_HEIGHT/2)
		self.close_gripper()
		rospy.sleep(2)
		self.limb.adjustPoseTo('position','z',lift_position)
		rospy.sleep(1)
		effort_box = sum(abs(effort) for effort in self.limb.joint_efforts().values())
		print('no box: ' + str(effort_nobox))
		print('w/ box: ' + str(effort_box))

		hasBox = effort_box-effort_nobox>effort_tol
		if not hasBox:
			self.open_gripper()
			if self.VERBOSE: rospy.loginfo('Failed to pick up box.')
		elif self.VERBOSE:
			rospy.loginfo('Successfully picked up box.')
			#self.limb.adjustPoseTo('position','z',self.Z_TABLE+self.BOX_HEIGHT/2)
			# self.open_gripper()
			# self.limb.adjustPoseTo('position','z',self.Z_TABLE+self.BOX_HEIGHT+0.1)

		if success:
			result_PickUpBox.is_picked = hasBox
			self.PickUpBoxAct.set_succeeded(result_PickUpBox)

	# Lower the gripper, close gripper, raise gripper, check effort, return whether or not an object is being supported 

	# Comments inside check_pickup 
		'''
		del self.effort[:]

		self.startPose = self.limb.endpoint_pose()
		self.startPose_container = self.pose_to_cartclass(pose = self.startPose, cartclass = self.startPose_container)

		self.startPose_container.position_z = lift_position + .1
		self.startPose_arg = self.cartclass_to_cartarg(cartclass = self.startPose_container, cartarg = self.startPose_arg)

		self.newCartPose = self.pose_to_cartclass(pose = self.startPose, cartclass = self.newCartPose)
		self.newCartPose.position_z = lift_position
		self.newCartPose_arg = self.cartclass_to_cartarg(cartclass = self.newCartPose, cartarg = self.newCartPose_arg)
		
		self.limb.go_to_cartesian_pose(joint_angles = self.startPose_arg)
		self.effort.append(self.limb.joint_efforts())


		self.limb.go_to_cartesian_pose(joint_angles = self.newCartPose_arg)
		self.close_gripper()
		rospy.sleep(2)
		self.limb.go_to_cartesian_pose(joint_angles = self.startPose_arg)
		self.effort.append(self.limb.joint_efforts())
		if(not self.compare_efforts(effort1 = self.effort[0], effort2 = self.effort[1])):
			self.limb.go_to_cartesian_pose(joint_angles = self.newCartPose_arg)
		'''

	def display_camera_callback(self, img_data):
		rospy.sleep(0.76)
		bridge = CvBridge()
		try:
			cv_image = bridge.imgmsg_to_cv2(img_data, 'bgr8')
			# self.image_topic.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
		except CvBridgeError, err:
			rospy.logerr(err)

		try:
			img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
			detector = apriltag.Detector()
			detections = detector.detect(img)
			centerApril = detections[0].center

			cv2.line(cv_image, (int(detections[0].corners[0][0]),int(detections[0].corners[0][1])), (int(detections[0].corners[1][0]),int(detections[0].corners[1][1])), (255,255,51), 3)
			cv2.line(cv_image, (int(detections[0].corners[1][0]),int(detections[0].corners[1][1])), (int(detections[0].corners[2][0]),int(detections[0].corners[2][1])), (255,255,51), 3)
			cv2.line(cv_image, (int(detections[0].corners[3][0]),int(detections[0].corners[3][1])), (int(detections[0].corners[2][0]),int(detections[0].corners[2][1])), (255,255,51), 3)
			cv2.line(cv_image, (int(detections[0].corners[3][0]),int(detections[0].corners[3][1])), (int(detections[0].corners[0][0]),int(detections[0].corners[0][1])), (255,255,51), 3)
		except:
			rospy.loginfo('No apriltag detected')
			pass

		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		blurred = cv2.GaussianBlur(gray, (5,5), 0)
		ret, thresh = cv2.threshold(blurred, 140, 255, cv2.THRESH_BINARY_INV)

		im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

		cnt = contours[0]
		max_area = cv2.contourArea(cnt)

		for contour in contours:
			if cv2.contourArea(contour) > max_area:
				cnt = contour
				max_area = cv2.contourArea(contour)

		rect = cv2.minAreaRect(cnt)
		box = cv2.boxPoints(rect)
		box = numpy.int0(box)

		cv2.drawContours(cv_image, [box], -1, (0,255,0), 3)

		a = sum(map(lambda x: x[0], box))/4
		b = sum(map(lambda x: x[1], box))/4
		centerBin = (a,b)

		cv2.imwrite('/home/albertgo/TeachBot/browser/public/images/cv_image.png', cv_image)


		try:
			distance = ((centerBin[0]-centerApril[0])**2+(centerBin[1]-centerApril[1])**2)**0.5
			if distance < 90:
				self.box_in_bin_topic.publish(True)
				self.command_complete_topic.publish()
				rospy.loginfo('Box is in bin')
			else:
				self.box_in_bin_topic.publish(False)
				rospy.loginfo('Box out of bin')
				self.command_complete_topic.publish()
		except:
			self.box_in_bin_topic.publish(False)
			self.command_complete_topic.publish()
			rospy.loginfo('No apriltag detected')

	def addSeq(self, seq):
		self.seqArr.append(seq.asDict())

	## ROS PUBLISHERS ##
	# Publishes message to ROSTopic to be receieved by JavaScript
	def pub_cmd(self, cmd):
		msg = Int32()
		msg.data = cmd
		self.cmd2browser.publish(msg)
		if self.VERBOSE: rospy.loginfo("I sent: " + str(cmd))
	def pub_num(self, num):
		msg = Int32()
		msg.data = num
		self.wheel_delta_topic.publish(msg)
		if self.VERBOSE: rospy.loginfo("I sent: " + str(num))
	def pub_arr(self, arr):
		msg = Float64MultiArray()
		msg.data = arr
		self.array_topic.publish(msg)
		if self.VERBOSE: rospy.loginfo("I sent: " + str(arr))

	## ROS SUBSCRIBERS ##
	def rx_audio_duration(self,data):
		self.audio_duration = data.audio_duration
		return True

	def forwardJointState(self, data):
		position = JointInfo()
		velocity = JointInfo()
		effort = JointInfo()
		for j in range(Module.JOINTS):
			setattr(position, 'j'+str(j), data.position[j+1])
			self.control['position'][self.control['i']]['right_j'+str(j)] = data.position[j+1]
			setattr(velocity, 'j'+str(j), data.velocity[j+1])
			self.control['velocity'][self.control['i']]['right_j'+str(j)] = data.velocity[j+1]
			setattr(effort, 'j'+str(j), data.effort[j+1])
			self.control['effort'][self.control['i']]['right_j'+str(j)] = data.effort[j+1]
		self.control['i'] = self.control['i']+1 if self.control['i']+1<self.control['order'] else 0
		self.position_topic.publish(position)
		self.velocity_topic.publish(velocity)
		self.effort_topic.publish(effort)

	def forwardEndpointState(self, data):
		endpoint_msg = EndpointInfo()
		endpoint_msg.position.x = data.pose.position.x
		endpoint_msg.position.y = data.pose.position.y
		endpoint_msg.position.z = data.pose.position.z
		endpoint_msg.orientation.x = data.pose.orientation.x
		endpoint_msg.orientation.y = data.pose.orientation.y
		endpoint_msg.orientation.z = data.pose.orientation.z
		endpoint_msg.orientation.w = data.pose.orientation.w
		self.endpoint_topic.publish(endpoint_msg)

	def rx_finished(self,data):
		if self.VERBOSE: rospy.loginfo('Rx from arm scroll wheel button: ' + self.navigator.button_string_lookup(data) + '. finished = ' + str(self.finished))

	def rx_cheat_code(self,data):
		if self.navigator.button_string_lookup(data) == 'OFF':
			if self.devMode:
				self.devMode = False
				self.DevModeSrv(0)
				self.lights.set_light_state('head_blue_light',False)
				if self.VERBOSE: rospy.loginfo('Exiting dev mode')
			else:
				self.devMode = True
				self.DevModeSrv(1)
				self.lights.set_light_state('head_blue_light',True)
				if self.VERBOSE: rospy.loginfo('Entering dev mode')

	def cb_allowCuffInteraction(self, data):
		status = "activated." if data.data else "off."
		rospy.loginfo("Cuff interaction " + status)
		self.allowCuffInteraction = data.data

	def subscribe_to_wheel_move(self):
		def rx_wheel_move(data):
			if data-self.wheel_state==0:
				delta=0
			elif data-self.wheel_state>255/2 or (data-self.wheel_state<0 and data-self.wheel_state>-255/2):
				delta=1
			else:
				delta=-1
			self.wheel_state = data
			req = Int32()
			req.data = delta
			self.wheel_delta_topic.publish(req)
			if self.VERBOSE: rospy.loginfo('Wheel value: {0}. Wheel delta: {1}.'.format(data, delta))

		if self.VERBOSE: rospy.loginfo('Subscribing to scroll wheel.')
		self.wheel_callback_id = self.navigator.register_callback(rx_wheel_move, 'right_wheel')

	def unsubscribe_from_wheel_move(self):
		if self.wheel_callback_id==-1:
			rospy.loginfo('Already unsubscribed from wheel movement.')
		self.navigator.deregister_callback(self.wheel_callback_id)
		self.wheel_callback_id = -1

	def _unsubscribe_from(self, obj, callback_ids):
		notified = False
		for key in callback_ids:
			if self.VERBOSE and not notified:
				rospy.loginfo('Already unsubscribed from ' + str(obj))
				notified = True
			obj.deregister_callback(callback_ids[key])
			callback_ids[key] = -1

	def rx_command(self, data):
		pass

	def cb_AdjustPoseBy(self, goal):
		if self.VERBOSE: rospy.loginfo('Adjusting pose by')

		success = True
		result_AdjustPoseBy = AdjustPoseByResult()
		result_AdjustPoseBy.is_done = False

		self.limb.adjustPoseBy(goal.geometry, goal.axis, eval(goal.amount))

		if success:
			result_AdjustPoseBy.is_done = True
			self.AdjustPoseByAct.set_succeeded(result_AdjustPoseBy)

	def cb_AdjustPoseTo(self, goal):
		if self.VERBOSE: rospy.loginfo('Adjusting pose to')

		result_AdjustPoseTo = AdjustPoseToResult()
		result_AdjustPoseTo.is_done = False

		self.limb.adjustPoseTo(goal.geometry, goal.axis, eval(goal.amount))

		result_AdjustPoseTo.is_done = True
		self.AdjustPoseToAct.set_succeeded(result_AdjustPoseTo)

	def cb_Button(self, data):
		self.finished = True
		rospy.loginfo('Received: ' + str(self.finished))

	def cb_Gripper(self,goal):

		result_Gripper = GripperResult()
		result_Gripper.is_done = False

		if (goal.grip==True):
			if self.VERBOSE: rospy.loginfo('Closing gripper')
			self.close_gripper()
		else:
			if self.VERBOSE: rospy.loginfo('Opening gripper')
			self.open_gripper()

		result_Gripper.is_done = True
		self.GripperAct.set_succeeded(result_Gripper)

	def cb_camera(self, data):
		if data.data == True:
			if self.VERBOSE: rospy.loginfo('Accessing camera')

			rp = intera_interface.RobotParams()
			valid_cameras = rp.get_camera_names()
			camera = intera_interface.Cameras()
			camera.start_streaming('right_hand_camera')
			rectify_image = False
			camera.set_callback('right_hand_camera', self.display_camera_callback)
		else:
			if self.VERBOSE: rospy.loginfo('Shutting down camera')
			rp = intera_interface.RobotParams()
			valid_cameras = rp.get_camera_names()
			camera = intera_interface.Cameras()
			camera.stop_streaming('right_hand_camera')
			self.command_complete_topic.publish()

	# When the lower cuff button is pressed, allow 2D position movement.
	def cb_cuff_lower(self, data):
		rospy.loginfo('Cuff lower button pressed.')
		self.limb.interaction_control(position_x=True, position_y=True)
		feedback = sawyer.msg.CuffInteractionFeedback()
		feedback.mode = True
		self.CuffInteractionAct.publish_feedback(feedback)

	# When the upper cuff button is pressed, allow 1D orientation movement.
	def cb_cuff_upper(self, data):
		rospy.loginfo('Cuff upper button pressed.')
		self.limb.interaction_control(orientation_z=True, in_endpoint_frame=True)
		feedback = sawyer.msg.CuffInteractionFeedback()
		feedback.mode = False
		self.CuffInteractionAct.publish_feedback(feedback)

	def cb_CuffWays(self, data):
		waypoints.append(self.limb.joint_angles())
		rospy.loginfo('Adding to waypoints')
		return True

	def unsubscribe_from_cuff_interaction(self):
		self._unsubscribe_from(self.cuff, self.cuff_callback_ids)
		self.limb.position_mode()

	def cb_CuffInteraction(self, goal):
		if self.VERBOSE: rospy.loginfo('Cuff interaction engaged')
		terminatingCondition = eval(goal.terminatingCondition)
		ways = goal.ways

		# Initialize action objects
		result = sawyer.msg.CuffInteractionResult()

		
		self.cuff_callback_ids['lower'] = self.cuff.register_callback(self.cb_cuff_lower, 'right_button_lower')
		self.cuff_callback_ids['upper'] = self.cuff.register_callback(self.cb_cuff_upper, 'right_button_upper')
		
		# Continue until terminating condition is met.
		# If in waypoint mode, store each waypoint.
		self.finished = False
		if ways:
			if terminatingCondition is not None:
				while not terminatingCondition(self):
					pass
				waypoints.append(self.limb.joint_angles())
		else:
			if terminatingCondition is not None:
				while not terminatingCondition(self):
					pass
		
		# After the terminating condition is met, reset to position mode and complete action.
		rospy.loginfo('Done with cuff interaction')
		self.unsubscribe_from_cuff_interaction()
		result.done = True
		self.CuffInteractionAct.set_succeeded(result)

	def cb_GoToCartesianPose(self, goal):
		if self.VERBOSE: rospy.loginfo('Going to cartesian pose')

		success = True

		result_GoToCartesianPose = GoToCartesianPoseResult()
		result_GoToCartesianPose.is_done = False

		if self.GoToCartesianPoseAct.is_preempt_requested():
			rospy.loginfo("%s: Preempted", 'n/a')
			self.GoToCartesianPoseAct.set_preempted()
			success = False

		self.limb.go_to_cartesian_pose(position=eval(goal.position), orientation=eval(goal.orientation), relative_pose=eval(goal.relative_pose), joint_angles=eval(goal.joint_angles), endpoint_pose=eval(goal.endpoint_pose))

		if success:
			result_GoToCartesianPose.is_done = True
			self.GoToCartesianPoseAct.set_succeeded(result_GoToCartesianPose)

	def cb_GoToJointAngles(self, goal):
		if self.VERBOSE: rospy.loginfo('Going to joint angles')

		speed_ratio = 0.5 if goal.speed_ratio == 0 else goal.speed_ratio

		ways = False
		if goal.name == 'waypoints.pop(0)':
			ways = True

		result = sawyer.msg.GoToJointAnglesResult()

		if goal.name is '':
			self.limb.go_to_joint_angles([goal.j0pos, goal.j1pos, goal.j2pos, goal.j3pos, goal.j4pos, goal.j5pos, goal.j6pos], speed_ratio=speed_ratio)
			result.success = True
			self.GoToJointAnglesAct.set_succeeded(result)
		else:
			if goal.wait == True:
				startTime = rospy.get_time()
				goto = self.limb.go_to_joint_angles(eval(goal.name), speed_ratio=speed_ratio, ways = ways)

				if goto == False:
					rospy.loginfo('correct me please')

					mixer.init()
					mixer.music.load('safety3.mp3')
					mixer.music.play()
					rospy.loginfo('Audio file played')
					rospy.sleep(10.5)

					goal_InteractionControl = InteractionControlGoal(
						position_only = False,
						orientation_x = True,
						orientation_y = True,
						orientation_z = True,
						position_x = True,
						position_y = True,
						position_z = True,
						PASS = False,
						ways = False
						)
					self.InteractionControlActCli.send_goal(goal_InteractionControl)
					self.InteractionControlActCli.wait_for_result()

					mixer.init()
					mixer.music.load('safety4.mp3')
					mixer.music.play()
					rospy.loginfo('Audio file played')
					rospy.sleep(4.5)
					self.limb.go_to_joint_angles(default)

				# while(rospy.get_time()-startTime<self.audio_duration or sum(abs(velocity) for velocity in self.limb.joint_velocities().values())>0.05):
				while(sum(abs(velocity) for velocity in self.limb.joint_velocities().values())>0.05):	
					pass

			else:
				goto = self.limb.go_to_joint_angles(eval(goal.name), speed_ratio=speed_ratio, ways = ways)
				if goto == False:
					rospy.loginfo('correct me please')

					mixer.init()
					mixer.music.load('safety3.mp3')
					mixer.music.play()
					rospy.loginfo('Audio file played')
					rospy.sleep(10.5)

					goal_InteractionControl = InteractionControlGoal(
						position_only = False,
						orientation_x = True,
						orientation_y = True,
						orientation_z = True,
						position_x = True,
						position_y = True,
						position_z = True,
						PASS = False,
						ways = False
						)
					self.InteractionControlActCli.send_goal(goal_InteractionControl)
					self.InteractionControlActCli.wait_for_result()

					mixer.init()
					mixer.music.load('safety4.mp3')
					mixer.music.play()
					rospy.loginfo('Audio file played')
					rospy.sleep(4.5)
					self.limb.go_to_joint_angles(default)
			result.success = True
			self.GoToJointAnglesAct.set_succeeded(result)

	def cb_interaction(self, goal):
		if self.VERBOSE: rospy.loginfo('Free motion is on')

		result = sawyer.msg.InteractionControlResult()
		
		self.limb.interaction_control(position_only=goal.position_only, orientation_x=goal.orientation_x, orientation_y=goal.orientation_y, orientation_z=goal.orientation_z, position_x=goal.position_x, position_y=goal.position_y, position_z=goal.position_z, in_endpoint_frame=goal.in_end_point_frame)

		if goal.PASS == False:
			ang_arr = [0]*self.JOINTS
			self.finished = False
			while(not self.finished):													# Wait for user to stop moving arm
				startPose = self.limb.endpoint_pose()
				for m in range(0,10):
					for j in range(0,self.JOINTS):
						ang_arr[j] = self.limb.joint_angles()['right_j' + str(j)]
					rospy.sleep(0.1)
			self.limb.position_mode()
			result.done = True
			self.InteractionControlAct.set_succeeded(result)
		elif goal.ways == False:
			self.finished = False
			while(not self.finished):
				pass
			self.limb.position_mode()
			result.done = True
			self.InteractionControlAct.set_succeeded(result)
		else:
			self.limb.interaction_control(position_only=goal.position_only, orientation_x=goal.orientation_x, orientation_y=goal.orientation_y, orientation_z=goal.orientation_z, position_x=goal.position_x, position_y=goal.position_y, position_z=goal.position_z, in_endpoint_frame=goal.in_end_point_frame)
			self.finished = False
			while not self.finished:
				pass
			waypoints.append(self.limb.joint_angles())
			self.limb.position_mode()
			result.done = True
			self.InteractionControlAct.set_succeeded(result)

	def cb_Wait(self, goal):

		result_wait = sawyer.msg.WaitResult()
		result_wait.success = False

		if goal.timeout<0:
			rospy.loginfo('Timeout not specified! Wait function cannot wait for forever.')
			raise
		startTime = rospy.get_time()

		if goal.what=='effort':
			startEffort = sum(abs(effort) for effort in self.limb.joint_efforts().values())
			while (rospy.get_time()-startTime<goal.timeout and not result_wait.success):
				result_wait.success = abs(startEffort-sum(abs(effort) for effort in self.limb.joint_efforts().values()))>0.1*startEffort
				rospy.sleep(0.1)
		else:
			pass

		if not result_wait.success:
			if rospy.get_time()-startTime>=goal.timeout:
				rospy.loginfo('Wait callback: timeout!')
			else:
				rospy.loginfo('Wait callback: failed to receive an input or wait not properly formatted.')
		else:
			rospy.loginfo('Wait callback: success!')

		self.WaitAct.set_succeeded(result_wait)

	def unsubscribe_from_multi_choice(self):
		if self.multi_choice_callback_ids[self.BUTTON.keys()[0]]==-1:
			rospy.loginfo('Already unsubscribed from multiple choice callbacks.')
		self._unsubscribe_from(self.navigator, self.multi_choice_callback_ids)

	def multiple_choice_button_pressed(self, name, data):
		if self.navigator.button_string_lookup(data) == 'OFF':
			if self.VERBOSE: rospy.loginfo(name + ' button pressed.')
			self.multiple_chocie_result = sawyer.msg.MultipleChoiceResult()
			self.multiple_chocie_result.answer = self.BUTTON[name];
			#self.unsubscribe_from_multi_choice()
			self.multiple_choice_selected = True

	def cb_MultipleChoice(self, goal):
		if self.VERBOSE: rospy.loginfo('Multiple choice ready')

		self.multiple_choice_selected = False

		for key in self.BUTTON:
			self.multi_choice_callback_ids[key] = self.navigator.register_callback(lambda data, key=key, self=self : self.multiple_choice_button_pressed(key, data), 'right_button_' + key)
			print(self.multi_choice_callback_ids[key])

		while not self.multiple_choice_selected:
			pass
		self.MultipleChoiceAct.set_succeeded(self.multiple_chocie_result)

	def cb_SetRobotMode(self, req):
		if self.VERBOSE: rospy.loginfo('Entering ' + req.mode + ' mode.')

		if not (self.modeTimer is None):
			self.modeTimer.shutdown()

		if req.mode == 'position':
			self.limb.exit_control_mode()
			self.limb.go_to_joint_angles(self.limb.joint_angles())
			if req.ways:
				rospy.loginfo('Setting waypoint')
				waypoints.append(self.limb.joint_angles())

		elif req.mode == 'admittance ctrl':
			# Set command timeout to be much greater than the command period
			self.limb.set_command_timeout(2)

			# Initialize Joints Dict
			joints = {};
			for j in req.joints:
				joints['right_j'+str(j)] = {}

			# Set min_thresh specs
			if len(req.min_thresh)!=0:
				for i,j in enumerate(req.joints):
					joints['right_j'+str(j)]['min_thresh'] = req.min_thresh[i]
			else:
				for j in joints.keys():
					joints[j]['min_thresh'] = 0

			# Set bias specs
			if len(req.bias)!=0:
				for i,j in enumerate(req.joints):
					joints['right_j'+str(j)]['bias'] = req.bias[i]
			else:
				for j in joints.keys():
					if j==shoulder:
						joints[j]['bias'] = BIAS_SHOULDER
					elif j==elbow:
						joints[j]['bias'] = BIAS_ELBOW
					elif j==wrist:
						joints[j]['bias'] = BIAS_WRIST
					else:
						joints[j]['bias'] = 0

			# Set F2V specs
			if len(req.F2V)!=0:
				for i,j in enumerate(req.joints):
					joints['right_j'+str(j)]['F2V'] = req.F2V[i]
			else:
				for j in joints.keys():
					joints[j]['F2V'] = self.FORCE2VELOCITY[j]

			self.modeTimer = rospy.Timer(rospy.Duration(0.1), lambda event=None : self.cb_AdmittanceCtrl(joints, eval(req.resetPos)))

		elif req.mode == 'impedance ctrl':
			# Set command timeout to be much greater than the command period
			self.limb.set_command_timeout(2)

			# Initialize Joints Dict
			joints = {};
			for joint in self.limb.joint_efforts().keys():
				joints[joint] = {}

			# Set V2F and X2F specs
			for joint in self.limb.joint_efforts().keys():
				joints[joint]['V2F'] = 5 if joint==shoulder else 10
				joints[joint]['X2F'] = 160 if joint==shoulder else 150
			for i,j in enumerate(req.joints):
				joints['right_j'+str(j)]['V2F'] = req.V2F[i]
				joints['right_j'+str(j)]['X2F'] = req.X2F[i]

			# Set position and velocity reference points
			x_ref = self.limb.joint_angles()
			for joint in self.limb.joint_efforts().keys():
				joints[joint]['x_ref'] = x_ref[joint]
				joints[joint]['v_ref'] = 0

			self.modeTimer = rospy.Timer(rospy.Duration(0.02), lambda event=None : self.cb_ImpedanceCtrl(joints, eval(req.resetPos)))

		elif req.mode == 'interaction ctrl':
			d = self.limb.interaction_control.func_defaults
			self.limb.interaction_control(position_only=req.position_only,
				                          orientation_only=req.orientation_only, 
				                          plane_horizontal=req.plane_horizontal,
				                          plane_vertical_xz=req.plane_vertical_xz,
				                          plane_vertical_yz=req.plane_vertical_yz,
				                          nullspace_only=req.nullspace_only,
				                          position_x=req.position_x,
				                          position_y=req.position_y,
				                          position_z=req.position_z,
				                          orientation_x=req.orientation_x,
				                          orientation_y=req.orientation_y,
				                          orientation_z=req.orientation_z,
				                          constrained_axes=d[12] if len(req.constrained_axes)==0 else req.constrained_axes,
				                          in_endpoint_frame=req.in_endpoint_frame,
				                          interaction_frame=d[14] if len(req.interaction_frame)==0 else req.interaction_frame,
				                          K_nullspace=d[15] if len(req.K_nullspace)==0 else req.K_nullspace,
				                          rate=d[16] if req.rate==0 else req.rate)

		else:
			rospy.logerr('Robot mode ' + req.mode + ' is not a supported mode.')

		return True

	def cb_AdmittanceCtrl(self, joints, resetPos, rateNom=10, tics=15):
		self.joint_safety_check(lambda self : self.limb.go_to_joint_angles(resetPos), lambda self : None)
		rospy.Publisher('/robot/joint_state_publish_rate',UInt16,queue_size=10).publish(rateNom)		# Set publish rate

		# Filter effort and convert into velocity
		velocities = self.limb.joint_velocities()
		for joint in velocities.keys():
			if joint in joints.keys():
				allForces = [self.control['effort'][i][joint] for i in range(self.control['order'])]
				filteredForce = sum(allForces)/self.control['order']
				filteredForce = filteredForce + joints[joint]['bias']
				if abs(filteredForce) < joints[joint]['min_thresh']:
					velocities[joint] = 0
				else:
					velocities[joint] = -joints[joint]['F2V']*filteredForce
			else:
				velocities[joint] = 0

		self.limb.set_joint_velocities(velocities)

	def cb_ImpedanceCtrl(self, joints, resetPos, rateNom=50):
		self.joint_safety_check(lambda self : self.limb.go_to_joint_angles(resetPos), lambda self : None)
		rospy.Publisher('/robot/joint_state_publish_rate',UInt16,queue_size=10).publish(rateNom)		# Set publish rate

		x = self.limb.joint_angles()
		v = self.limb.joint_velocities()
		efforts = self.limb.joint_efforts()
		for joint in efforts.keys():
			efforts[joint] = joints[joint]['X2F']*(joints[joint]['x_ref']-x[joint]) + joints[joint]['V2F']*(joints[joint]['v_ref']-v[joint])

		self.limb.set_joint_torques(efforts)

	def cb_InteractionCtrl(self):
		pass

	# Allows user to move arm in zero G mode
	def user_move(self):
		startPose = self.limb.endpoint_pose()
		self.limb.interaction_control(orientation_x=False, orientation_y=False, orientation_z=False, position_x=True, position_y=True, position_z=True)
		while(self.endpoints_equal(startPose,self.limb.endpoint_pose(),tol=0.01)):	# Wait for user to begin moving arm
			pass
		rospy.sleep(0.5)
		while(not self.endpoints_equal(startPose,self.limb.endpoint_pose())):		# Wait for user to stop moving arm
			startPose = self.limb.endpoint_pose()
			rospy.sleep(1)
		self.limb.position_mode()

class Mode(Enum):
	POSITION = 1			# Fixed-position joint control.
	ADMITTANCE_CTRL = 2		# Custom admittance control given torque-speed conversion factor.
	IMPEDANCE_CTRL = 3		# Custom impedance control given springs and dampers.
	INTERACTION_CTRL = 4	# Built-in zero-G mode in various axes.

## DEFINE IMPORTANT CONSTANTS ##
if __name__ == '__main__':
	# Commonly used objects
	default = [0.0, -0.78, 0.0, 1.57, 0, -0.79, 0.2]
	joint_buttons = [0.0, -0.78, 0.0, 1.57, 0, -2.99, -1.37]

	BIAS_SHOULDER = -0.55#-0.5
	BIAS_ELBOW = 0.4
	BIAS_WRIST = -0.15
	#shoulder_wrist_bias = {shoulder: BIAS_SHOULDER, wrist: BIAS_WRIST}
	#shoulder_wrist_thresh = {shoulder: 1.0, wrist: 0.5}

	# Joint offset angles in order to align properly
	j6_offset = 0.2

	# Joint Names
	shoulder = 'right_j0'
	elbow = 'right_j3'
	wrist = 'right_j5'

	# Joint Positions
	scara_j1_clearance = -0.06	# Adjust j1 to avoid scraping table in SCARA
	no_hit = -0.04
	scara_j6_gripper = 1.80		# constant j6 during SCARA mode to keep the gripper horizontal
	DSP = -0.10		# Default shoulder position in SCARA
	j2max = 3.04	# Angle limit
	j4max = 2.99	# The maximum joint angle achievable by right_j4 and _j5
	j5min = -2.99
	j5max = 1.57	# Prevent from stretching the tube
	j2scara = math.pi/2 - math.pi+j4max
	j6scara = -3*math.pi/2 + math.pi-j4max

	dof_demo_rotation = math.pi/8.0

	# 1
	joint_motor_animation_0 = [0]*Module.JOINTS
	joint_motor_animation_0[0] = 0.3
	joint_motor_animation_0[6] = j6_offset
	joint_motor_animation_1 = joint_motor_animation_0[:]
	joint_motor_animation_1[4] = j4max

	# 4
	joint_test = [0]*Module.JOINTS
	for j in range(0,Module.JOINTS):
		joint_angle_arr = joint_motor_animation_0[:]
		if (j==1 or j==3 or j==5):
			joint_angle_arr[j] = -0.7
		else:
			joint_angle_arr[j] = 0.7
		joint_test[j] = joint_angle_arr
	
	# 6-15
	joint_dof_start = [0.78, scara_j1_clearance, 1.57, -1.57, 2.99, -0.78, scara_j6_gripper]
	joint_dof_shoulder = [0.78, scara_j1_clearance, 1.57, -1.57, 2.99, -0.78, scara_j6_gripper]
	joint_dof_shoulder[0] += dof_demo_rotation
	joint_dof_elbow = [0.78, scara_j1_clearance, 1.57, -1.57, 2.99, -0.78, scara_j6_gripper]
	joint_dof_elbow[3] += dof_demo_rotation
	joint_dof_wrist = [0.78, scara_j1_clearance, 1.57, -1.57, 2.99, -0.78, scara_j6_gripper]
	joint_dof_wrist[5] += dof_demo_rotation
	joint_dof_up = [DSP,-0.3,j2scara,0,-j4max,0,j6scara]

	# In between these two parts, showing the encoder video
	pos_encoder_video = [0.0, -1.57, 0.0, 1.57, 0, 0, 0.2]

	# 18-24
	point_a = [0.60, scara_j1_clearance, 1.57, -1.40, 2.99, 0.0, 1.80]
	point_b = [0.60, scara_j1_clearance, 1.57, -1.40, 2.99, 0.0, 1.80]
	point_c = [0.60, scara_j1_clearance, 1.57, -1.40, 2.99, 0.0, 1.80]
	point_b[3] = -1.87
	point_c[3] = -2.17

	# 25
	joint_push_down = [0, 0, 0, -2.2, -2.97, 0, -math.pi/2+j6_offset]
	#joint_push_down = [math.pi/2,0,0,math.pi/2,0,0,0]

	kinematics_init_pos = [0.90060,-0.05870,1.57015,-1.56932,2.97598,-1.57,1.79896]
	kinematics_shoulder_arc = [0.90060,-0.05870,1.57015,-2.2,2.97598,-1.57,1.79896]

	# 32-37
	joint_dot_1 = kinematics_init_pos[:]
	joint_dot_1[5] = -0.3
	# joint_dot_1 = [DSP,no_hit,j2scara,0,-j4max,-1.83,j6scara]
	joint_dot_2 = kinematics_init_pos[:]
	joint_dot_2[3] = -2.2
	# joint_dot_2 = [-0.21,no_hit,j2scara,0,-j4max,0,j6scara]
	joint_arc_wrist = joint_dot_2[:]
	joint_arc_wrist[5] = -0.6
	# joint_arc_wrist = [joint_dot_2[0],no_hit,j2scara,0,-j4max,-j4max,j6scara]
	joint_ortho_kin = joint_arc_wrist[:]
	# joint_ortho_kin[5] = -math.pi/2
	joint_arc_elbow = joint_ortho_kin[:]
	joint_arc_elbow[3] = -1.6
	# joint_arc_shoulder = joint_ortho_kin[:]
	# joint_arc_shoulder[0] = -0.45

	joint_dot_3_init = kinematics_init_pos[:]
	joint_dot_3_init[0] = 1.1

	# 42
	dot_3 = {'position': intera_interface.Limb.Point(x=0.59,y=0.56,z=0.05)}
	joint_dot_3 = [1.10041,-0.05880,1.56815,-0.99882,2.97123,-0.00150,1.79917]
	# joint_dot_3 = [0.13,no_hit,j2scara,0,-j4max,-0.07,j6scara]
	joint_dot_3_2_1 = joint_dot_3_init[:]
	joint_dot_3_2_1[3] = joint_dot_3[3]

	# 46
	joint_dot_3_4_2 = joint_dot_3_init[:]
	joint_dot_3_4_2[3] = (joint_dot_3_init[3]+joint_dot_3[3])/2
	joint_dot_3_4_2[5] = (joint_dot_3_init[5]+joint_dot_3[5])/2
	joint_dot_3_4_1 = joint_dot_3_init[:]
	joint_dot_3_4_1[3] = joint_dot_3_4_2[3]
	joint_dot_3_4_3 = joint_dot_3_4_2[:]
	joint_dot_3_4_3[3] = joint_dot_3[3]

	# 47
	joint_dot_3_8_4 = joint_dot_3_4_2[:]
	joint_dot_3_8_2 = joint_dot_3_init[:]
	joint_dot_3_8_2[3] = (joint_dot_3_init[3]+joint_dot_3_8_4[3])/2
	joint_dot_3_8_2[5] = (joint_dot_3_init[5]+joint_dot_3_8_4[5])/2
	joint_dot_3_8_6 = joint_dot_3_8_4[:]
	joint_dot_3_8_6[3] = (joint_dot_3_8_4[3]+joint_dot_3[3])/2
	joint_dot_3_8_6[5] = (joint_dot_3_8_4[5]+joint_dot_3[5])/2
	joint_dot_3_8_1 = joint_dot_3_init[:]
	joint_dot_3_8_1[3] = joint_dot_3_8_2[3]
	joint_dot_3_8_3 = joint_dot_3_8_2[:]
	joint_dot_3_8_3[3] = joint_dot_3_8_4[3]
	joint_dot_3_8_5 = joint_dot_3_8_4[:]
	joint_dot_3_8_5[3] = joint_dot_3_8_6[3]
	joint_dot_3_8_7 = joint_dot_3_8_6[:]
	joint_dot_3_8_7[3] = joint_dot_3[3]

	# 49-
	waypoints = []

	# 58
	joint_high_two = [0.0, 0.3, 0.0, -1.08, 0.0, -0.78, 0.2]

	## MODULE 2 ##
	# init_joint_arg                = [1.133,-0.678481445312,-0.433721679687,1.33986621094,-0.763953125,-1.132484375,0.959416015625]
	avoid_collision				  = [0.48652,-0.27409,-1.40738,1.64292,0.23778,-0.10091,0.17191]
	fail_init_joint_arg           = [0.83073,-0.11565,-1.25283,1.24106,1.35105,1.28988,-0.27901]
	success_pickup_box1           = [0.84542,-0.08553,-1.34804,1.27302,1.43411,1.36175,-1.82399]

	above_first_bin_joint_arg     = [1.14708,0.02910,-1.40753,1.92801,1.64855,1.42886,-2.23213]

	above_second_box_joint_arg    = [0.67078,-0.00905,-1.43319,1.59981,1.55063,1.39695,-2.37913]
	above_third_box_joint_arg     = [0.29264,-0.21117,-1.25342,1.56628,1.30664,1.24663,-1.04227]
	above_fourth_box_joint_arg    = [0.49833,-0.21497,-1.28520,2.12842,1.45044,1.20956,-3.03918]

	camera_pos                    = [0.5832646484375, -1.301193359375, -0.193248046875, 2.0165146484375, 0.0075908203125, -0.75755078125, 0.3351982421875] # Temporary pose. This is not working yet.
	camera_pos2					  = [0.21234375, 0.2708505859375, -1.70441796875, 1.7846484375, 1.9352724609375, -0.17507421875, -3.083966796875]

	Module()