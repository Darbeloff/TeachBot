# Parts of this file were written using code licensed under the following:
# 
# Copyright (c) 2016-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

## IMPORTS ##
# Basic
import rospy, math, PyKDL, pygame
from std_msgs.msg import String
from geometry_msgs.msg import (
	PoseStamped,
	Pose
)

# Intera
import intera_interface
from intera_interface import (
	Limb,
	Lights,
	CHECK_VERSION
)
from intera_motion_msgs.msg import (
	TrajectoryOptions
)
from intera_core_msgs.msg import InteractionControlCommand
from intera_motion_interface import (
	MotionTrajectory,
	MotionWaypoint,
	MotionWaypointOptions,
	InteractionOptions
)
from intera_motion_interface.utility_functions import (int2bool, bool2int, boolToggle)
from intera_core_msgs.srv import (
	SolvePositionFK,
	SolvePositionFKRequest
)
from sensor_msgs.msg import JointState

class LimbPlus(Limb):
	def __init__(self, VERBOSE=False):
		super(LimbPlus, self).__init__()
		self.VERBOSE = VERBOSE
		self.nav = intera_interface.Navigator()
		self.icc_pub = rospy.Publisher('/robot/limb/right/interaction_control_command', InteractionControlCommand, queue_size = 1)

	def go_to_joint_angles(self, joint_angles = [0.0, -0.78, 0.0, 1.57, 0, -0.79, 0.2], speed_ratio = 0.5, accel_ratio = 0.5, timeout = None, fail_plan = None):
		# Old home position: [math.pi/2, -0.9, 0.0, 1.8, 0.0, -0.9, 0.0]
		try:
			if isinstance(joint_angles, dict):
				joint_angles_arr = [0]*len(joint_angles)
				for j in range(len(joint_angles_arr)):
					joint_angles_arr[j] = joint_angles['right_j' + str(j)]
				joint_angles = joint_angles_arr
			traj = MotionTrajectory(limb = self)
			wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=speed_ratio,max_joint_accel=accel_ratio)
			waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = self)
			joint_angles_start = self.joint_ordered_angles()
			waypoint.set_joint_angles(joint_angles = joint_angles_start)
			traj.append_waypoint(waypoint.to_msg())
			if len(joint_angles) != len(joint_angles_start):
				rospy.logerr('The number of joint_angles must be %d', len(joint_angles_start))
				return None
			waypoint.set_joint_angles(joint_angles = joint_angles)
			traj.append_waypoint(waypoint.to_msg())
			result = traj.send_trajectory(timeout=timeout)
			if result is None:
				rospy.logerr('Trajectory FAILED to send')
				return False
			if result.result:
				if self.VERBOSE: rospy.loginfo('Motion controller successfully finished the trajectory!')
				return True
			else:
				rospy.logerr('Motion controller failed to complete the trajectory with error %s',result.errorId)
				rospy.loginfo('Could not complete trajectory due to collision')
				if fail_plan is None or fail_plan == '':
					pass
				elif fail_plan == 'audio':
					pygame.mixer.init()
					pygame.mixer.music.load("collision.mp3")
					pygame.mixer.music.play()
					while pygame.mixer.music.get_busy(): 
						pygame.time.Clock().tick(10)
				elif fail_plan == 'user assist':
					pygame.mixer.init()
					pygame.mixer.music.load('safety3.mp3')
					pygame.mixer.music.play()
					while pygame.mixer.music.get_busy(): 
						pygame.time.Clock().tick(10)

					self.interaction_control(
						orientation_x = True,
						orientation_y = True,
						orientation_z = True,
						position_x = True,
						position_y = True,
						position_z = True,
					)
					while self.endpoint_pose()['position'].x<0.9:
						pass
					self.go_to_joint_angles(self.joint_angles())

					pygame.mixer.init()
					pygame.mixer.music.load('safety4.mp3')
					pygame.mixer.music.play()
					while pygame.mixer.music.get_busy(): 
						pygame.time.Clock().tick(10)
					self.go_to_joint_angles()
				else:
					rospy.logerr('Did not recognize fail plan ' + fail_plan)
				return False
		except rospy.ROSInterruptException:
			rospy.logerr('Keyboard interrupt detected from the user. Exiting before trajectory completion.')

	def go_to_cartesian_pose(self, position = None, orientation = None, relative_pose = None,
		in_tip_frame = False, joint_angles = None, tip_name = 'right_hand',
		linear_speed = 0.6, linear_accel = 0.6, rotational_speed = 1.57,
		rotational_accel = 1.57, timeout = None, endpoint_pose = None):
		
		traj_options = TrajectoryOptions()
		traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
		traj = MotionTrajectory(trajectory_options = traj_options, limb = self)

		wpt_opts = MotionWaypointOptions(max_linear_speed=linear_speed,
										 max_linear_accel=linear_accel,
										 max_rotational_speed=rotational_speed,
										 max_rotational_accel=rotational_accel,
										 max_joint_speed_ratio=1.0)
		waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = self)

		joint_names = self.joint_names()

		if joint_angles and len(joint_angles) != len(joint_names):
			rospy.logerr('len(joint_angles) does not match len(joint_names!)')
			return None

		if (position is None and orientation is None and relative_pose is None and endpoint_pose is None):
			if joint_angles:
				# does Forward Kinematics
				waypoint.set_joint_angles(joint_angles, tip_name, joint_names)
			else:
				rospy.loginfo("No Cartesian pose or joint angles given. Using default")
				waypoint.set_joint_angles(joint_angles=None, active_endpoint=args.tip_name)
		else:
			endpoint_state = self.tip_state(tip_name)
			if endpoint_state is None:
				rospy.logerr('Endpoint state not found with tip name %s', tip_name)
				return None
			pose = endpoint_state.pose

			if relative_pose is not None:
				if len(relative_pose) != 6:
					rospy.logerr('Relative pose needs to have 6 elements (x,y,z,roll,pitch,yaw)')
					return None
				# create kdl frame from relative pose
				rot = PyKDL.Rotation.RPY(relative_pose[3],
										 relative_pose[4],
										 relative_pose[5])
				trans = PyKDL.Vector(relative_pose[0],
									 relative_pose[1],
									 relative_pose[2])
				f2 = PyKDL.Frame(rot, trans)
				# and convert the result back to a pose message
				if in_tip_frame:
				  # end effector frame
				  pose = posemath.toMsg(posemath.fromMsg(pose) * f2)
				else:
				  # base frame
				  pose = posemath.toMsg(f2 * posemath.fromMsg(pose))
			else:
				if endpoint_pose is None:
					if position is not None and len(position) == 3:
						pose.position.x = position[0]
						pose.position.y = position[1]
						pose.position.z = position[2]
					if orientation is not None and len(orientation) == 4:
						pose.orientation.x = orientation[0]
						pose.orientation.y = orientation[1]
						pose.orientation.z = orientation[2]
						pose.orientation.w = orientation[3]
				else:
					pose.position.x = endpoint_pose['position'].x
					pose.position.y = endpoint_pose['position'].y
					pose.position.z = endpoint_pose['position'].z
					pose.orientation.x = endpoint_pose['orientation'].x
					pose.orientation.y = endpoint_pose['orientation'].y
					pose.orientation.z = endpoint_pose['orientation'].z
					pose.orientation.w = endpoint_pose['orientation'].w
			poseStamped = PoseStamped()
			poseStamped.pose = pose

			if not joint_angles:
				# using current joint angles for nullspace bais if not provided
				joint_angles = self.joint_ordered_angles()
				if self.VERBOSE: print(poseStamped.pose)
				waypoint.set_cartesian_pose(poseStamped, tip_name, joint_angles)
			else:
				waypoint.set_cartesian_pose(poseStamped, tip_name, joint_angles)

		#rospy.loginfo('Sending waypoint: \n%s', waypoint.to_string())

		traj.append_waypoint(waypoint.to_msg())

		result = traj.send_trajectory(timeout=timeout)
		if result is None:
			rospy.logerr('Trajectory FAILED to send')
			return

		if result.result:
			if self.VERBOSE: rospy.loginfo('Motion controller successfully finished the trajectory!')
		else:
			rospy.logerr('Motion controller failed to complete the trajectory with error %s',
						 result.errorId)

	def interaction_control(self, position_only = False, orientation_only = False, plane_horizontal = False,
		plane_vertical_xz = False, plane_vertical_yz = False, nullspace_only = False,
		position_x = False, position_y = False, position_z = False,
		orientation_x = False, orientation_y = False, orientation_z = False,
		constrained_axes = [1, 1, 1, 1, 1, 1], in_endpoint_frame = False, interaction_frame = [0, 0, 0, 1, 0, 0, 0], 
		K_nullspace = [10.0, 10.0, 7.0, 0.0, 0.0, 0.0, 0.0], rate = 10):

		try:
			rospy.Subscriber('zeroG_topic', String, self.zeroG_callback)
			self.zeroG = True
			rospy.sleep(0.5)

			if rate > 0:
				rate = rospy.Rate(rate)
			elif rate == 0:
				rospy.logwarn('Interaction control options will be set only once!')
			elif rate < 0:
				rospy.logerr('Invalid publish rate!')

			# set the interaction control options in the current configuration
			interaction_options = InteractionOptions()

			# if one of the options is set
			unconstrained_axes_default = [0, 0, 0, 0, 0, 0]
			unconstrained_axes = unconstrained_axes_default

			# create a list of the constrained zero G modes
			sum_mode_list = sum(bool2int([position_only, orientation_only, plane_horizontal, plane_vertical_xz, plane_vertical_yz, nullspace_only]))

			# create a list of the free axis options
			free_axis_list = bool2int([position_x, position_y, position_z, orientation_x, orientation_y, orientation_z])
			sum_free_axis_list = sum(free_axis_list)

			zero_stiffness_axes = boolToggle(constrained_axes)
			sum_zero_stiffness_axes = sum(zero_stiffness_axes)

			if (sum_mode_list==0 and sum_zero_stiffness_axes==0 and sum_free_axis_list==0):
				rospy.loginfo('Constrained in all directions')

			if (sum_mode_list>1 and sum_zero_stiffness_axes==0 and sum_free_axis_list==0):
				rospy.logerr('You can set only one of the options among "pos_only", "ori_only", "plane_hor", and "plane_ver"! The movement of endpoint will be constrained in all directions!')

			# give an error if the mode options as well as the individual axis options are set
			if (sum_mode_list==1 and sum_zero_stiffness_axes==0 and sum_free_axis_list>0):
				rospy.logerr('The individual axis options cannot be used together with the mode options! The movement of endpoint will be constrained in all directions!')

			# give an error when the axes are specified by more than one method
			if (sum_mode_list==0 and sum_zero_stiffness_axes>0 and sum_free_axis_list>0):
				rospy.logerr('You can only set the axes either by an array or individual options! The movement of endpoint will be constrained in all directions!')

			if (sum_mode_list==1 and sum_zero_stiffness_axes==0 and sum_free_axis_list>=0):
				if position_only:
					unconstrained_axes = [1, 1, 1, 0, 0, 0]
				if orientation_only:
					unconstrained_axes = [0, 0, 0, 1, 1, 1]
				if plane_horizontal:
					unconstrained_axes = [1, 1, 0, 0, 0, 0]
				if plane_vertical_xz:
					unconstrained_axes = [1, 0, 1, 0, 0, 0]
				if plane_vertical_yz:
					unconstrained_axes = [0, 1, 1, 0, 0, 0]
				if nullspace_only:
					unconstrained_axes = [0, 0, 0, 0, 0, 0]

			# if the axes are specified by an array
			if (sum_mode_list==0 and sum_zero_stiffness_axes>0 and sum_free_axis_list==0):
				unconstrained_axes = zero_stiffness_axes

			# if the axes are specified by individual options
			if (sum_mode_list==0 and sum_zero_stiffness_axes==0 and sum_free_axis_list>0):
				unconstrained_axes = free_axis_list

			# set the stiffness to zero by default
			interaction_options.set_K_impedance([0, 0, 0, 0, 0, 0])

			# set the axes with maximum stiffness
			interaction_options.set_max_impedance(boolToggle(int2bool(unconstrained_axes)))

			interaction_options.set_in_endpoint_frame(in_endpoint_frame)

			# set nullspace stiffness to zero if nullspace_only option is provided
			if nullspace_only:
				K_nullspace = [0, 0, 0, 0, 0, 0, 0]

			interaction_options.set_K_nullspace(K_nullspace)

			if len(interaction_frame) < 7:
				rospy.logerr('The number of elements must be 7!')
			elif len(interaction_frame) == 7:
				quat_sum_square = interaction_frame[3]*interaction_frame[3] + interaction_frame[4]*interaction_frame[4]
				+ interaction_frame[5]*interaction_frame[5] + interaction_frame[6]*interaction_frame[6]
				if quat_sum_square  < 1.0 + 1e-7 and quat_sum_square > 1.0 - 1e-7:
					interaction_frame_pose = Pose()
					interaction_frame_pose.position.x = interaction_frame[0]
					interaction_frame_pose.position.y = interaction_frame[1]
					interaction_frame_pose.position.z = interaction_frame[2]
					interaction_frame_pose.orientation.w = interaction_frame[3]
					interaction_frame_pose.orientation.x = interaction_frame[4]
					interaction_frame_pose.orientation.y = interaction_frame[5]
					interaction_frame_pose.orientation.z = interaction_frame[6]
					interaction_options.set_interaction_frame(interaction_frame_pose)
				else:
					rospy.logerr('Invalid input to quaternion! The quaternion must be a unit quaternion!')
			else:
				rospy.logerr('Invalid input to interaction_frame!')

			# always enable the rotations for constrained zero-G
			interaction_options.set_rotations_for_constrained_zeroG(True)

			msg = interaction_options.to_msg()

			# print the resultant interaction options once
			#rospy.loginfo(msg)
			self.icc_pub.publish(msg)

			rs = intera_interface.RobotEnable(CHECK_VERSION)

		except rospy.ROSInterruptException:
			rospy.logerr('Keyboard interrupt detected from the user. %s',
						 'Exiting the node...')

		if not rs.state().enabled:
			self.position_mode()

	# This does not yet work
	def jointAngles2Pose(self, joint_angles):
		ns = "ExternalTools/right/PositionKinematicsNode/FKService"
		fksvc = rospy.ServiceProxy(ns, SolvePositionFK)
		fkreq = SolvePositionFKRequest()
		joints = JointState()
		if self.VERBOSE: print(self.joint_names())
		for key in self.joint_names():
			joints.name.append(key)
		joints.position = joint_angles
		rospy.wait_for_service(ns, 5.0)
		resp = fksvc(fkreq)
		if self.VERBOSE: rospy.loginfo(resp)

	def adjustPoseTo(self, geometry, axis, amount):
		pose = self.endpoint_pose()
		kwargs = {axis: amount}
		pose[geometry] = pose[geometry]._replace(**kwargs)
		self.go_to_cartesian_pose(endpoint_pose = pose)

	def adjustPoseBy(self, geometry, axis, amount):
		pose = self.endpoint_pose()
		self.adjustPoseTo(geometry, axis, getattr(pose[geometry],axis)+amount)

	def adjustOrientationTo(self, **kwargs):
		pose = self.endpoint_pose()
		pose['orientation'] = pose['orientation']._replace(**kwargs)
		self.go_to_cartesian_pose(endpoint_pose = pose)

	def zeroG_callback(self,msg):
		self.zeroG = False
		if self.VERBOSE: rospy.loginfo('ZeroG Callback')

	def position_mode(self):
		# send a message to put the robot back into position mode
		if self.VERBOSE: rospy.loginfo('Entering position mode')
		position_mode = InteractionOptions()
		position_mode.set_interaction_control_active(False)
		self.icc_pub.publish(position_mode.to_msg())
		rospy.sleep(0.5)