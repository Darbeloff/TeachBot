#!/usr/bin/env python
## IMPORTS ##
# Basic
import rospy, math, actionlib
import numpy as np
import sys
import roslib

from std_msgs.msg import Bool, String, Int32, Float64, Float64MultiArray, UInt16, Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from controller_manager_msgs.srv import SwitchController
import sensor_msgs
import threading

from ur.msg import *
from ur.srv import *

from geometry_msgs.msg import WrenchStamped

# joint names defined by UR in the correct order 
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

# joint names using 'right_j0' and 'right_j1' for consistency across packages and easy string construction in functions 
JOINT_CONTROL_NAMES = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5']

class Module():
    JOINTS = 6
    FORCE2VELOCITY = {'right_j0': 0.06, 'right_j1': 0.06, 'right_j2': 0.4, 'right_j3': 0.2, 'right_j4': 1, 'right_j5': 0.9}
    def __init__(self):
        #Initialize node
        rospy.init_node('ur_comm_node', anonymous=True)
        self.VERBOSE = True

        # Global Vars
        self.audio_duration = 0
        self.finished = False
        self.devMode = False
        self.allowCuffInteraction = False
        self.modeTimer = None

        # Custom Control Variables --> used in Admittance Control
        self.control = {
            'i': 0, # index we are on out of the 15 values (0-14)
            'order': 15, # how many we are keeping for filtering
            'effort': [],			# Structured self.control['effort'][tap #, e.g. i][joint, e.g. 'right_j0']
            'position': [],
            'velocity': []
        }
        
        # Variables for forces-x --> used in Admittance control, UR specific
        # Maybe later could include full wrench values?
        self.forces = {
            'i': 0, # index we are on out of the 15 values (0-14)
            'order': 15, # how many we are keeping for filtering
            'fx': [] # different from Sawyer, this gathers the wrench values, forces in x-direction
        }
        self.forces['fx'] = [0]*self.forces['order']
        self.FORCE_STANDARDIZATION_CONSTANT = .2

        # Create empty structures in self.control to match with above comments 
        zeroVec = dict()
        for joint in JOINT_CONTROL_NAMES:
            zeroVec[joint] = 0.0
        for i in range(self.control['order']):
            self.control['effort'].append(zeroVec.copy())
            self.control['position'].append(zeroVec.copy())
            self.control['velocity'].append(zeroVec.copy())

        # Publish topics to Browser
        self.command_complete_topic = rospy.Publisher('/command_complete', Empty, queue_size=1) #this is for the module/browser
        self.position_topic = rospy.Publisher('/teachbot/position', JointInfo, queue_size=1)
        self.velocity_topic = rospy.Publisher('/teachbot/velocity', JointInfo, queue_size=1)
        self.effort_topic = rospy.Publisher('/teachbot/effort', JointInfo, queue_size=1)

        # Publish topics to UR
        self.publish_velocity_to_robot = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=1)
        
        # Action Servers
        self.GoToJointAnglesAct = actionlib.SimpleActionServer('/teachbot/GoToJointAngles', GoToJointAnglesAction, execute_cb=self.cb_GoToJointAngles, auto_start=True)
        
        # Service Servers
        rospy.Service('/teachbot/audio_duration', AudioDuration, self.rx_audio_duration)
        rospy.Service('/teachbot/set_robot_mode', SetRobotMode, self.cb_SetRobotMode)

        # Action Clients - Publish to robot
        self.joint_traj_client = actionlib.SimpleActionClient('/scaled_pos_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.velocity_traj_client = actionlib.SimpleActionClient('/scaled_vel_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        # Subscribed Topics
        rospy.Subscriber('/joint_states', sensor_msgs.msg.JointState, self.forwardJointState)
        rospy.Subscriber('/wrench', WrenchStamped, self.cb_filter_forces)


        rospy.loginfo('TeachBot is initialized and ready to go.')


    def rx_audio_duration(self,data):
        self.audio_duration = data.audio_duration
        return True

    '''
    Read information from joint states, parse into position, velocity, and effort, then 
    forward the information by publishing it.
    '''
    def forwardJointState(self, data):
        position = JointInfo()
        velocity = JointInfo()
        effort = JointInfo()
        for j in range(Module.JOINTS):
            setattr(position, 'j'+str(j), data.position[j])
            setattr(velocity, 'j'+str(j), data.velocity[j])
            setattr(effort, 'j'+str(j), data.effort[j])

            # Update self.control which stores joint states to be used by admittance control + other functions
            # position, velocity, and effort keep 15 values (for filtering later), so set the self.control['i'] index's j'th point to this value
            self.control['position'][self.control['i']]['right_j'+str(j)] = data.position[j] 
            self.control['velocity'][self.control['i']]['right_j'+str(j)] = data.velocity[j]
            self.control['effort'][self.control['i']]['right_j'+str(j)] = data.effort[j]

        # since we are trying to keep only 15 values for filtering, +1 if i is less than 15 and reset to 0 otherwise 
        self.control['i'] = self.control['i']+1 if self.control['i']+1<self.control['order'] else 0

        # Publish joint state information to the browser 
        self.position_topic.publish(position)
        self.velocity_topic.publish(velocity)
        self.effort_topic.publish(effort)
    
    '''
    Used to help the forces acting on wrist_3_joint (the endpoint)
    Stores data on the x-direction only as of right now, since that is what will be used for admittance ctrl (optimized to SCARA)
    '''
    def cb_filter_forces(self, data):
        for j in range(self.forces['order']):
            self.forces['fx'][self.forces['i']] = data.wrench.force.x
        
        self.forces['i'] = self.forces['i']+1 if self.forces['i']+1<self.forces['order'] else 0
        
    '''
    When the subscriber to "GoToJointAngles" receives the name of a constant from the browser,
    it is evaluated in this function and parsed into a "FollowTrajectoryGoal" object and sent to the
    action client provided by Universal Robots driver. Upon completion, it publishes to "/command-complete"
    to let the browser know that it has finished its task.
    '''
    def cb_GoToJointAngles(self, goal):
        self.joint_traj_client.wait_for_server()

        followJoint_msg = FollowJointTrajectoryGoal()

        if goal.name != '':
            print goal.name
            followJoint_msg.trajectory = self.create_traj_goal(eval(goal.name))
        else:
            followJoint_msg.trajectory = self.create_traj_goal([goal.j0pos, goal.j1pos, goal.j2pos, goal.j3pos, goal.j4pos, goal.j5pos])
        
        # Send goal to action client and wait for completion of task
        self.joint_traj_client.send_goal(followJoint_msg)
        self.joint_traj_client.wait_for_result()

        # Set success and return to browser module
        result = ur.msg.GoToJointAnglesResult()
        result.success = True
        self.GoToJointAnglesAct.set_succeeded(result)

    '''
    Helper function to create a JointTrajectory message, which is used to interact with the ROS driver and give the arm commands
    on how/where to move. 
    '''
    def create_traj_goal(self, array):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = JOINT_NAMES

        jointPositions_msg = JointTrajectoryPoint()
        jointPositions_msg.positions = array
        jointPositions_msg.time_from_start = rospy.Duration(3)

        traj_msg.points = [jointPositions_msg,]

        return traj_msg
    
    '''
    Set the robot's mode of movement
    '''
    def cb_SetRobotMode(self, req):
        if self.VERBOSE: rospy.loginfo('Entering ' + req.mode + ' mode.')

        if not (self.modeTimer is None):
            self.modeTimer.shutdown()

        if req.mode == 'position':          
            rospy.wait_for_service('/controller_manager/switch_controller')
            try:
                switch_controller = rospy.ServiceProxy(
                                    'controller_manager/switch_controller', SwitchController)
                ret = switch_controller(['scaled_pos_traj_controller'], ['joint_group_vel_controller'], 2)
                if ret.ok:
                    rospy.loginfo("Switched to scaled pos traj controller")
            except rospy.ServiceException, e:
                print "Service call failed to switch to scaled_pos_traj_controller"
                rospy.logerr("Service call failed to switch to scaled_pos_traj_controller")

                       
        elif req.mode == 'admittance ctrl':
            # Initialize Joints Dict
            joints = {}
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
                    joints[j]['bias'] = 0

            # Set F2V specs
            if len(req.F2V)!=0:
                for i,j in enumerate(req.joints):
                    joints['right_j'+str(j)]['F2V'] = req.F2V[i]
            else:
                for j in joints.keys():
                    joints[j]['F2V'] = self.FORCE2VELOCITY[j]

            # Switch to the joint group velocity controller 
            rospy.wait_for_service('/controller_manager/switch_controller')
            try:
                switch_controller = rospy.ServiceProxy(
                                    'controller_manager/switch_controller', SwitchController)
                ret = switch_controller(['joint_group_vel_controller'], ['scaled_pos_traj_controller'], 2)
                if ret.ok:
                    rospy.loginfo("Switched to joint_group_vel_controller")
            except rospy.ServiceException, e:
                print "Service call failed to switch to scaled_pos_traj_controller"
                rospy.logerr("Service call failed to switch to joint_group_vel_controller")

            
            self.modeTimer = rospy.Timer(rospy.Duration(0.1), lambda event=None : self.cb_AdmittanceCtrl(joints, eval(req.resetPos)))
        
        else:
            rospy.logerr('Robot mode ' + req.mode + ' is not a supported mode.')

        return True

    def cb_AdmittanceCtrl(self, joints, resetPos, rateNom=10, tics=15):
        # new velocities to send to robot
        velocities = {}
        for j in range(len(joints.keys())):
            velocities['right_j'+str(j)] = 0
        
        for joint in JOINT_CONTROL_NAMES:
            if joint in joints.keys(): # this is inside joints which is passed in through action server, the dict only gives joints to control, others stay immobile
                allForces = [self.forces['fx'][i] for i in range(self.forces['order'])] # gather the effort that was stored
                
                filteredForce = sum(allForces)/self.forces['order'] # filter the effort
                
                filteredForce = filteredForce + joints[joint]['bias'] # this is from service
                # the forces could be -27 to 27 so need to standardize it to be under min_thresh like Sawyer if too weak
                filteredForce = filteredForce * self.FORCE_STANDARDIZATION_CONSTANT 

                if abs(filteredForce) < joints[joint]['min_thresh']: # this is from service 
                    velocities[joint] = 0
                else:
                    velocities[joint] = joints[joint]['F2V']*filteredForce # this is from service
                    rospy.loginfo(str(velocities[joint]) + ' calculation')
            else:
                velocities[joint] = 0
        
        # Publish velocities to UR
        velocity_msg = Float64MultiArray()

        velocity_data = []
        for j in range(len(velocities.keys())):
            velocity_data.append(velocities['right_j'+str(j)])

        velocity_msg.data = velocity_data
        rospy.loginfo(velocity_data)
        self.publish_velocity_to_robot.publish(velocity_msg)



if __name__ == '__main__':
    ## DEFINE IMPORTANT CONSTANTS --- MAKE SURE THEY MATCH WITH MODULE 1 OR 2 CONSTANTS ##

    # POSITION CONSTANTS - ARRAYS THAT MATCH JOINT_NAMES
    ZERO = [0, -1.57, 0, -1.57, 0, 0]
    SCARA = [0, -3.14, 0, -3.14, -1.57, 0]
    WRIST_3_FWD = [0, -3.14, 0, -3.14, -1.57, -.5]
    WRIST_2_FWD = [0, -3.14, 0, -3.14, -1.0, 0]
    WRIST_1_FWD = [0, -3.14, 0, -2.54, -1.57, 0]
    ELBOW_FWD = [0, -3.14, 0.5, -3.14, -1.57, 0]
    SHOULDER_FWD = [0, -2.80, 0, -3.14, -1.57, 0]
    BASE_FWD = [0.50, -3.14, 0, -3.14, -1.57, 0]
    
    default = ZERO

    # TODO: Figure out what no_hit is and what j4 max is    
    # TODO: Figure out DPS becaues the arm needs to be over the table for this part
    DSP = SCARA[1]
    j2scara = SCARA[2]
    over_table = -3.5
    j4max = SCARA[4]

    # 1
    joint_motor_animation_0 = SCARA
    joint_motor_animation_1 = [0, -3.14, -0.25, -3.14, -1.25, 0]
    # 4
    joint_test = [0]*Module.JOINTS
    joint_test = [WRIST_3_FWD, WRIST_2_FWD, WRIST_1_FWD, ELBOW_FWD, SHOULDER_FWD, BASE_FWD]
    
    # 6 - 15
    # TODO Find the hard-coded values that work for UR
    # joint_dof_start = [-1.57, DSP,j2scara,0,-j4max,0]
    # joint_dof_shoulder = [-1.57, -2.80 ,j2scara,0,-j4max,0]
    # joint_dof_elbow = [-1.57, DSP, .30, 0,-j4max,0]
    # joint_dof_wrist = [-1.57, DSP,j2scara,0, 0.45,0]
    # joint_dof_up = [-1.57, DSP, j2scara,0,-j4max,0]
    # for now use the following so it goes through this section of the module but doesn't swing around like crazy
    joint_dof_start = SCARA
    joint_dof_shoulder = SCARA
    joint_dof_elbow = SCARA
    joint_dof_wrist = SCARA
    joint_dof_up = SCARA

    m = Module()

    try:
        rospy.spin()

    except rospy.ROSInterruptException:
        pass


