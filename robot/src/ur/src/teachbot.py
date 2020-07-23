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
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg

from ur_dashboard_msgs.msg import RobotMode

from ur.msg import *
from ur.srv import *

from geometry_msgs.msg import WrenchStamped

# joint names defined by UR in the correct order 
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

# joint names using 'right_j0' and 'right_j1' for consistency across packages and easy string construction in functions 
JOINT_CONTROL_NAMES = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5']

# Controller names
CONTROLLERS = {
    "position": "scaled_pos_traj_controller",
    "velocity": "joint_group_vel_controller"
}


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
        self.robot_state = 0
        self.current_controller = "scaled_pos_traj_controller"

        # Custom Control Variables --> used in Admittance Control
        self.control = {
            'i': 0, # index we are on out of the 15 values (0-14)
            'order': 15, # how many we are keeping for filtering
            # Structured self.control['effort'][tap #, e.g. i][joint, e.g. 'right_j0']
            'effort': [], # effort is read-only. UR does not support direct effort/torque control for safety reasons.
            'position': [],
            'velocity': []
        }
        
        # Variables for forces-x --> used in Admittance control, UR specific
        self.forces = {
            'i': 0, # index we are on out of the 60 values (0-59)
            'order': 60, # how many we are keeping for filtering
            'fx': [], # different from Sawyer, this gathers the wrench values, forces in x-direction
            'fy': [],
            'fz': []
        }
        self.forces['fx'] = [0]*self.forces['order']
        self.forces['fy'] = [0]*self.forces['order']
        self.forces['fz'] = [0]*self.forces['order']
        self.FORCE_STANDARDIZATION_CONSTANT = .2

        # Variables used in filtering velocity output in impedance control
        self.vel_filter = {
            'i': 0,
            'order': 7,
            'velocity': {}
        }
        for name in JOINT_CONTROL_NAMES:
            self.vel_filter['velocity'][name] = [0.0] * self.vel_filter['order']

        # Create empty structures in self.control to match with above comments 
        zeroVec = dict()
        for joint in JOINT_CONTROL_NAMES:
            zeroVec[joint] = 0.0
        for i in range(self.control['order']):
            self.control['effort'].append(zeroVec.copy())
            self.control['position'].append(zeroVec.copy())
            self.control['velocity'].append(zeroVec.copy())

        # Variables to store the status of gripper
        self.gripper_stats = {
            'gACT': 0,
            'gGTO': 0,
            'gSTA': 0,
            'gOBJ': 0,
            'gFLT': 0,
            'gPR': 0,
            'gPO': 0,
            'gCU': 0
        }

        # Publish topics to Browser
        self.command_complete_topic = rospy.Publisher('/command_complete', Empty, queue_size=1) #this is for the module/browser
        self.position_topic = rospy.Publisher('/teachbot/position', JointInfo, queue_size=1)
        self.velocity_topic = rospy.Publisher('/teachbot/velocity', JointInfo, queue_size=1)
        self.gripper_command_pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=10)

        # Publish topics to UR
        self.publish_velocity_to_robot = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=1)
        
        rospy.Subscriber('/comtest', Empty, self.cb_run_some_tests)

        # Action Servers
        self.GoToJointAnglesAct = actionlib.SimpleActionServer('/teachbot/GoToJointAngles', GoToJointAnglesAction, execute_cb=self.cb_GoToJointAngles, auto_start=True)
        
        # Service Servers
        rospy.Service('/teachbot/audio_duration', AudioDuration, self.rx_audio_duration)
        rospy.Service('/teachbot/set_robot_mode', SetRobotMode, self.cb_SetRobotMode)

        # Service Clients
        self.switch_controller = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)

        # Action Clients - Publish to robot
        self.joint_traj_client = actionlib.SimpleActionClient('/scaled_pos_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.velocity_traj_client = actionlib.SimpleActionClient('/scaled_vel_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        # Subscribed Topics
        rospy.Subscriber('/joint_states', sensor_msgs.msg.JointState, self.forwardJointState)
        rospy.Subscriber('/wrench', WrenchStamped, self.cb_filter_forces)
        rospy.Subscriber('/ur_hardware_interface/robot_mode', RobotMode, self.cb_ReadRobotState)
        rospy.Subscriber("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input, self.printGripperStatus)

        res = self.switch_controller([],['scaled_pos_traj_controller','joint_group_vel_controller'], 1, True, 10.0)
        res = self.switch_controller(['scaled_pos_traj_controller'],[], 1, True, 10.0)

        self.initialize_gripper()
        rospy.loginfo('TeachBot is initialized and ready to go.')
        
    '''
    Temporary callback for testing some functionalities.
    '''
    def cb_run_some_tests(self, data):

        self.switchController('velocity')

    '''
    Initialize the gripper. This is necessary to use it, or it won't respond.
    '''
    def initialize_gripper(self):
        command = outputMsg.Robotiq2FGripper_robot_output();
        command.rACT = 0
        self.gripper_command_pub.publish(command)
        rospy.sleep(1)
        command.rACT = 1
        command.rGTO = 1
        command.rSP  = 255 # 0-255, changes the finger's moving speed.
        command.rFR  = 10 # 0-255, changes the gripping force.
        self.gripper_command_pub.publish(command)

    '''
    
    '''
    def execute_gripper_command(self, todo):
        command = outputMsg.Robotiq2FGripper_robot_output();
        if todo == 'open':
            command.rPR = 0
        elif todo == 'close':
            command.rPR = 230
        else:
            rospy.logwarn("Command "+str(todo)+" not supported! go to teachbot.py to add gripper functionalities.")
            pass

    def printGripperStatus(self, msg):
        for key in self.gripper_stats.keys():
            self.gripper_stats[key] = eval('msg.'+key)
        print(self.gripper_stats)
    '''
    Read current robot's mode, which is pre-defined in ur_robot_driver. This callback is only to check if the
    robot is powered on/off, booting, or ready.

    It has nothing to do with SetRobotMode.
    '''
    def cb_ReadRobotState(self, data):
        self.robot_state = data.mode
        if(data.mode==7):
            rospy.loginfo('Robot powered on and brake released, ready to accept commands.')


    def rx_audio_duration(self,data):
        self.audio_duration = data.audio_duration
        return True

    '''
    Switch between position and velocity controllers.
    More drivers are supported but need to know how to use roscontrol.
    '''
    def switchController(self, controller):
        if controller.lower() not in CONTROLLERS.keys():
            rospy.logerr("Invalid controller to switch!")
        else:
            rospy.wait_for_service('/controller_manager/switch_controller')
            switch_controller_srv = rospy.ServiceProxy(
                                        'controller_manager/switch_controller', SwitchController)
            try:
                # unload all controllers and then load the desired controller.
                res = switch_controller_srv([],[val for val in CONTROLLERS.values()], 1, True, 10.0)
                ret = switch_controller_srv([CONTROLLERS[controller.lower()]], [], 1, True, 10.0)
                if ret.ok:
                    rospy.loginfo("Switched to {} controller".format(CONTROLLERS[controller.lower()]))
            except rospy.ServiceException, e:
                rospy.logerr("Service call failed to switch to {}".format(CONTROLLERS[controller.lower()]))

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
            # position, velocity keep 15 values (for filtering later), so set the self.control['i'] index's j'th point to this value
            self.control['position'][self.control['i']]['right_j'+str(j)] = data.position[j] 
            self.control['velocity'][self.control['i']]['right_j'+str(j)] = data.velocity[j]

        # Robot orders joint names alphabetically, so elbow comes first and shoulder_pan comes third.
        # Code below swaps those two to match name to value.
        temp_p = self.control['position'][self.control['i']]['right_j0']
        self.control['position'][self.control['i']]['right_j0'] = self.control['position'][self.control['i']]['right_j2']
        self.control['position'][self.control['i']]['right_j2'] = temp_p
        temp_v = self.control['velocity'][self.control['i']]['right_j0']
        self.control['velocity'][self.control['i']]['right_j0'] = self.control['velocity'][self.control['i']]['right_j2']
        self.control['velocity'][self.control['i']]['right_j2'] = temp_v
        
        # since we are trying to keep only 'order' values for filtering, +1 if i is less than 'order' and reset to 0 otherwise 
        self.control['i'] = self.control['i']+1 if self.control['i']+1<self.control['order'] else 0

        # Publish joint state information to the browser 
        self.position_topic.publish(position)
        self.velocity_topic.publish(velocity)

    
    '''
    Used to help the forces acting on wrist_3_joint (the endpoint)
    Stores data on the x-direction only as of right now, since that is what will be used for admittance ctrl (optimized to SCARA)
    '''
    def cb_filter_forces(self, data):
        for j in range(self.forces['order']):
            self.forces['fx'][self.forces['i']] = data.wrench.force.x
            self.forces['fy'][self.forces['i']] = data.wrench.force.x
            self.forces['fz'][self.forces['i']] = data.wrench.force.x
        
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
            self.switchController('position')

        elif req.mode == 'admittance_ctrl':
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
            self.switchController('velocity')
            
            self.modeTimer = rospy.Timer(rospy.Duration(0.1), lambda event=None : self.cb_AdmittanceCtrl(joints, eval(req.resetPos)))
        
        # '''
        # Currently, impedance control is set to work only with one joint, specifically
        # the elbow joint at one specified posture. This is due to force input in cartesian
        # coordinate and movement output in joint coordinate. A more complete impedance
        # control with full-arm scale is possible with more complex calculations but for
        # current usage of impedance control, only one joint is sufficient.

        # The format below just follows the format from Saywer's version of teachbot.py and
        # is not intended to use directly as is for multiple joint control.
        # '''
        elif req.mode == 'impedance ctrl':
            # Initialize Joints Dict
            joints = {}
            for j in req.joints:
                joints['right_j'+str(j)] = {}

            # Set V2F and X2F specs (currently not supported, do not uncomment)
            # for i,j in enumerate(req.joints):
            #     joints['right_j'+str(j)]['V2F'] = req.V2F[i]
            #     joints['right_j'+str(j)]['X2F'] = req.X2F[i]

            # Set position and velocity reference points
            x_ref = self.control['position'][self.control['i']]
            print(x_ref)
            for j in req.joints:
                joints['right_j'+str(j)]['x_ref'] = x_ref['right_j'+str(j)]
                joints['right_j'+str(j)]['v_ref'] = 0

            # Switch to the joint group velocity controller
            self.switchController('velocity')

            self.modeTimer = rospy.Timer(rospy.Duration(0.004), lambda event=None : self.cb_ImpedanceCtrl(joints, eval(req.resetPos)))

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


    '''
    Callback function for impedance contro.

    Note that this only applies to elbow joint at posture IMPE_INIT.
    '''
    def cb_ImpedanceCtrl(self, joints, resetPos, rateNom=10, tics=15):
        # Define some constants used in this callback func.
        F_threshold = 5
        F_cutoff = 2 * F_threshold
        f_max_offset = 8.7
        Kpf = 45
        Kfv = -0.059

        velocities = {}
        for name in JOINT_CONTROL_NAMES:
            velocities[name] = 0

        for joint in joints.keys():
            delta_pos = self.control['position'][self.control['i']][joint] - joints[joint]['x_ref']
            bias_z = -math.sin(delta_pos) * f_max_offset
            maf_fz = sum(self.forces['fz']) / self.forces['order']
            force_input = maf_fz - bias_z
            if (abs(force_input) < F_threshold/2):
                force_input = 0

            P2F = Kpf * delta_pos
            force_input += P2F

            if abs(force_input) < F_cutoff:
                F2V = Kfv / (2 * F_cutoff) * force_input * abs(force_input)
            else:
                if (force_input < 0):
                    F2V = Kfv * (force_input + F_threshold)
                else:
                    F2V = Kfv * (force_input - F_threshold)

            self.vel_filter['velocity'][joint][self.vel_filter['i']] = F2V
            velocities[joint] = sum(self.vel_filter['velocity'][joint]) / self.vel_filter['order']

        self.vel_filter['i'] = self.vel_filter['i']+1 if self.vel_filter['i']+1<self.vel_filter['order'] else 0

        velocities_data = []
        for i in range(len(JOINT_CONTROL_NAMES)):
            velocities_data.append(velocities['right_j'+str(i)])

        velocity_msg = Float64MultiArray()
        velocity_msg.data = velocities_data
        print(velocities_data)
        # self.publish_velocity_to_robot.publish(velocity_msg)


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
    IMPE_INIT = [1.57, -3.14, 1.57, -1.57, 1.57, 0]
    
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


