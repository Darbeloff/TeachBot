#!/usr/bin/env python
## IMPORTS ##
# Basic
import rospy, math, actionlib, threading, sys, time
from math import radians, degrees
import numpy as np
import roslib
# The following two packages are not well documented. Ideally they are used for gravity compensation.
import PyKDL as kdl
import kdl_parser_local as kdl_parser
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

import sensor_msgs
from tf.msg import tfMessage
from std_msgs.msg import Bool, String, Int32, Float64, Float64MultiArray, UInt16, Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from controller_manager_msgs.srv import SwitchController
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg
from trac_ik_python.trac_ik import IK

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
        self.endpoint_prev = [0.0] * 7

        # Custom Control Variables --> used in Admittance Control
        self.control = {
            'i': 0, # index we are on out of the order values (0 to order-1)
            'order': 60, # how many we are keeping for filtering
            # Structured self.control['effort'][tap #, e.g. i][joint, e.g. 'right_j0']
            'effort': [], # effort is read-only. UR does not support direct effort/torque control for safety reasons.
            'position': [],
            'velocity': []
        }

        # Create empty structures in self.control to match with above comments 
        zeroVec = dict()
        for joint in JOINT_CONTROL_NAMES:
            zeroVec[joint] = 0.0
        for i in range(self.control['order']):
            self.control['effort'].append(zeroVec.copy())
            self.control['position'].append(zeroVec.copy())
            self.control['velocity'].append(zeroVec.copy())

        # Variables for tf values
        self.transform = {
            'initialized': False,
            'i': 0,
            'order': 50,
            # structured self.transform['translation'][self.transform['i']][0/1/2/3 to select corresponding axes]
            'translation':[],
            'rotation':[],
            'faulty': 0
        }
        for i in range(self.transform['order']):
            self.transform['translation'].append([0.0, 0.0, 0.0])
                # translation order: [x, y, z]
            self.transform['rotation'].append([0.0, 0.0, 0.0, 0.0])
                # rotation order: [x, y, z, w]

        # zeroTr = dict()
        # zeroRo = dict()
        # for axis in 'xyz':
        #     zeroTr[axis] = 0.0
        #     zeroRo[axis] = 0.0
        # zeroRo['w'] = 0.0
        # for i in range(self.transform['order']):
        #     self.transform['translation'].append(zeroTr.copy())
        #     self.transform['rotation'].append(zeroRo.copy())

        self.transform_init = {
            'i': 0,
            'order': 100,
            'translation': [],
            'rotation': []
        }
        for i in range(self.transform_init['order']):
            self.transform_init['translation'].append([0.0, 0.0, 0.0])
                # translation order: [x, y, z]
            self.transform_init['rotation'].append([0.0, 0.0, 0.0, 0.0])
                # rotation order: [x, y, z, w]
        
        self.transform_categories = {
            'length': 0,
            'count': 0,
            'done': False,
            'data':{}
        }
        
        # Variables for forces-x --> used in Admittance control, UR specific.
        # For now, these variables are used in impedance control.
        self.forces = {
            'i': 0, # index we are on out of the 60 values (0-59)
            'order': 59, # how many we are keeping for filtering
            'fx': [], # different from Sawyer, this gathers the wrench values, forces in x-direction
            'fy': [],
            'fz': []
        }
        self.forces['fx'] = [0]*self.forces['order']
        self.forces['fy'] = [0]*self.forces['order']
        self.forces['fz'] = [0]*self.forces['order']
        self.FORCE_STANDARDIZATION_CONSTANT = .2

        # Variables for a low-pass effort filter.
        self.eff_filter = {
            'i': 0,
            'order': 60,
            'effort': {}
        }
        for name in JOINT_CONTROL_NAMES:
            self.eff_filter['effort'][name] = [0.0] * self.eff_filter['order']

        # Variables used in filtering velocity output in admittance control
        self.vel_filter_adm = {
            'i': 0,
            'velocity': {},
            'order': 1,
            'vel_cmd_prev': {},
            'K_past': 0.5
        }
        for name in JOINT_CONTROL_NAMES:
            self.vel_filter_adm['velocity'][name] = [0.0] * self.vel_filter_adm['order']
            self.vel_filter_adm['vel_cmd_prev'][name] = 0.0

        # Variables used in filtering velocity output in impedance control
        self.vel_filter_imp = {
            'i': 0,
            'order': 7,
            'velocity': {}
        }
        for name in JOINT_CONTROL_NAMES:
            self.vel_filter_imp['velocity'][name] = [0.0] * self.vel_filter_imp['order']

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
        self.endpoint_topic = rospy.Publisher('/teachbot/EndpointInfo', EndpointInfo, queue_size=1)
        # self.wrench_topic = rospy.Publisher('')
        self.gripper_command_pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=10)

        # Publish topics to UR
        self.publish_velocity_to_robot = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=1)

        # Subscribed Topics
        rospy.Subscriber('/joint_states', sensor_msgs.msg.JointState, self.forwardJointState)
        rospy.Subscriber('/tf', tfMessage, self.forwardEndpointInfo)
        rospy.Subscriber('/wrench', WrenchStamped, self.cb_filter_forces)
        rospy.Subscriber('/ur_hardware_interface/robot_mode', RobotMode, self.cb_ReadRobotState)
        rospy.Subscriber("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input, self.printGripperStatus)

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

        res = self.switch_controller([],['scaled_pos_traj_controller','joint_group_vel_controller'], 1, True, 10.0)
        res = self.switch_controller(['scaled_pos_traj_controller'],[], 1, True, 10.0)

        # Initialize position
        self.joint_traj_client.wait_for_server()
        initialize_msg = FollowJointTrajectoryGoal()
        initialize_msg.trajectory = self.create_traj_goal(joint_dof_start, speed_ratio=0.8)
        self.joint_traj_client.send_goal(initialize_msg)
        self.joint_traj_client.wait_for_result()

        self.initialize_gripper()
        rospy.loginfo('TeachBot is initialized and ready to go.')

        urdf_str = rospy.get_param('/robot_description')
        self.ik_solver = IK("base", "tool0", urdf_string=urdf_str)

        self.temp_j = [0.0]*6
        self.temp_tf = [0.0]*7
        self.qinit = [0.0]*6

        self.iktest_qinit = deg_to_rad([-80, -110, -60, -90, 65, 30])
        # self.iktest_pgoal = [0.5384, -0.4975, 0.452, -0.2523, 0.8498, -0.4381, 0.1494] # works but q is unknown
        self.iktest_pgoal = [0.3125, -0.615, -0.1562, -0.3024, 0.9401, -0.1391, 0.074]

        # self.print_tf_periodic()
        # self.find_ik()

        # parser_tree = kdl_parser.treeFromParam('/robot_description')
        # print(parser_tree[1].getNrOfJoints())
        # chain = parser_tree[1].getChain('base_link', 'tool0')        
        # grav_vector = kdl.Vector(0, 0, -9.81)  # relative to kdl chain base link
        # dyn_kdl = kdl.ChainDynParam(chain, grav_vector)
        # jt_positions = kdl.JntArray(6)
        # jt_positions[0] = 0.0
        # jt_positions[1] = -90/180*3.14
        # jt_positions[2] = -0/180*3.14
        # jt_positions[3] = -90/180*3.14
        # jt_positions[4] = -0/180*3.14
        # jt_positions[5] = 0.0

        # grav_matrix = kdl.JntArray(6)
        # dyn_kdl.JntToGravity(jt_positions, grav_matrix)

        # gravity_compensating_jt_torques = [grav_matrix[i] for i in range(grav_matrix.rows())]
        # print(gravity_compensating_jt_torques)

        # print([self.control['effort'][self.control['i']]['right_j'+str(idx)] for idx in range(6)])

        # quat = [self.transform['rotation'][self.transform['i']][axis] for axis in 'xyzw']
        # print(quat)
        # r = R.from_quat(quat)
        # print(r.as_dcm())

        
    '''
    Temporary functions for testing some functionalities.
    '''

    def print_q_periodic(self):
        for j in range(Module.JOINTS):
            self.temp_j[j] = round(self.control['position'][self.control['i']]['right_j'+str(j)], 4)
        print(self.temp_j)
        self.thread_print = threading.Timer(1, self.print_q_periodic)
        self.thread_print.start()

    def print_tf_periodic(self):
        for i, axis in enumerate('xyz'):
            self.temp_tf[i] = round(self.transform['translation'][self.transform['i']][axis], 4)
            self.temp_tf[i+3] = round(self.transform['rotation'][self.transform['i']][axis], 4)
        self.temp_tf[i+3] = round(self.transform['rotation'][self.transform['i']][axis], 4)
        print(self.temp_tf)
        self.thread_print = threading.Timer(1, self.print_tf_periodic)
        self.thread_print.start()


    def initialize_tf(self, tf, finalize=False):
        if finalize:
            for i in range(self.transform['order']):
                self.transform['translation'][i] = self.transform_categories['data']['max_key'][-self.transform['order']+i]
            self.transform['initialized'] = True
        else:
            for key in self.transform_categories['data'].keys():
                diff = np.subtract(self.transform_categories['data'][key][0], tf)
                diff_sum = np.sum(np.absolute(diff))
                print(math.sqrt(diff_sum))
                if diff_sum < 0.05:
                    self.transform_categories['data'][key].append(tf)
                    return
            self.transform_categories['data'][str(self.transform_categories['length'])] = [tf]
            self.transform_categories['length'] += 1


    def find_ik(self):
        for j in range(Module.JOINTS):
            self.qinit[j] = self.control['position'][self.control['i']]['right_j'+str(j)]
        ik_solution = self.ik_solver.get_ik(
            self.qinit,
            self.iktest_pgoal[0],
            self.iktest_pgoal[1],
            self.iktest_pgoal[2],
            self.iktest_pgoal[3],
            self.iktest_pgoal[4],
            self.iktest_pgoal[5],
            self.iktest_pgoal[6])
        if ik_solution != None:
            print([round(j, 1) for j in rad_to_deg(ik_solution)])
        else:
            print("None")
        self.thread_test = threading.Timer(0.2, self.find_ik)
        self.thread_test.start()

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
                    self.current_controller = CONTROLLERS[controller.lower()]
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
        temp_joint_angles = []
        for j in range(Module.JOINTS):
            setattr(position, 'j'+str(j), data.position[j])
            setattr(velocity, 'j'+str(j), data.velocity[j])
            setattr(effort, 'j'+str(j), data.effort[j])

            # Update self.control which stores joint states to be used by admittance control + other functions
            # position, velocity keep 15 values (for filtering later), so set the self.control['i'] index's j'th point to this value
            self.control['position'][self.control['i']]['right_j'+str(j)] = data.position[j] 
            self.control['velocity'][self.control['i']]['right_j'+str(j)] = data.velocity[j]
            self.control['effort'][self.control['i']]['right_j'+str(j)] = data.effort[j]
            self.eff_filter['effort']['right_j'+str(j)][self.eff_filter['i']] = data.effort[j]
        
        # Robot orders joint names alphabetically, so elbow comes first and shoulder_pan comes third.
        # Code below swaps those two to fix the order.
        temp_p = self.control['position'][self.control['i']]['right_j0']
        self.control['position'][self.control['i']]['right_j0'] = self.control['position'][self.control['i']]['right_j2']
        self.control['position'][self.control['i']]['right_j2'] = temp_p
        temp_v = self.control['velocity'][self.control['i']]['right_j0']
        self.control['velocity'][self.control['i']]['right_j0'] = self.control['velocity'][self.control['i']]['right_j2']
        self.control['velocity'][self.control['i']]['right_j2'] = temp_v
        temp_e = self.control['effort'][self.control['i']]['right_j0']
        self.control['effort'][self.control['i']]['right_j0'] = self.control['effort'][self.control['i']]['right_j2']
        self.control['effort'][self.control['i']]['right_j2'] = temp_e
        
        # since we are trying to keep only 'order' values for filtering, +1 if i is less than 'order' and reset to 0 otherwise 
        self.control['i'] = self.control['i']+1 if self.control['i']+1<self.control['order'] else 0
        self.eff_filter['i'] = self.eff_filter['i']+1 if self.eff_filter['i']+1<self.eff_filter['order'] else 0

        # Publish joint state information to the browser 
        self.position_topic.publish(position)
        self.velocity_topic.publish(velocity)
        # TODO: add effort topic to publish (if needed)

        # for key, val in self.control['effort'][self.control['i']].items():
        #     self.control['effort'][self.control['i']][key] = round(val, 4)

        # print(self.control['effort'][self.control['i']].values())


    '''
    Forward endpoint information to Module.js so that the information can be used
    on the browser interaction.

    For some reason, endpoint info publishes wrong data erratically. The bad data
    for each of the data seems to be constant so an if statement is used to filter
    out bad data. Also, for translation data, wrong data appears at the same time
    for x, y, and z. Those 3 values are their all okay or bad simultaneously.
    CAUTION: VALUE OF BAD DATA ACROSS DIFFERENT ROBOTS MAY BE DIFFERENT. IF THIS
    MESSAGE IS STILL PRESENT, YOU NEED TO CHECK WITH DIFFERENT ROBOTS!
    '''
    def forwardEndpointInfo(self, data):
        endpoint_msg = EndpointInfo()
        tf_data = data.transforms[0].transform
        if not self.transform['initialized']:
            tf = [0.0] * 7
            tf[0] = tf_data.translation.x
            tf[1] = tf_data.translation.y
            tf[2] = tf_data.translation.z
            tf[3] = tf_data.rotation.x
            tf[4] = tf_data.rotation.y
            tf[5] = tf_data.rotation.z
            tf[6] = tf_data.rotation.w
            if self.transform_categories['count'] < 2000:
                self.transform_categories['count'] += 1
                self.initialize_tf(tf)
            else:
                if not self.transform_categories['done']:
                    self.transform_categories['done'] = True
                    print('Done.')
                    max_key = self.transform_categories['data'].keys()[0]
                    for key, val in self.transform_categories['data'].items():
                        if len(val) > len(self.transform_categories['data'][max_key]):
                            max_key = key
                        print("key " + str(key) + " has " + str(len(val)) + " elements.")
                    print("key with max values is " + str(max_key))
                    self.transform_categories['max_key'] = max_key
                    self.initialize_tf(tf, finalize=True)
        else:
            # Error data occurs at the same time.
            if (abs(tf_data.translation.x)-0.425<0.01): # NOT DONE YET!
                endpoint_msg.position.x = self.endpoint_prev[0]
                endpoint_msg.position.y = self.endpoint_prev[1]
                endpoint_msg.position.z = self.endpoint_prev[2]
                endpoint_msg.orientation.x = self.endpoint_prev[3]
                endpoint_msg.orientation.y = self.endpoint_prev[4]
                endpoint_msg.orientation.z = self.endpoint_prev[5]
                endpoint_msg.orientation.w = self.endpoint_prev[6]
                endpoint_msg.false_data = 1
            else:
                endpoint_msg.position.x = tf_data.translation.x
                endpoint_msg.position.y = tf_data.translation.y
                endpoint_msg.position.z = tf_data.translation.z
                endpoint_msg.orientation.x = tf_data.rotation.x
                endpoint_msg.orientation.y = tf_data.rotation.y
                endpoint_msg.orientation.z = tf_data.rotation.z
                endpoint_msg.orientation.w = tf_data.rotation.w
                self.endpoint_prev[0] = tf_data.translation.x
                self.endpoint_prev[1] = tf_data.translation.y
                self.endpoint_prev[2] = tf_data.translation.z
                self.endpoint_prev[3] = tf_data.rotation.x
                self.endpoint_prev[4] = tf_data.rotation.y
                self.endpoint_prev[5] = tf_data.rotation.z
                self.endpoint_prev[6] = tf_data.rotation.w

            # Increment/Zero the index i in self.transform
            self.transform['i'] = self.transform['i']+1 if self.transform['i']+1<self.transform['order'] else 0

            self.endpoint_topic.publish(endpoint_msg)

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
        # print(sum(self.forces['fx'])/self.forces['order'])
    '''
    When the subscriber to "GoToJointAngles" receives the name of a constant from the browser,
    it is evaluated in this function and parsed into a "FollowTrajectoryGoal" object and sent to the
    action client provided by Universal Robots driver. Upon completion, it publishes to "/command-complete"
    to let the browser know that it has finished its task.
    '''
    def cb_GoToJointAngles(self, goal):

        if self.current_controller != CONTROLLERS["position"]:
            self.switchController("position")

        self.joint_traj_client.wait_for_server()

        followJoint_msg = FollowJointTrajectoryGoal()

        if goal.name != '':
            print(goal.name)
            followJoint_msg.trajectory = self.create_traj_goal(eval(goal.name), goal.speed_ratio)
        else:
            followJoint_msg.trajectory = self.create_traj_goal([goal.j0pos, goal.j1pos, goal.j2pos, goal.j3pos, goal.j4pos, goal.j5pos], goal.speed_ratio)
        
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

    speed_ratio modifies how long the arm takes to reach to the target position. The default is 1.
    It is converted into speed_ratio by a function. This is because normally, smaller speed_ratio
    is interpreted as slower speed but smaller value of arg in rospy.Duration(arg) makes the robot move faster.
    This conversion makes speed_ratio more intuitive to use.
    '''
    def create_traj_goal(self, array, speed_ratio=1):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = JOINT_NAMES

        jointPositions_msg = JointTrajectoryPoint()
        jointPositions_msg.positions = array
        if speed_ratio <= 0:
            duration = 3
        else:
            duration = 1.5/speed_ratio + 1.5
        jointPositions_msg.time_from_start = rospy.Duration(duration)

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
            # Clear any values in the filter variable so that the next admittance_control starts off clean.
            for name in JOINT_CONTROL_NAMES:
                self.vel_filter_adm['velocity'][name] = [0.0] * self.vel_filter_adm['order']
                self.vel_filter_adm['vel_cmd_prev'][name] = 0.0

        elif req.mode == 'admittance ctrl' or req.mode == 'admittance_ctrl':
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
            
            self.modeTimer = rospy.Timer(rospy.Duration(0.02), lambda event=None : self.cb_AdmittanceCtrl_new(joints, eval(req.resetPos)))

        elif req.mode == 'impedance ctrl' or req.mode == 'impedance_ctrl':
            '''
            Currently, impedance control is set to work only with one joint, specifically
            the elbow joint at one specified posture. This is due to force input in cartesian
            coordinate and movement output in joint coordinate. A more complete impedance
            control with full-arm scale is possible with full kinematics but for
            current usage of impedance control, this implementation is sufficient.

            The format below just follows the format from Saywer's version of teachbot.py and
            is not intended to use directly as is for multiple joint control.
            '''

            # Initialize Joints Dict
            joints = {}
            for j in req.joints:
                joints['right_j'+str(j)] = {}

            # Set position and velocity reference points
            x_ref = self.control['position'][self.control['i']]

            for idx, joint in enumerate(joints.values()):
                joints[joint]['P2F'] = req.P2F[idx]
                joints[joint]['F2V'] = req.F2V[idx]

            f_min_thresh = req.min_thresh
            # Switch to the joint group velocity controller
            self.switchController('velocity')

            self.modeTimer = rospy.Timer(rospy.Duration(0.004), lambda event=None : self.cb_ImpedanceCtrl(joints, eval(req.resetPos), f_min_thresh))

        elif req.mode == 'interaction_trans':
            if len(req.axis_transformation)!=0:
                arg_axis_trans = req.axis_transformation
            else:
                arg_axis_trans = ['x', 'y', 'z']

            arg_active_axes = req.active_axes

            self.switchController('velocity')

            self.modeTimer = rospy.Timer(rospy.Duration(0.05), lambda event=None: self.cb_InteractionTrans(arg_axis_trans, arg_active_axes))

        else:
            rospy.logerr('Robot mode ' + req.mode + ' is not a supported mode.')

        return True


    def cb_AdmittanceCtrl_new(self, joints, resetPos, rateNom=10, tics=15):
        '''
        Implement admittance control using effort of each joint.

        This function implements admittance control. It takes the joint effort as input and gives
        joint velocity as output. As of 11/4/2020, this function only works in the joint space, not
        in the cartesian coordinate of the end effector.

        The velocity output profile is split into two segments to avoid vibration near zero and
        abrupt acceleration from zero. An exponential velocity provile is used near zero up to a
        threshold called 'linear_threshold'. After that, the velocity profile becomes linear. The
        exponential section helps to flaten the velocity provile by eliminating discontinuity.

        Parameters
        ----------
        joints : dictionary
            A dictionary containing joint names to activate along with relevant thersholds and parameters. Check method cb_SetRobotMode for more details.
        resetPos : list of int
            joint angles when the robot goes out of boundary. Currently not used.

        Returns
        -------
        None

        '''
        log_bias = {}
        check_bias = True

        velocities = {}
        for joint in JOINT_CONTROL_NAMES:
            velocities[joint] = 0

        for joint in joints.keys():
            linear_threshold = 2 * joints[joint]['min_thresh']
            filtered_effort = sum([self.control['effort'][i][joint] for i in range(self.control['order'])])
            filtered_effort /= self.control['order']
            vel_cmd = filtered_effort - joints[joint]['bias']

            # BIAS FINDER CODE, not needed for teachbot to run properly.
            log_bias[joint] = round(filtered_effort, 3)

            if abs(vel_cmd) < joints[joint]['min_thresh']:
                vel_cmd = 0
            elif abs(vel_cmd) < linear_threshold:
                # if vel_cmd > 0:
                #     vel_cmd -= joints[joint]['min_thresh']
                # else:
                #     vel_cmd += joints[joint]['min_thresh']
                vel_cmd = -joints[joint]['F2V'] / (2*linear_threshold) * vel_cmd * abs(vel_cmd)
            else:
                # vel_cmd = -joints[joint]['F2V'] / 2 * linear_threshold * vel_cmd/abs(vel_cmd)
                if vel_cmd > 0:
                    vel_cmd -= joints[joint]['min_thresh']
                else:
                    vel_cmd += joints[joint]['min_thresh']
                vel_cmd *= -joints[joint]['F2V']

            self.vel_filter_adm['velocity'][joint][self.vel_filter_adm['i']] = vel_cmd
            vel_cmd = sum(self.vel_filter_adm['velocity'][joint]) / self.vel_filter_imp['order']
            velocities[joint] = self.vel_filter_adm['K_past']*self.vel_filter_adm['vel_cmd_prev'][joint] + (1-self.vel_filter_adm['K_past'])*vel_cmd
            if abs(velocities[joint]) < 0.001:
                velocities[joint] = 0.0
            self.vel_filter_adm['vel_cmd_prev'][joint] = velocities[joint]
            # velocities[joint] = sum(self.vel_filter_adm['velocity'][joint]) / self.vel_filter_imp['order']

        self.vel_filter_adm['i'] = self.vel_filter_adm['i']+1 if self.vel_filter_adm['i']+1<self.vel_filter_adm['order'] else 0

        # rospy.loginfo(velocities)
        velocity_msg = Float64MultiArray()

        velocity_data = []
        for j in range(len(velocities.keys())):
            velocity_data.append(velocities['right_j'+str(j)])
        
        if check_bias:
            rospy.loginfo('bias mode: '+str(log_bias))
        else:
            velocity_msg.data = velocity_data
            rospy.loginfo(velocity_data)
            self.publish_velocity_to_robot.publish(velocity_msg)


    def cb_AdmittanceCtrl(self, joints, resetPos, rateNom=10, tics=15):
        '''
        Deprecated! Use cb_AdmittanceCtrl_new. Delete this function after you check that
        cb_AdmittanceCtrl is fully functional.
        '''

        # new velocities to send to robot
        velocities = {}
        for joint in range(len(joints.keys())):
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
    def cb_ImpedanceCtrl(self, joints, resetPos, f_min_thresh, rateNom=10, tics=15):
        '''
        Impedance control implementation

        This is a simplified impedance control in joint space only. In other words, you can only
        achieve impedance control for individual joints and the system has no knowledge of its
        location in the cartesian coordinate. Also, the impedance control is only tested when
        only one joint is activated.
        '''

        # Define some constants used in this callback func.
        F_threshold = f_min_thresh
        F_cutoff = 2 * F_threshold
        f_max_offset = 8.7

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

            # P2F = Kpf * delta_pos
            P2F = joints[joint]['P2F'] * delta_pos
            force_input += P2F

            if abs(force_input) < F_cutoff:
                # F2V = -Kfv / (2 * F_cutoff) * force_input * abs(force_input)
                F2V = -joints[joint]['F2V'] / (2 * F_cutoff) * force_input * abs(force_input)
            else:
                if (force_input < 0):
                    # F2V = -Kfv * (force_input + F_threshold)
                    F2V = -joints[joint]['F2V'] * (force_input + F_threshold)
                else:
                    # F2V = -Kfv * (force_input - F_threshold)
                    F2V = -joints[joint]['F2V'] * (force_input - F_threshold)

            self.vel_filter_imp['velocity'][joint][self.vel_filter_imp['i']] = F2V
            velocities[joint] = sum(self.vel_filter_imp['velocity'][joint]) / self.vel_filter_imp['order']

        self.vel_filter_imp['i'] = self.vel_filter_imp['i']+1 if self.vel_filter_imp['i']+1<self.vel_filter_imp['order'] else 0

        velocities_data = []
        for i in range(len(JOINT_CONTROL_NAMES)):
            velocities_data.append(velocities['right_j'+str(i)])

        velocity_msg = Float64MultiArray()
        velocity_msg.data = velocities_data
        print(velocities_data)
        # self.publish_velocity_to_robot.publish(velocity_msg)

    def cb_InteractionTrans(self, axis_transformation, active_axes):
        '''
        This callback function implements interaction control but with
        fixed angles of the end effector. This means that it moves
        in translational directions (x, y, and z) but the end-
        effector does not rotate. A full interaction control
        requires rotation matrix, forward kinematics and stuff, and
        even without a full interaction control, teachbot can run 90%
        of the code. As such, it is not implemented. The methods
        used in this function such as mapping xyz from tcp to base only
        applies to this simplified interaction control. DO NOT use it
        outside of this function!

        Future To-Do: implement full interaction control.

        Edit:
        Current work is not complete and may only work in situations where the coordinate of end effector
        is aligned to that of the base in multiple of 90 degrees (90, 180, 270, 360). For example, if the
        end effector coordinate is rotated 32 degrees in x axis, this implementation will not work.
        
        My advice is to implement a full interaction control involving full kinematics. 
        
        '''

        axis_mapping = {
            'x':1,
            '-x':-1,
            'y':2,
            '-y':-2,
            'z':3,
            '-z':-3
        }
        # Get initial joint angles for IK solver
        qinit = [0] * 6
        for j in range(Module.JOINTS):
            qinit[j] = self.control['position'][self.control['i']]['right_j'+str(j)]

        # Axis transformation
        trans_mat = np.zeros((3,3))
        for row, axis in enumerate(axis_transformation):
            col = abs(axis_mapping[axis])-1
            trans_mat[row][col] = axis_mapping[axis]/abs(axis_mapping[axis])

        # Get force applied to the gripper
        force_input = []
        force_input.append(self.forces['fx'][self.forces['i']])
        force_input.append(self.forces['fy'][self.forces['i']])
        force_input.append(self.forces['fz'][self.forces['i']])
        force_input = np.matmul(trans_mat, np.array(force_input))
        force_input = np.multiply(force_input, active_axes)

        test_quat = [self.transform['rotation'][self.transform['i']][axis] for axis in 'xyzw']
        rotation_matrix = quat_to_rotmatrix(test_quat)
        v_g_tcp = np.matmul(rotation_matrix, [0,0,-9.81])

        # ik_solution = self.ik_solver.get_ik(
        #     self.qinit,
        #     self.iktest_pgoal[0],
        #     self.iktest_pgoal[1],
        #     self.iktest_pgoal[2],
        #     self.iktest_pgoal[3],
        #     self.iktest_pgoal[4],
        #     self.iktest_pgoal[5],
        #     self.iktest_pgoal[6])
        # if ik_solution != None:
        #     print([round(j, 1) for j in rad_to_deg(ik_solution)])
        # else:
        #     print("None")
        pass

# Utility functions used across different methods of class Module
def quat_to_rotmatrix(joint_quat):
    '''This funciton calculates the rotation matrix from joint quaternion.

    Arguments: list of joint quaternion in the order of x, y, z, w.
    Returns: rotation matrix in numpy array matrix.
    '''
    rot_mat = np.zeros((3,3))
    x2 = joint_quat[0]*joint_quat[0]
    y2 = joint_quat[1]*joint_quat[1]
    z2 = joint_quat[2]*joint_quat[2]

    xy = joint_quat[0]*joint_quat[1]
    xz = joint_quat[0]*joint_quat[2]
    yz = joint_quat[1]*joint_quat[2]

    wx = joint_quat[3]*joint_quat[0]
    wy = joint_quat[3]*joint_quat[1]
    wz = joint_quat[3]*joint_quat[2]

    rot_mat[0][0] = 1 - 2*y2 - 2*z2
    rot_mat[0][1] = 2*xy + 2*wz
    rot_mat[0][2] = 2*xz - 2*wy
    rot_mat[1][0] = 2*xy - 2*wz
    rot_mat[1][1] = 1 - 2*x2 - 2*z2
    rot_mat[1][2] = 2*yz + 2*wx
    rot_mat[2][0] = 2*xz + 2*wy
    rot_mat[2][1] = 2*yz - 2*wx
    rot_mat[2][2] = 1 - 2*x2 - 2*y2

    return rot_mat

def deg_to_rad(list_q):
    return [radians(j) for j in list_q]

def rad_to_deg(list_q):
    return [degrees(j) for j in list_q]

if __name__ == '__main__':
    ## DEFINE IMPORTANT CONSTANTS --- MAKE SURE THEY MATCH WITH MODULE 1 OR 2 CONSTANTS ##

    # POSITION CONSTANTS - ARRAYS THAT MATCH JOINT_NAMES
    ZERO = [0, -1.57, 0, -1.57, 0, 0]
    default = [0, -1.57, 0, -1.57, 0, 0]
    to_be_tuned = deg_to_rad([-90, -120, -60, -90, -90, 180])
    # SCARA = [0, -3.14, 0, -3.14, -1.57, 0] # OLD SCARA POSE, COLLIDE WITH THE TABLE
    SCARA = deg_to_rad([-60, -150, -60, -150, -90, 180])
    WRIST_3_FWD = [0, -3.14, 0, -3.14, -1.57, -.5]
    WRIST_2_FWD = [0, -3.14, 0, -3.14, -1.0, 0]
    WRIST_1_FWD = [0, -3.14, 0, -2.54, -1.57, 0]
    ELBOW_FWD = [0, -3.14, 0.5, -3.14, -1.57, 0]
    SHOULDER_FWD = [0, -2.80, 0, -3.14, -1.57, 0]
    BASE_FWD = [0.50, -3.14, 0, -3.14, -1.57, 0]
    test_base = deg_to_rad([-30, -150, -60, -150, -90, 180])
    impedance_start = deg_to_rad([-90, -180, 90, -90, 90, 180])

    # TODO: Figure out what no_hit is and what j4 max is    
    # TODO: Figure out DPS becaues the arm needs to be over the table for this part
    DSP = SCARA[1]
    j2scara = SCARA[2]
    over_table = -3.5
    j4max = SCARA[4]

    # 1
    joint_motor_animation_0 = SCARA[:]
    joint_motor_animation_1 = deg_to_rad([-60, -150, 0.0, -150, -90, 180])
    joint_motor_animation_2 = deg_to_rad([-60, -120, 0.0, -150, -90, 180])
    # 4
    span_test =  [joint for joint in joint_motor_animation_0]
    span_test[0] += 0.3
    slift_test = [joint for joint in joint_motor_animation_0]
    slift_test[1] += 0.3
    elbow_test = [joint for joint in joint_motor_animation_0]
    elbow_test[2] += 0.3
    wrist1_test = [joint for joint in joint_motor_animation_0]
    wrist1_test[3] += 0.8
    wrist2_test = [joint for joint in joint_motor_animation_0]
    wrist2_test[4] += 0.8
    wrist3_test = [joint for joint in joint_motor_animation_0]
    wrist3_test[5] += 0.8
    joint_test = [span_test, slift_test, elbow_test, wrist1_test, wrist2_test, wrist3_test]
    # joint_test = [0]*Module.JOINTS
    # joint_test = [WRIST_3_FWD, WRIST_2_FWD, WRIST_1_FWD, ELBOW_FWD, SHOULDER_FWD, BASE_FWD]
    
    # 6 - 15
    # TODO Find the hard-coded values that work for UR
    # joint_dof_shoulder = [-1.57, -2.80 ,j2scara,0,-j4max,0]
    # joint_dof_elbow = [-1.57, DSP, .30, 0,-j4max,0]
    # joint_dof_wrist = [-1.57, DSP,j2scara,0, 0.45,0]
    # joint_dof_up = [-1.57, DSP, j2scara,0,-j4max,0]
    # for now use the following so it goes through this section of the module but doesn't swing around like crazy
    joint_dof_start =       deg_to_rad([-60, -150, -60, -150, -90, 180]) # SCARA
    joint_dof_shoulder =    deg_to_rad([-90, -150, -60, -150, -90, 180])
    joint_dof_elbow =       deg_to_rad([-60, -120, -90, -150, -90, 180])
    joint_dof_wrist =       deg_to_rad([-60, -150, -60, -150, -45, 180])
    joint_dof_up = SCARA
    # TEMP position for development at BIC.
    joint_dof_start_BIC =   deg_to_rad([-130, -150, -60, -150, -90, 180])

    pos_encoder_video = deg_to_rad([-90, -90, -60, -180, -90, 180])

    # "2dPosition" section
    kinematics_init_pos =       deg_to_rad([-55, -125, -118, -116, -90, 180])
    kinematics_shoulder_arc =   deg_to_rad([-85, -125, -118, -116, -90, 180]) # from kinematics_init_pos
    joint_dot_1 =               to_be_tuned
    joint_dot_2 =               to_be_tuned
    joint_arc_wrist =           to_be_tuned
    joint_ortho_kin =           to_be_tuned
    joint_arc_shoulder =        to_be_tuned
    
    joint_dot_3_init =          to_be_tuned
    joint_dot_3 =               to_be_tuned
    
    joint_dot_3_2_1 =           to_be_tuned
    joint_dot_3_2_2 =           joint_dot_3
    
    joint_dot_3_4_1 =           to_be_tuned
    joint_dot_3_4_2 =           to_be_tuned
    joint_dot_3_4_3 =           to_be_tuned
    joint_dot_3_4_4 =           joint_dot_3

    joint_dot_3_8_1 =           to_be_tuned
    joint_dot_3_8_2 =           to_be_tuned
    joint_dot_3_8_3 =           to_be_tuned
    joint_dot_3_8_4 =           to_be_tuned
    joint_dot_3_8_5 =           to_be_tuned
    joint_dot_3_8_6 =           to_be_tuned
    joint_dot_3_8_7 =           to_be_tuned
    joint_dot_3_8_8 =           joint_dot_3

    # "feedback" section
    point_a =               to_be_tuned
    point_b =               to_be_tuned
    point_c =               to_be_tuned

    joint_push_forward =    impedance_start

    m = Module()

    try:
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    finally:
        m.thread_print.cancel()
        m.thread_test.cancel()