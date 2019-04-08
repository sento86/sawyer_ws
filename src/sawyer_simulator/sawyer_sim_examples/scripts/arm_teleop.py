#!/usr/bin/env python
from _socket import timeout

# Copyright (c) 2015-2018, Rethink Robotics Inc.
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

"""
Sawyer SDK Inverse Kinematics Pick and Place Demo
"""

#!/usr/bin/env python
import argparse
import struct
import sys

from copy import copy, deepcopy
#import copy
import copy

#from future.backports.test.support import verbose

import rospy
import rospkg

import actionlib

import numpy as np
import math
import random
import time

import intera_interface
from intera_interface import CHECK_VERSION

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)
from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
    PointStamped,
    WrenchStamped,
    Twist,
    TwistStamped,
)
from std_msgs.msg import (
    Header,
    Empty,
    UInt16,
    Bool,
)
from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from sensor_msgs.msg import (
    Joy,
    JointState
)
from intera_core_msgs.msg import (
    JointCommand,
    SEAJointState,
    EndpointState
)

#from intera_tools import Tuck
#from tuck_arms import Tuck
from sawyer_pykdl import sawyer_kinematics

from tf import TransformListener
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from bson.json_util import default
from intera_core_msgs.msg._EndpointState import EndpointState

global efforts
efforts = [0, 0, 0, 0, 0, 0, 0]

from pid import PID

import intera_interface

####################################################################################

def load_gazebo_models(table_pose=Pose(position=Point(x=0.75, y=0.0, z=0.0)),
                       table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.4225, y=0.1265, z=0.7725)),
                       block_reference_frame="world"):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('sawyer_sim_examples')+"/models/"
    # Load Table SDF
    table_xml = ''
    with open (model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')
    # Load Block URDF
    block_xml = ''
    with open (model_path + "block/model.urdf", "r") as block_file:
        block_xml=block_file.read().replace('\n', '')
    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    # Spawn Block URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("cafe_table")
        resp_delete = delete_model("block")
    except rospy.ServiceException, e:
        print("Delete Model service call failed: {0}".format(e))

####################################################################################

class TeleopControl(object):

    def __init__(self, limb, hover_distance = 0.15, sim = True, control = 'postition', device = 'falcon', server = True, verbose = True, tip_name="right_gripper_tip"):
        if limb=='right':
            self._limb_sign = 1.0
        elif limb=='left':
            self._limb_sign = -1.0
        else:
            self._limb_sign = 0.0
        self._limb_name = limb # string
        self._tip_name = tip_name # string
        self._sim = sim # Mode simulation/real
        self._control = control # Control position/velocity/effort
        self._device = device # Device falcon/phantom/xsens
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._limb = intera_interface.Limb(limb)
        #self._limb = intera_interface.limb.Limb(limb)
        #self._arm = intera_interface.limb.Limb(limb)
        #self._gripper = intera_interface.Gripper(limb)
     #   self._gripper = intera_interface.Gripper(limb+'_gripper')
        #self._gripper = intera_interface.Gripper()
#         ns_ik = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
#         self._iksvc = rospy.ServiceProxy(ns_ik, SolvePositionIK)
#         rospy.wait_for_service(ns_ik, 5.0)
        
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
        #self._rs = intera_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
   
        
        print("Running. Ctrl-c to quit")
        positions = {
            'left':  [-0.11, -0.62, -1.15, 1.32,  0.80, 1.27,  2.39],
            'right':  [0.11, -0.62,  1.15, 1.32, -0.80, 1.27, -2.39],
        }

#         if self._sim==True:
#             # Wait for the All Clear from emulator startup
#             rospy.wait_for_message("/robot/sim/started", Empty) # Comment this line to control the real intera (uncomment to simulated robot)

        #print("Setting up the tuck service")
        #self._tuck = Tuck(True)
        #self._untuck = Tuck(False)

        self._kin = sawyer_kinematics(limb)
        #self._joint_names = self._arm.joint_names()
        self._joint_names = self._limb.joint_names()
        self._angles = self._limb.joint_angles()
        self._velocities = self._limb.joint_velocities()
        self._efforts = self._limb.joint_efforts()
        self._pose = self._limb.endpoint_pose()

        self._cbJoy = rospy.Subscriber("joy", Joy, self.callback_joy)
        self._cbCmdJoint = rospy.Subscriber(limb+"/cmd_joint", JointState, self.callback_cmd_joint, queue_size=1)
        #self._cbCmdJointPhantom = rospy.Subscriber(limb+"/joint_states/phantom", JointState, self.callback_cmd_joint_phantom, queue_size=1)
        self._cbCmdJointPhantom = rospy.Subscriber(limb+"/cmd_joint/phantom", JointState, self.callback_cmd_joint_phantom, queue_size=1)
        self._cbCmdJointFalcon = rospy.Subscriber(limb+"/cmd_joint/falcon", JointState, self.callback_cmd_joint_falcon, queue_size=1)
        self._cbCmdJointXsens = rospy.Subscriber(limb+"/cmd_joint/xsens", JointState, self.callback_cmd_joint_xsens, queue_size=1)
        self._cbCmdVel = rospy.Subscriber(limb+"/cmd_vel", TwistStamped, self.callback_cmd_vel, queue_size=1)
        self._cbCmdPose = rospy.Subscriber(limb+"/cmd_pose", PoseStamped, self.callback_cmd_pose, queue_size=1)
        self._cbCmdGripper = rospy.Subscriber(limb+"/cmd_gripper", Header, self.callback_cmd_gripper, queue_size=1)
        if limb=='right':
            self._cbState = rospy.Subscriber("/robot/limb/left/endpoint_state", EndpointState, self.callback_state, queue_size=1)
            self._pub_collision = rospy.Publisher('/robot/limb/right/suppress_collision_avoidance', Empty, queue_size=10)
        elif limb=='left':
            self._cbState = rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, self.callback_state, queue_size=1)
            self._pub_collision = rospy.Publisher('/robot/limb/left/suppress_collision_avoidance', Empty, queue_size=10)
        self._joy = Joy
        self._cmd_joint = JointState
        self._cmd_joint_phantom = JointState
        self._cmd_joint_falcon = JointState
        self._cmd_joint_xsens = JointState
        self._cmd_vel = TwistStamped
        self._cmd_pose = PoseStamped
        self._cmd_gripper = Header
  
        self.q0 = 0.0 # s0
        self.q1 = 0.0 # s1
        self.q2 = 0.0 # e0
        self.q3 = 0.0 # e1
        self.q4 = 0.0 # w0
        self.q5 = 0.0 # w1
        self.q6 = 0.0 # w2
  
        self.e0 = 0.0 # s0
        self.e1 = 0.0 # s1
        self.e2 = 0.0 # e0
        self.e3 = 0.0 # e1
        self.e4 = 0.0 # w0
        self.e5 = 0.0 # w1
        self.e6 = 0.0 # w2
  
        self.u0 = 0.0 # s0
        self.u1 = 0.0 # s1
        self.u2 = 0.0 # e0
        self.u3 = 0.0 # e1
        self.u4 = 0.0 # w0
        self.u5 = 0.0 # w1
        self.u6 = 0.0 # w2
        
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.wx = 0.0
        self.wy = 0.0
        self.wz = 0.0
        
        self.dx = 0.0
        self.dy = 0.0
        self.dz = 0.0
        self.dR = 0.0
        self.dP = 0.0
        self.dY = 0.0
        
        self.ex = 0.0
        self.ey = 0.0
        self.ez = 0.0
        self.eR = 0.0
        self.eP = 0.0
        self.eY = 0.0
        
        self.ux = 0.0
        self.uy = 0.0
        self.uz = 0.0
        self.uR = 0.0
        self.uP = 0.0
        self.uY = 0.0

        if self._sim==True:
            
            # Cartesian PID control (values for simulated intera robot)
            self.vx_pid = PID(0.5, 0.0, 0.1, 1.0, -1.0)
            self.vy_pid = PID(0.5, 0.0, 0.1, 1.0, -1.0)
            self.vz_pid = PID(0.5, 0.0, 0.1, 1.0, -1.0)
            self.wx_pid = PID(0.2, 0.0, 0.1, 1.0, -1.0)
            self.wy_pid = PID(0.2, 0.0, 0.1, 1.0, -1.0)
            self.wz_pid = PID(0.2, 0.0, 0.1, 1.0, -1.0)

            # Joint PID control (values for simulated intera robot)
#             self.q0_pid = PID(40.0, 0.0, 0.04, 1.0, -1.0)
#             self.q1_pid = PID(20.0, 0.0, 0.02, 1.0, -1.0)
#             self.q2_pid = PID(20.0, 0.0, 0.02, 1.0, -1.0)
#             self.q3_pid = PID(10.0, 0.0, 0.01, 1.0, -1.0)
#             self.q4_pid = PID(10.0, 0.0, 0.01, 1.0, -1.0)
#             self.q5_pid = PID(10.0, 0.0, 0.01, 1.0, -1.0)
#             self.q6_pid = PID(10.0, 0.0, 0.01, 1.0, -1.0)
            
            self.q0_pid = PID(20.0, 0.0, 0.2, 1.0, -1.0)
            self.q1_pid = PID(20.0, 0.0, 0.05, 1.0, -1.0)
            self.q2_pid = PID(20.0, 0.0, 0.05, 1.0, -1.0)
            self.q3_pid = PID(10.0, 0.0, 0.01, 1.0, -1.0)
            self.q4_pid = PID(10.0, 0.0, 0.01, 1.0, -1.0)
            self.q5_pid = PID(10.0, 0.0, 0.01, 1.0, -1.0)
            self.q6_pid = PID(10.0, 0.0, 0.01, 1.0, -1.0)
        
        else:
            
            if self._control=='position':

                if self._limb_name=='left':
                     
                    # Cartesian PID control (values for real intera robot) -> Ideal values without FF
                    self.vx_pid = PID(20.0, 0.0, 0.2, 1.0, -1.0)
                    self.vy_pid = PID(20.0, 0.0, 0.2, 1.0, -1.0)
                    self.vz_pid = PID(20.0, 0.0, 0.2, 1.0, -1.0)
                    self.wx_pid = PID(10.0, 0.0, 0.05, 1.0, -1.0)
                    self.wy_pid = PID(10.0, 0.0, 0.05, 1.0, -1.0)
                    self.wz_pid = PID(10.0, 0.0, 0.05, 1.0, -1.0)
                 
                    # Joint PID control (values for real intera robot) -> Joint position controller
                    self.q0_pid = PID(40.0, 0.0, 4.0, 1.0, -1.0)
                    self.q1_pid = PID(40.0, 0.0, 4.0, 1.0, -1.0)
                    self.q2_pid = PID(30.0, 0.0, 3.0, 1.0, -1.0) # This joint is fixed, but has to be controlled
                    self.q3_pid = PID(30.0, 0.0, 3.0, 1.0, -1.0)
                    self.q4_pid = PID(40.0, 0.0, 2.0, 1.0, -1.0)
                    self.q5_pid = PID(20.0, 0.0, 2.0, 1.0, -1.0)
                    self.q6_pid = PID(20.0, 0.0, 1.0, 1.0, -1.0)
                    
                elif self._limb_name=='right':
                    
                    # Cartesian PID control (values for real intera robot)
                    self.vx_pid = PID(20.0, 0.0, 0.2, 1.0, -1.0)
                    self.vy_pid = PID(20.0, 0.0, 0.2, 1.0, -1.0)
                    self.vz_pid = PID(20.0, 0.0, 0.2, 1.0, -1.0)
                    self.wx_pid = PID(10.0, 0.0, 0.05, 1.0, -1.0)
                    self.wy_pid = PID(10.0, 0.0, 0.05, 1.0, -1.0)
                    self.wz_pid = PID(10.0, 0.0, 0.05, 1.0, -1.0)
                 
                    # Joint PID control (values for real intera robot) -> Joint position controller
                    self.q0_pid = PID(40.0, 0.0, 4.0, 1.0, -1.0)
                    self.q1_pid = PID(40.0, 0.0, 4.0, 1.0, -1.0)
                    self.q2_pid = PID(30.0, 0.0, 3.0, 1.0, -1.0) # This joint is fixed, but has to be controlled
                    self.q3_pid = PID(30.0, 0.0, 3.0, 1.0, -1.0)
                    self.q4_pid = PID(40.0, 0.0, 2.0, 1.0, -1.0)
                    self.q5_pid = PID(20.0, 0.0, 2.0, 1.0, -1.0)
                    self.q6_pid = PID(20.0, 0.0, 1.0, 1.0, -1.0)
                
                else:
                 
                    # Cartesian PID control (values for real intera robot)
                    self.vx_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0)
                    self.vy_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0)
                    self.vz_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0)
                    self.wx_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0)
                    self.wy_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0)
                    self.wz_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0)
                    
                    # Joint PID control (values for real intera robot) -> Joint velocity controller
                    self.q0_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0)
                    self.q1_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0)
                    self.q2_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0) # This joint is fixed, but has to be controlled
                    self.q3_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0)
                    self.q4_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0)
                    self.q5_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0)
                    self.q6_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0)
            
            elif self._control=='velocity':
             
                if self._limb_name=='left':
                    
                    # Cartesian PID control (values for real intera robot)
                    if self._device=='falcon':
#                         # PID gains for Falcon -> Ideal values without FF
#                         self.vx_pid = PID(1.0, 0.0, 0.05, 1.0, -1.0)
#                         self.vy_pid = PID(1.0, 0.0, 0.05, 1.0, -1.0)
#                         self.vz_pid = PID(1.0, 0.0, 0.05, 1.0, -1.0)
#                         self.wx_pid = PID(0.2, 0.0, 0.02, 1.0, -1.0)
#                         self.wy_pid = PID(0.2, 0.0, 0.02, 1.0, -1.0)
#                         self.wz_pid = PID(0.2, 0.0, 0.02, 1.0, -1.0)
                        # PID gains for Falcon -> Ideal values with FF
                        self.vx_pid = PID(2.0, 0.0, 0.2, 1.0, -1.0)
                        self.vy_pid = PID(2.0, 0.0, 0.2, 1.0, -1.0)
                        self.vz_pid = PID(2.0, 0.0, 0.2, 1.0, -1.0)
                        self.wx_pid = PID(0.5, 0.0, 0.05, 1.0, -1.0)
                        self.wy_pid = PID(0.5, 0.0, 0.05, 1.0, -1.0)
                        self.wz_pid = PID(0.5, 0.0, 0.05, 1.0, -1.0)
                    elif self._device=='phantom':
                        # PID gains for Phantom
                        self.vx_pid = PID(2.0, 0.0, 0.2, 1.0, -1.0)
                        self.vy_pid = PID(2.0, 0.0, 0.2, 1.0, -1.0)
                        self.vz_pid = PID(2.0, 0.0, 0.2, 1.0, -1.0)
                        self.wx_pid = PID(2.0, 0.0, 0.2, 1.0, -1.0)
                        self.wy_pid = PID(2.0, 0.0, 0.2, 1.0, -1.0)
                        self.wz_pid = PID(2.0, 0.0, 0.2, 1.0, -1.0)
                    elif self._device=='xsens':
                        # PID gains for XSens MTw Awinda
                        self.vx_pid = PID(1.0, 0.0, 0.1, 1.0, -1.0)
                        self.vy_pid = PID(1.0, 0.0, 0.1, 1.0, -1.0)
                        self.vz_pid = PID(1.0, 0.0, 0.1, 1.0, -1.0)
                        self.wx_pid = PID(1.0, 0.0, 0.1, 1.0, -1.0)
                        self.wy_pid = PID(1.0, 0.0, 0.1, 1.0, -1.0)
                        self.wz_pid = PID(1.0, 0.0, 0.1, 1.0, -1.0)
                    
                    # Joint PID control (values for real intera robot) -> Joint velocity controller
                    self.q0_pid = PID(2.0, 0.0, 0.1, 1.0, -1.0)
                    self.q1_pid = PID(2.0, 0.0, 0.1, 1.0, -1.0)
                    self.q2_pid = PID(2.0, 0.0, 0.1, 1.0, -1.0) # This joint is fixed, but has to be controlled
                    self.q3_pid = PID(2.0, 0.0, 0.1, 1.0, -1.0)
                    self.q4_pid = PID(2.0, 0.0, 0.1, 1.0, -1.0)
                    self.q5_pid = PID(2.0, 0.0, 0.1, 1.0, -1.0)
                    self.q6_pid = PID(2.0, 0.0, 0.1, 1.0, -1.0)
                    
                elif self._limb_name=='right':

                    # Cartesian PID control (values for real intera robot)
                    if self._device=='falcon':
#                         # PID gains for Falcon -> Ideal values without FF
#                         self.vx_pid = PID(1.0, 0.0, 0.05, 1.0, -1.0)
#                         self.vy_pid = PID(1.0, 0.0, 0.05, 1.0, -1.0)
#                         self.vz_pid = PID(1.0, 0.0, 0.05, 1.0, -1.0)
#                         self.wx_pid = PID(0.2, 0.0, 0.02, 1.0, -1.0)
#                         self.wy_pid = PID(0.2, 0.0, 0.02, 1.0, -1.0)
#                         self.wz_pid = PID(0.2, 0.0, 0.02, 1.0, -1.0)
                        # PID gains for Falcon -> Ideal values with FF
                        self.vx_pid = PID(2.0, 0.0, 0.2, 1.0, -1.0)
                        self.vy_pid = PID(2.0, 0.0, 0.2, 1.0, -1.0)
                        self.vz_pid = PID(2.0, 0.0, 0.2, 1.0, -1.0)
                        self.wx_pid = PID(1.0, 0.0, 0.05, 1.0, -1.0)
                        self.wy_pid = PID(1.0, 0.0, 0.05, 1.0, -1.0)
                        self.wz_pid = PID(1.0, 0.0, 0.05, 1.0, -1.0)
                    elif self._device=='phantom':
                        # PID gains for Phantom
                        self.vx_pid = PID(2.0, 0.0, 0.2, 1.0, -1.0)
                        self.vy_pid = PID(2.0, 0.0, 0.2, 1.0, -1.0)
                        self.vz_pid = PID(2.0, 0.0, 0.2, 1.0, -1.0)
                        self.wx_pid = PID(2.0, 0.0, 0.2, 1.0, -1.0)
                        self.wy_pid = PID(2.0, 0.0, 0.2, 1.0, -1.0)
                        self.wz_pid = PID(2.0, 0.0, 0.2, 1.0, -1.0)
                    elif self._device=='xsens':
                        # PID gains for XSens MTw Awinda
                        self.vx_pid = PID(2.0, 0.0, 0.2, 1.0, -1.0)
                        self.vy_pid = PID(2.0, 0.0, 0.2, 1.0, -1.0)
                        self.vz_pid = PID(2.0, 0.0, 0.2, 1.0, -1.0)
                        self.wx_pid = PID(2.0, 0.0, 0.2, 1.0, -1.0)
                        self.wy_pid = PID(2.0, 0.0, 0.2, 1.0, -1.0)
                        self.wz_pid = PID(2.0, 0.0, 0.2, 1.0, -1.0)
                    
                    # Joint PID control (values for real intera robot) -> Joint velocity controller
                    self.q0_pid = PID(2.0, 0.0, 0.1, 1.0, -1.0)
                    self.q1_pid = PID(2.0, 0.0, 0.1, 1.0, -1.0)
                    self.q2_pid = PID(2.0, 0.0, 0.1, 1.0, -1.0) # This joint is fixed, but has to be controlled
                    self.q3_pid = PID(2.0, 0.0, 0.1, 1.0, -1.0)
                    self.q4_pid = PID(2.0, 0.0, 0.1, 1.0, -1.0)
                    self.q5_pid = PID(2.0, 0.0, 0.1, 1.0, -1.0)
                    self.q6_pid = PID(2.0, 0.0, 0.1, 1.0, -1.0)
                
                else:
                 
                    # Cartesian PID control (values for real intera robot)
                    self.vx_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0)
                    self.vy_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0)
                    self.vz_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0)
                    self.wx_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0)
                    self.wy_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0)
                    self.wz_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0)
                    
                    # Joint PID control (values for real intera robot) -> Joint velocity controller
                    self.q0_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0)
                    self.q1_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0)
                    self.q2_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0) # This joint is fixed, but has to be controlled
                    self.q3_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0)
                    self.q4_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0)
                    self.q5_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0)
                    self.q6_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0)
                
            else:
             
                # Cartesian PID control (values for real intera robot)
                self.vx_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0)
                self.vy_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0)
                self.vz_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0)
                self.wx_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0)
                self.wy_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0)
                self.wz_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0)
                
                # Joint PID control (values for real intera robot) -> Joint velocity controller
                self.q0_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0)
                self.q1_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0)
                self.q2_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0) # This joint is fixed, but has to be controlled
                self.q3_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0)
                self.q4_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0)
                self.q5_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0)
                self.q6_pid = PID(0.0, 0.0, 0.0, 1.0, -1.0)
        
        # Set new_cmd_vel, new_cmd_pose, new_cmd_joint as false
        self.new_cmd_vel = False
        self.new_cmd_pose = False
        self.new_cmd_joint = False
        self.new_cmd_joint_phantom = False
        self.new_cmd_joint_falcon = False
        self.new_cmd_joint_xsens = False
        self.record_pose = True
        
        new_pose_msg = Pose()
        
        ns_grav = "/robot/limb/" + limb + "/gravity_compensation_torques"
        self._cbGravity = rospy.Subscriber(ns_grav, SEAJointState, self.callback_gravity)
        self._gravity = SEAJointState
        
        self._cmd = JointCommand()
        ns_cmd = "robot/limb/" + limb + "/joint_command"
        self._pub_cmd = rospy.Publisher(ns_cmd,
                                         JointCommand, queue_size=10)
        
        self._error = JointCommand()
        ns_error = "robot/limb/" + limb + "/joint_error"
        self._pub_error = rospy.Publisher(ns_error,
                                         JointCommand, queue_size=10)
        
        """
        'Velocity control' of one of the arms by commanding joint velocities.
        (based on Cartesian velocity and Jacobian???).
        """
        self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16, queue_size=10)
        # control parameters
        self._rate = 100.0  # Hz
        # set joint state publishing to 100Hz
        self._pub_rate.publish(self._rate)
        
        """
        'Trajectory control' of one of the arms by sending trajectories to
        the action server.
        """
        self._server = server
        if server:
            ns_traj = 'robot/limb/' + limb + '/'
            self._client = actionlib.SimpleActionClient(
                ns_traj + "follow_joint_trajectory",
                FollowJointTrajectoryAction,
            )
            self._goal = FollowJointTrajectoryGoal()
            self._goal_time_tolerance = rospy.Time(0.1)
            self._goal.goal_time_tolerance = self._goal_time_tolerance
            server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
            if not server_up:
                rospy.logerr("Timed out waiting for Joint Trajectory"
                             " Action Server to connect. Start the action server"
                             " before running example.")
                rospy.signal_shutdown("Timed out waiting for Action Server")
                sys.exit(1)
            self.clear(limb)
    
#     def ik_request(self, pose):
#         hdr = Header(stamp=rospy.Time.now(), frame_id='base')
#         ikreq = SolvePositionIKRequest()
#         ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
#         try:
#             resp = self._iksvc(ikreq)
#         except (rospy.ServiceException, rospy.ROSException), e:
#             rospy.logerr("Service call failed: %s" % (e,))
#             return False
#         # Check if result valid, and type of seed ultimately used to get solution
#         # convert rospy's string representation of uint8[]'s to int's
#         resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
#         limb_joints = {}
#         if (resp_seeds[0] != resp.RESULT_INVALID):
#             seed_str = {
#                         ikreq.SEED_USER: 'User Provided Seed',
#                         ikreq.SEED_CURRENT: 'Current Joint Angles',
#                         ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
#                        }.get(resp_seeds[0], 'None')
#             if self._verbose:
#                 print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
#                          (seed_str)))
#             # Format solution into Limb API-compatible dictionary
#             limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
#             if self._limb_name+'_measure_joint' in limb_joints:
#                 del limb_joints[self._limb_name+'_measure_joint']
#             if self._verbose:
#                 print("IK Joint Solution:\n{0}".format(limb_joints))
#                 print("------------------")
#         else:
#             rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
#             return False
#         return limb_joints

    def _guarded_move_to_joint_position(self, joint_angles, timeout=10.0):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles, timeout)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def move_to_neutral(self, timeout=10.0):
        print("Moving the {0} arm to neutral pose...".format(self._limb_name))
        self._limb.move_to_neutral(timeout)
        #self._arm.move_to_neutral() #Sets arm back into a neutral pose.
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def move_to_angles(self, start_angles=None, timeout=10.0):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles, timeout)
        print("Running. Ctrl-c to quit")

    def move_to_pose(self, pose, timeout=5.0):
        # servo down to release
        joint_angles = self._limb.ik_request(pose, self._tip_name)
        self._guarded_move_to_joint_position(joint_angles, timeout)

    def set_joint_position(self, joint_positions):
        if joint_positions:
            self._limb.set_joint_positions(joint_positions)
            #self._arm.set_joint_positions(joint_positions)
        else:
            rospy.logerr("No Joint Position provided for set_joint_positions. Staying put.")

    def set_joint_velocity(self, joint_velocities):
        if joint_velocities:
            self._limb.set_joint_velocities(joint_velocities)
            #self._arm.set_joint_velocities(joint_velocities)
        else:
            rospy.logerr("No Joint Velocity provided for set_joint_velocities. Staying put.")

    def set_joint_torque(self, joint_torques):
        if joint_torques:
            self._limb.set_joint_torques(joint_torques)
        else:
            rospy.logerr("No Joint Torque provided for set_joint_torques. Staying put.")

    def gripper_calibrate(self):
        self._gripper.calibrate()
        rospy.sleep(5.0)
        
    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose, hover_distance=0.1, timeout=2.0):
        approach = deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + hover_distance
        joint_angles = self._limb.ik_request(approach, self._tip_name)
        self._guarded_move_to_joint_position(joint_angles, timeout)

    def _retract(self, timeout=2.0):
        # retrieve current pose from endpoint
        pose = self._limb.endpoint_pose()
        pose_msg = self.get_pose_msg(pose)
        joint_angles = self.get_angles(pose_msg)
        # servo up from current pose
        self._guarded_move_to_joint_position(joint_angles, timeout)

    def get_pose_msg(self, pose):
        # convert to pose msg
        pose_msg = Pose()
        pose_msg.position.x = pose['position'].x
        pose_msg.position.y = pose['position'].y
        pose_msg.position.z = pose['position'].z
        pose_msg.orientation.x = pose['orientation'].x
        pose_msg.orientation.y = pose['orientation'].y
        pose_msg.orientation.z = pose['orientation'].z
        pose_msg.orientation.w = pose['orientation'].w
        return pose_msg

    def get_angles(self, pose, hover_distance=0.0):
        # get joint angles from pose        
        pose_hover = deepcopy(pose)
        pose_hover.position.z = pose_hover.position.z + hover_distance # Get angles for the position + hover distance
        joint_angles = self._limb.ik_request(pose_hover, self._tip_name)
#         position = [pose.position.x, pose.position.y, pose.position.z]
#         orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w] # Info: http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html#rotation
#         joint_angles = self._kin.inverse_kinematics(position, orientation) # More info: https://pypi.org/project/PyKDL/
        return joint_angles

    def get_angles_array(self, pose, hover_distance=0.0):
        # get joint angles from pose        
        pose.position.z = pose.position.z + hover_distance # Get angles for the position + hover distance
        joint_angles = self._limb.ik_request(pose, self._tip_name)
        #joint_angles = self.get_angles(pose, hover_distance)
        
        if joint_angles:
            joint_angles_array = joint_angles.values()
            joint_angles_array[0] = joint_angles[self._limb_name+'_j0']
            joint_angles_array[1] = joint_angles[self._limb_name+'_j1']
            joint_angles_array[2] = joint_angles[self._limb_name+'_j2']
            joint_angles_array[3] = joint_angles[self._limb_name+'_j3']
            joint_angles_array[4] = joint_angles[self._limb_name+'_j4']
            joint_angles_array[5] = joint_angles[self._limb_name+'_j5']
            joint_angles_array[6] = joint_angles[self._limb_name+'_j6']
#             position = [pose.position.x, pose.position.y, pose.position.z]
#             orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w] # Info: http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html#rotation
#             joint_angles = self._kin.inverse_kinematics(position, orientation) # More info: https://pypi.org/project/PyKDL/
            return joint_angles_array
        else:
            return joint_angles
        
    def pick(self, pose):
        if rospy.is_shutdown():
            return
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose, timeout=5.0)
        # servo to pose
        self.move_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._retract()

    def place(self, pose):
        if rospy.is_shutdown():
            return
        # servo above pose
        self._approach(pose, timeout=5.0)
        # servo to pose
        self.move_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()

    def generate_trajectory(self, initial, final, pts=10):
        trajectory = []
        for i in range(0, pts):
            new_pose = Pose()
            new_pose.position.x = initial.position.x+(final.position.x-initial.position.x)*i/pts
            new_pose.position.y = initial.position.y+(final.position.y-initial.position.y)*i/pts
            new_pose.position.z = initial.position.z+(final.position.z-initial.position.z)*i/pts
            new_pose.orientation = initial.orientation
            trajectory.append(new_pose)
            #polish_poses.append(Pose(
            #position=Point(x, y, z),
            #orientation=overhead_orientation))
        return trajectory
    
    def angle_diff(self, angle1, angle2):
        
         # Rotate angle1 with angle2 so that the sought after
         # angle is between the resulting angle and the x-axis
         angle = angle1 - angle2

         # "Normalize" angle to range [-180,180[
         if angle < -math.pi:
             angle = angle+math.pi*2
         elif angle > math.pi:
             angle = angle-math.pi*2
        
         return angle

####################################################################################

    def callback_joy(self, data):
        self._joy = data
        #print(self._joy)
        
    def callback_cmd_joint(self, data):
        self._cmd_joint = data
        #print(self._cmd_joint)
        # Set angular velocities
        self.q0 = self._cmd_joint.velocity[0] # s0
        self.q1 = self._cmd_joint.velocity[1] # s1
        self.q2 = self._cmd_joint.velocity[2] # e0
        self.q3 = self._cmd_joint.velocity[3] # e1
        self.q4 = self._cmd_joint.velocity[4] # w0
        self.q5 = self._cmd_joint.velocity[5] # w1
        self.q6 = self._cmd_joint.velocity[6] # w2
        # Reset cartesian velocities
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.wx = 0.0
        self.wy = 0.0
        self.wz = 0.0
        # Reset cartesian pose
        self.dx = 0.0
        self.dy = 0.0
        self.dz = 0.0
        self.dR = 0.0
        self.dP = 0.0
        self.dY = 0.0
        # Set new_cmd_joint as true
        #if (self.q0!=0.0 or self.q1!=0.0 or self.q2!=0.0 or self.q3!=0.0 or self.q4!=0.0 or self.q5!=0.0 or self.q6!=0.0):
        if (math.fabs(self.q0)>0.01 or math.fabs(self.q1)>0.01 or math.fabs(self.q2)>0.01 or math.fabs(self.q3)>0.01 or math.fabs(self.q4)>0.01 or math.fabs(self.q5)>0.01 or math.fabs(self.q6)>0.01):
            self.new_cmd_joint = True
            self.record_pose = True
        
    def callback_cmd_joint_phantom(self, data):
        self._cmd_joint_phantom = data
        #print(self._cmd_joint_phantom)
        # Set angular position        
        self.q0 = self._cmd_joint_phantom.position[0] # s0
        self.q1 = self._cmd_joint_phantom.position[1] # s1
        self.q2 = self._cmd_joint_phantom.position[2] # e0
        self.q3 = self._cmd_joint_phantom.position[3] # e1
        self.q4 = self._cmd_joint_phantom.position[4] # w0
        self.q5 = self._cmd_joint_phantom.position[5] # w1
        self.q6 = self._cmd_joint_phantom.position[6] # w2
        # Reset cartesian velocities
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.wx = 0.0
        self.wy = 0.0
        self.wz = 0.0
        # Reset cartesian pose
        self.dx = 0.0
        self.dy = 0.0
        self.dz = 0.0
        self.dR = 0.0
        self.dP = 0.0
        self.dY = 0.0
        # Set new_cmd_joint as true
        #if (self.q0!=0.0 or self.q1!=0.0 or self.q2!=0.0 or self.q3!=0.0 or self.q4!=0.0 or self.q5!=0.0 or self.q6!=0.0):
        if (math.fabs(self.q0)>0.01 or math.fabs(self.q1)>0.01 or math.fabs(self.q2)>0.01 or math.fabs(self.q3)>0.01 or math.fabs(self.q4)>0.01 or math.fabs(self.q5)>0.01 or math.fabs(self.q6)>0.01):
            self.new_cmd_joint_phantom = True
            self.record_pose = True
        
    def callback_cmd_joint_falcon(self, data):
        self._cmd_joint_falcon = data
        #print(self._cmd_joint_falcon)
        # Set angular position      
        self.q0 = self._cmd_joint_falcon.position[0] # s0
        self.q1 = self._cmd_joint_falcon.position[1] # s1
        self.q2 = self._cmd_joint_falcon.position[2] # e0
        self.q3 = self._cmd_joint_falcon.position[3] # e1
        self.q4 = self._cmd_joint_falcon.position[4] # w0
        self.q5 = self._cmd_joint_falcon.position[5] # w1
        self.q6 = self._cmd_joint_falcon.position[6] # w2
        # Reset cartesian velocities
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.wx = 0.0
        self.wy = 0.0
        self.wz = 0.0
        # Reset cartesian pose
        self.dx = 0.0
        self.dy = 0.0
        self.dz = 0.0
        self.dR = 0.0
        self.dP = 0.0
        self.dY = 0.0
        # Set new_cmd_joint as true
        #if (self.q0!=0.0 or self.q1!=0.0 or self.q2!=0.0 or self.q3!=0.0 or self.q4!=0.0 or self.q5!=0.0 or self.q6!=0.0):
        if (math.fabs(self.q0)>0.01 or math.fabs(self.q1)>0.01 or math.fabs(self.q2)>0.01 or math.fabs(self.q3)>0.01 or math.fabs(self.q4)>0.01 or math.fabs(self.q5)>0.01 or math.fabs(self.q6)>0.01):
            self.new_cmd_joint_falcon = True
            self.record_pose = True

    def callback_cmd_joint_xsens(self, data):
        self._cmd_joint_xsens = data
        #print(self._cmd_joint_xsens)
        # Set angular position
        self.q0 = self._cmd_joint_xsens.position[0] # s0
        self.q1 = self._cmd_joint_xsens.position[1] # s1
        self.q2 = self._cmd_joint_xsens.position[2] # e0
        self.q3 = self._cmd_joint_xsens.position[3] # e1
        self.q4 = self._cmd_joint_xsens.position[4] # w0
        self.q5 = self._cmd_joint_xsens.position[5] # w1
        self.q6 = self._cmd_joint_xsens.position[6] # w2
        # Reset cartesian velocities
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.wx = 0.0
        self.wy = 0.0
        self.wz = 0.0
        # Reset cartesian pose
        self.dx = 0.0
        self.dy = 0.0
        self.dz = 0.0
        self.dR = 0.0
        self.dP = 0.0
        self.dY = 0.0
        # Set new_cmd_joint as true
        #if (self.q0!=0.0 or self.q1!=0.0 or self.q2!=0.0 or self.q3!=0.0 or self.q4!=0.0 or self.q5!=0.0 or self.q6!=0.0):
        if (math.fabs(self.q0)>0.01 or math.fabs(self.q1)>0.01 or math.fabs(self.q2)>0.01 or math.fabs(self.q3)>0.01 or math.fabs(self.q4)>0.01 or math.fabs(self.q5)>0.01 or math.fabs(self.q6)>0.01):
            self.new_cmd_joint_xsens = True
            self.record_pose = True
        
    def callback_cmd_vel(self, data):
        self._cmd_vel = data
        #print(self._cmd_vel)
        # Set cartesian velocities
        self.vx = self._cmd_vel.twist.linear.x
        self.vy = self._cmd_vel.twist.linear.y
        self.vz = self._cmd_vel.twist.linear.z
        self.wx = self._cmd_vel.twist.angular.x
        self.wy = self._cmd_vel.twist.angular.y
        self.wz = self._cmd_vel.twist.angular.z
        
#         self.local_rotation = quaternion_from_euler(self.wx, self.wy, self.wz)
#         current_pose_msg = self.get_pose_msg(self._limb.endpoint_pose()) 
#         self.current_orientation = [current_pose_msg.orientation.w,
#                                     current_pose_msg.orientation.x, 
#                                     current_pose_msg.orientation.y, 
#                                     current_pose_msg.orientation.z]
#         self.global_angular_velocity = self.current_orientation * self.local_rotation
#         
#         #self.wx = self.global_angular_velocity[0]
#         #self.wy = self.global_angular_velocity[1]
#         #self.wz = self.global_angular_velocity[2]
        
        # Reset angular velocities
        self.q0 = 0.0 # s0
        self.q1 = 0.0 # s1
        self.q2 = 0.0 # e0
        self.q3 = 0.0 # e1
        self.q4 = 0.0 # w0
        self.q5 = 0.0 # w1
        self.q6 = 0.0 # w2
        # Reset cartesian pose
        self.dx = 0.0
        self.dy = 0.0
        self.dz = 0.0
        self.dR = 0.0
        self.dP = 0.0
        self.dY = 0.0
        # Set new_cmd_vel as true
        #if (self.vx!=0.0 or self.vy!=0.0 or self.vz!=0.0 or self.wx!=0.0 or self.wy!=0.0 or self.wz!=0.0):
        if (math.fabs(self.vx)>0.01 or math.fabs(self.vy)>0.01 or math.fabs(self.vz)>0.01 or math.fabs(self.wx)>0.01 or math.fabs(self.wy)>0.01 or math.fabs(self.wz)>0.01):
            self.new_cmd_vel = True
            self.record_pose = True
        
    def callback_cmd_pose(self, data):
        self._cmd_pose = data
        #print(self._cmd_pose)
        
        if(self.record_pose == True):
   
#             # The Pose of the movement in its initial location.
#             current_pose_msg = self.get_pose_msg(self._limb.endpoint_pose())
#             
#             self.initial_position = [current_pose_msg.position.x,
#                                      current_pose_msg.position.y,
#                                      current_pose_msg.position.z]
             
            # Set untuck position as initial offset
            if(self._limb_name=='right'):
                self.initial_position = [0.544, -0.216, 0.252]
            elif(self._limb_name=='left'):
                self.initial_position = [0.544,  0.216, 0.252]
 
#             self.initial_quaternion = [current_pose_msg.orientation.w, 
#                                        current_pose_msg.orientation.x, 
#                                        current_pose_msg.orientation.y, 
#                                        current_pose_msg.orientation.z]
#             
#             euler = euler_from_quaternion(self.initial_quaternion)
#             print euler
             
            # An orientation for gripper fingers to be overhead and parallel to the obj
            roll = 0
            pitch = 0
            yaw = math.pi
            self.initial_quaternion = quaternion_from_euler(roll, pitch, yaw)
 
            self.initial_euler = euler_from_quaternion(self.initial_quaternion)
             
            self.initial_joints = self._limb.joint_angles()
             
            self.record_pose = False

 
 ################################################################
#         # Set cartesian pose (position + orientation)
#         self.dx = self._cmd_pose.twist.linear.x
#         self.dy = self._cmd_pose.twist.linear.y
#         self.dz = self._cmd_pose.twist.linear.z
#         self.dR = self._cmd_pose.twist.angular.x
#         self.dP = self._cmd_pose.twist.angular.y
#         self.dY = self._cmd_pose.twist.angular.z
 ################################################################
        
        
 ################################################################
 
        self.initial_position = [0, 0, 0]
        self.initial_euler = [0, 0, 0]
 
        # Set cartesian pose (position + orientation)
        self.dx = self._cmd_pose.pose.position.x
        self.dy = self._cmd_pose.pose.position.y
        self.dz = self._cmd_pose.pose.position.z
        
        if((self._cmd_pose.pose.orientation.w==0.0) and (self._cmd_pose.pose.orientation.w==0.0) and
        (self._cmd_pose.pose.orientation.w==0.0) and (self._cmd_pose.pose.orientation.w==0.0)):
        
            self.dR = 0.0
            self.dP = 0.0
            self.dY = 0.0
            
        else:
            delta_quaternion = [self._cmd_pose.pose.orientation.w, 
                                self._cmd_pose.pose.orientation.x, 
                                self._cmd_pose.pose.orientation.y, 
                                self._cmd_pose.pose.orientation.z]
                        
            delta_euler = euler_from_quaternion(delta_quaternion)
             
            self.dR = delta_euler[2]
            self.dP = delta_euler[1]
            self.dY = delta_euler[0]

 ################################################################    
        
        self.new_x = self.initial_position[0] + self.dx
        self.new_y = self.initial_position[1] + self.dy
        self.new_z = self.initial_position[2] + self.dz
        new_point = Point(x=self.new_x, y=self.new_y, z=self.new_z)
      
        self.new_Y = self.initial_euler[0] + self.dY
        self.new_P = self.initial_euler[1] + self.dP
        self.new_R = self.initial_euler[2] + self.dR
        new_quaterion = quaternion_from_euler(self.new_R, self.new_P, self.new_Y)
        
        new_orientation = Quaternion(x=new_quaterion[1],
                                     y=new_quaterion[2],
                                     z=new_quaterion[3],
                                     w=new_quaterion[0])
        #new_orientation = self._cmd_pose.pose.orientation
 
        self.new_pose_msg = Pose(position=new_point,
                                 orientation=new_orientation)
#         self.new_pose_msg = self._pose_stmp.pose

        ################################################

        # Reset cartesian velocities
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.wx = 0.0
        self.wy = 0.0
        self.wz = 0.0
        # Reset angular velocities
        self.q0 = 0.0 # s0
        self.q1 = 0.0 # s1
        self.q2 = 0.0 # e0
        self.q3 = 0.0 # e1
        self.q4 = 0.0 # w0
        self.q5 = 0.0 # w1
        self.q6 = 0.0 # w2
        # Set new_cmd_vel as true
        #if (self.dx!=0.0 or self.dy!=0.0 or self.dz!=0.0 or self.dR!=0.0 or self.dP!=0.0 or self.dY!=0.0):
        if (math.fabs(self.dx)>0.01 or math.fabs(self.dy)>0.01 or math.fabs(self.dz)>0.01 or math.fabs(self.dR)>0.01 or math.fabs(self.dP)>0.01 or math.fabs(self.dY)>0.01):
            self.new_cmd_pose = True
        
    def callback_cmd_gripper(self, data):
        self._cmd_gripper = data
        if self._cmd_gripper.seq == 0:
            self.gripper_open()
        else:
            self.gripper_close()
        
    def callback_gravity(self, data):
        self._gravity = data
        global efforts
        #efforts = np.negative(self._gravity.gravity_model_effort)
        efforts = self._gravity.gravity_model_effort
        
    def callback_state(self, data):
        # The Pose of the movement in its current location.
        current_pose = self.get_pose_msg(self._limb.endpoint_pose())
        # Distance between both grippers
        dx = data.pose.position.x-current_pose.position.x
        dy = data.pose.position.y-current_pose.position.y
        dz = data.pose.position.z-current_pose.position.z
        dist = math.sqrt(dx*dx + dy*dy +dz*dz)
        # Disable collisiotn avoidance when grippers are closer than certain value
        if(dist < 0.3):
            collision = Empty()
            self._pub_collision.publish(collision)


####################################################################################

    def loop(self, fc = 100.0):
        
#         # Joint angles for right/left arm (initial position to avoid table)
#         joint_angles = {self._limb_name+'_s0': -0.5 * self._limb_sign,
#                         self._limb_name+'_s1': -1.0,
#                         self._limb_name+'_e0': 0.0 * self._limb_sign,
#                         self._limb_name+'_e1': 2.0,
#                         self._limb_name+'_w0': 0.0 * self._limb_sign,
#                         self._limb_name+'_w1': 1.0,
#                         self._limb_name+'_w2': 0.0 * self._limb_sign}
#         self.move_to_angles(joint_angles, timeout=3.0)
        
        # Joint angles for right/left arm (tuck position)
        joint_angles = {self._limb_name+'_j0': 0 * self._limb_sign,
                        self._limb_name+'_j1': -1.0,
                        self._limb_name+'_j2': 1.2 * self._limb_sign,
                        self._limb_name+'_j3': 2.0,
                        self._limb_name+'_j4': -0.6 * self._limb_sign,
                        self._limb_name+'_j5': 1.0,
                        self._limb_name+'_j6': 0.5 * self._limb_sign}
        self.move_to_angles(joint_angles, timeout=3.0)
         
#         # Joint angles for right/left arm (zero in all joints)
# #         joint_angles = {self._limb_name+'_s0': 0 * self._limb_sign,
#         joint_angles = {self._limb_name+'_s0': math.pi/4.0 * self._limb_sign,
#                         self._limb_name+'_s1': 0,
#                         self._limb_name+'_e0': 0 * self._limb_sign,
#                         self._limb_name+'_e1': 0,
#                         self._limb_name+'_w0': 0 * self._limb_sign,
#                         self._limb_name+'_w1': 0,
#                         self._limb_name+'_w2': 0 * self._limb_sign}
#         self.move_to_angles(joint_angles, timeout=3.0)
        
        # Joint angles for right/left arm
        if self._device=="phantom":
            joint_angles = {self._limb_name+'_j0': 0.0 * self._limb_sign,
                            self._limb_name+'_j1': -0.7,
                            self._limb_name+'_j2': 0.0 * self._limb_sign,
                            self._limb_name+'_j3': 1.8,
                            self._limb_name+'_j4': -0.0 * self._limb_sign,
                            self._limb_name+'_j5': 0.5,
                            self._limb_name+'_j6': 0.0 * self._limb_sign}
#             joint_angles = {self._limb_name+'_j0': 0.0,
#                             self._limb_name+'_j1': 0.0,
#                             self._limb_name+'_j2': 0.0,
#                             self._limb_name+'_j3': 0.0,
#                             self._limb_name+'_j4': 0.0,
#                             self._limb_name+'_j5': 0.0,
#                             self._limb_name+'_j6': 0.0}
            self.move_to_angles(joint_angles, timeout=5.0)

        elif self._device=="falcon":
            if True:
                start_pose_msg = Pose(position=Point(x=0.5, y=-self._limb_sign*0.6, z=0.0),
                                      orientation=Quaternion(x=0, y=1, z=0, w=0))
            else:
                start_pose_msg = Pose(position=Point(x=0.6, y=-self._limb_sign*0.0, z=0.2),
                                      orientation=Quaternion(x=0.1158, y=-0.3886, z=0.8370, w=0.3675))
            self.move_to_pose(start_pose_msg, timeout=5.0)

        elif self._device=="xsens":
     
            # Neutral position
            #self.move_to_neutral()

            # Forward position
            #start_pose_msg = Pose(position=Point(x=0.8, y=-self._limb_sign*0.2, z=0.3),
            #                      orientation=Quaternion(x=0.0, y=0.7071, z=self._limb_sign*0.7071, w=0.0))

            # L-shape position
            start_pose_msg = Pose(position=Point(x=0.5, y=-self._limb_sign*0.3, z=0.0),
                                  orientation=Quaternion(x=0.0, y=0.7071, z=self._limb_sign*0.7071, w=0.0))

            self.move_to_pose(start_pose_msg, timeout=5.0)

        tc = 1.0/fc
        rate = rospy.Rate(fc)
        #rospy.spin()
        
        joint_positions = dict()
        joint_velocities = dict()
        joint_errors = dict()
        
        # Enable intera again??
        #intera_interface.RobotEnable(CHECK_VERSION)
        
#         current_pose = self._pose # Save initial pose
#         current_position = current_pose['position']
#         current_orientation = current_pose['orientation']
#             
#         # The Pose of the movement in its initial location.
#         current_pose_msg = Pose()
#         current_pose_msg = self.get_pose_msg(current_pose)
        
        # Save current joint angles
#         u_joints = self.get_angles_array(current_pose_msg, hover_distance=0.0) # This is very noisy because of IK
#         print u_joints
#         u_joints = self._limb.joint_angles().values()
#         print u_joints
        joint_positions = self._limb.joint_angles()
        
        while not rospy.is_shutdown():
            
#             if self.new_cmd_vel:
#                   
#                 self.new_cmd_vel = False
#                   
#                 new_x = current_pose_msg.position.x + self.vx*tc
#                 new_y = current_pose_msg.position.y + self.vy*tc
#                 new_z = current_pose_msg.position.z + self.vz*tc
#                 new_point = Point(x=new_x, y=new_y, z=new_z)
#                   
#                 current_quaternion = [current_pose_msg.orientation.w, 
#                                       current_pose_msg.orientation.x, 
#                                       current_pose_msg.orientation.y, 
#                                       current_pose_msg.orientation.z]
#                   
#                 current_euler = euler_from_quaternion(current_quaternion)
#       
#                 new_Y = current_euler[0] + self.wz*tc
#                 new_P = current_euler[1] + self.wy*tc
#                 new_R = current_euler[2] + self.wx*tc
#                   
#                 new_quaterion = quaternion_from_euler(new_R, new_P, new_Y)
#       
#                 new_orientation = Quaternion(x=-new_quaterion[1],
#                                              y=-new_quaterion[2],
#                                              z=-new_quaterion[3],
#                                              w=-new_quaterion[0])
#                   
#                 new_pose_msg = Pose(position=new_point,
#                                     orientation=new_orientation)
#                   
#                 #self.move_to_pose(new_pose_msg, timeout=tc)
#                 #rospy.sleep(tc)
#       
#                 # servo down to release
#                 #joint_angles = self._limb.ik_request(new_pose_msg, self._tip_name)
#                 #self._guarded_move_to_joint_position(joint_angles, timeout=tc)
#                   
#                   
#                 #print self.vx*tc, " ", self.vy*tc, " ", self.vz*tc
#                 #print current_pose_msg
#                 #print new_pose_msg
#                   
#                 #if(new_pose_msg.position!=current_pose_msg.position or new_pose_msg.orientation!=current_pose_msg.orientation):
#                 u_joints = self.get_angles_array(new_pose_msg, hover_distance=0.0)
#                 if(u_joints):
#                     joint_positions = dict(zip(self._joint_names, u_joints))
#                     #self.set_joint_position(joint_positions)
#                     self._limb.move_to_joint_positions(joint_positions, timeout=tc)
#                     # Save new pose message for new iteration
#                     current_pose_msg = new_pose_msg

##################################################

            # Joint angles/velocities depending on the controller used
            #u_joints = np.array([[self.q0], [self.q1], [self.q2], [self.q3], [self.q4], [self.q5], [self.q6]]) # Define a column vector?
            u_joints = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]]) # Define a column vector?
            #u_joints = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # Define a column vector?
              
            if self.new_cmd_vel:
                   
                self.new_cmd_vel = False
                # Cartesian velocities (linear and angular)
                u_cartesian = np.array([[self.vx], [self.vy], [self.vz], [self.wx], [self.wy], [self.wz]]) # Define a column vector
                # Computation of joint velocities from cartesian space using Jacobian matrix
                jacobianInv = self._kin.jacobian_pseudo_inverse()
                #jacobianInv = np.linalg.pinv(self._kin.jacobian(), rcond=1e-3) # big rcond to avoid singularities (default rcond=1e-15)
                u_joints = jacobianInv * u_cartesian
                
            elif self.new_cmd_joint:
                   
                self.new_cmd_joint = False
                # Joint angles/velocities depending on the controller used
                u_joints = np.array([[self.q0], [self.q1], [self.q2], [self.q3], [self.q4], [self.q5], [self.q6]]) # Define a column vector
                
            elif self.new_cmd_joint_phantom or self.new_cmd_joint_falcon or self.new_cmd_joint_xsens:
                   
                self.new_cmd_joint_phantom = False
                self.new_cmd_joint_falcon = False
                self.new_cmd_joint_xsens = False
                q_joints = self._limb.joint_angles()#.values()
                self.e0 = self.q0 - q_joints[self._limb_name+'_j0']
                self.e1 = self.q1 - q_joints[self._limb_name+'_j1']
                self.e2 = self.q2 - q_joints[self._limb_name+'_j2']
                self.e3 = self.q3 - q_joints[self._limb_name+'_j3']
                self.e4 = self.q4 - q_joints[self._limb_name+'_j4']
                self.e5 = self.q5 - q_joints[self._limb_name+'_j5']
                self.e6 = self.q6 - q_joints[self._limb_name+'_j6']

                self.u0 = -self.q0_pid.update_PID(self.e0)
                self.u1 = -self.q1_pid.update_PID(self.e1)
                self.u2 = -self.q2_pid.update_PID(self.e2)
                self.u3 = -self.q3_pid.update_PID(self.e3)
                self.u4 = -self.q4_pid.update_PID(self.e4)
                self.u5 = -self.q5_pid.update_PID(self.e5)
                self.u6 = -self.q6_pid.update_PID(self.e6)

                if self._sim==True:
                    # Add a dead zone around zero. These control actions are angular speed
                    if np.abs(self.u0) < 0.1:
                        self.u0 = 0.0
                    if np.abs(self.u1) < 0.1:
                        self.u1 = 0.0
                    if np.abs(self.u2) < 0.1:
                        self.u2 = 0.0
                    if np.abs(self.u3) < 0.1:
                        self.u3 = 0.0
                    if np.abs(self.u4) < 0.1:
                        self.u4 = 0.0
                    if np.abs(self.u5) < 0.1:
                        self.u5 = 0.0
                    if np.abs(self.u6) < 0.1:
                        self.u6 = 0.0
                else:
                    if self._control=="position":
                        # Add a dead zone around zero. These control actions are angular speed
                        if np.abs(self.u0) < 0.8:
                            self.u0 = 0.0
                        if np.abs(self.u1) < 0.8:
                            self.u1 = 0.0
                        if np.abs(self.u2) < 0.6:
                            self.u2 = 0.0
                        if np.abs(self.u3) < 0.6:
                            self.u3 = 0.0
                        if np.abs(self.u4) < 0.6:
                            self.u4 = 0.0
                        if np.abs(self.u5) < 0.6:
                            self.u5 = 0.0
                        if np.abs(self.u6) < 0.2:
                            self.u6 = 0.0
                
                # Joint angles/velocities depending on the controller used
                u_joints = np.array([[self.u0], [self.u1], [self.u2], [self.u3], [self.u4], [self.u5], [self.u6]]) # Define a column vector
                
            elif self.new_cmd_pose:
                   
                self.new_cmd_pose = False
               
#                 #self.move_to_pose(self.new_pose_msg, timeout=tc)
# 
#                 #if(new_pose_msg.position!=current_pose_msg.position or new_pose_msg.orientation!=current_pose_msg.orientation):
#                 u_joints = self.get_angles_array(self.new_pose_msg, hover_distance=0.0)
#                 if(u_joints):
#                     joint_positions = dict(zip(self._joint_names, u_joints))
#                     #self.set_joint_position(joint_positions)
#                     self._limb.move_to_joint_positions(joint_positions, timeout=tc)
#                     # Save new pose message for new iteration
#                     #current_pose_msg = new_pose_msg

                current_pose_msg = self.get_pose_msg(self._limb.endpoint_pose())
 
                current_quaternion = [current_pose_msg.orientation.w, 
                                      current_pose_msg.orientation.x, 
                                      current_pose_msg.orientation.y, 
                                      current_pose_msg.orientation.z]
                    
                current_euler = euler_from_quaternion(current_quaternion)

                self.ex = self.new_x - current_pose_msg.position.x
                self.ey = self.new_y - current_pose_msg.position.y
                self.ez = self.new_z - current_pose_msg.position.z
                #self.eR = self.new_R - current_euler[2]
                #self.eP = self.new_P - current_euler[1]
                #self.eY = self.new_Y - current_euler[0]
                self.eR = self.angle_diff(self.new_R, current_euler[2])
                self.eP = self.angle_diff(self.new_P, current_euler[1])
                self.eY = self.angle_diff(self.new_Y, current_euler[0])
                
                self.ux = -self.vx_pid.update_PID(self.ex)
                self.uy = -self.vy_pid.update_PID(self.ey)
                self.uz = -self.vz_pid.update_PID(self.ez)
                self.uR = self.wx_pid.update_PID(self.eR)
                self.uP = -self.wy_pid.update_PID(self.eP)
                self.uY = self.wz_pid.update_PID(self.eY)
                
                # Cancel out linear/angular velocity control depending on the control method
                if((self.dx==0.0) and (self.dy==0.0) and (self.dz==0.0)):
                    self.ux = 0.0
                    self.uy = 0.0
                    self.uz = 0.0

                if((self.dR==0.0) and (self.dP==0.0) and (self.dY==0.0)):
                    self.uR = 0.0
                    self.uP = 0.0
                    self.uY = 0.0
                
                # Add a dead zone around zero. These control actions are linear and angular velocity
                if np.abs(self.ux) < 0.01:
                    self.ux = 0.0
                if np.abs(self.uy) < 0.01:
                    self.uy = 0.0
                if np.abs(self.uz) < 0.01:
                    self.uz = 0.0
                if np.abs(self.uR) < 0.01:
                    self.uR = 0.0
                if np.abs(self.uP) < 0.01:
                    self.uP = 0.0
                if np.abs(self.uY) < 0.01:
                    self.uY = 0.0

                # Cartesian velocities (linear and angular)
                u_cartesian = np.array([[self.ux], [self.uy], [self.uz], [self.uR], [self.uP], [self.uY]]) # Define a column vector
                # Computation of joint velocities from cartesian space using Jacobian matrix
                #jacobianInv = self._kin.jacobian_pseudo_inverse()
                jacobianInv = np.linalg.pinv(self._kin.jacobian(), rcond=1e-15) # big rcond to avoid singularities (default rcond=1e-15)
                u_joints = jacobianInv * u_cartesian

            else:
                   
                self.new_cmd_vel = False
                self.new_cmd_joint = False
                self.new_cmd_joint_phantom = False
                self.new_cmd_joint_falcon = False
                self.new_cmd_joint_xsens = False
                self.new_cmd_pose = False
                #self.record_pose = True
                # Joint angles/velocities depending on the controller used
                u_joints = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]]) # Define a column vector

##################################################
            
            if self._control=='position':

#                 # Update joint_positions only if there's a big difference between actual and reference joint positions (to avoid stuck configurations)
#                 joint_positions_current = self._limb.joint_angles()
#                 error_norm = np.linalg.norm(np.array(joint_positions_current.values()).transpose()-np.array(joint_positions.values()).transpose())
#                 if(error_norm>0.05):
#                     joint_positions = joint_positions_current
                    
                #u_joints = self.get_angles_array(current_pose_msg, hover_distance=0.0) # This is very noisy because of IK
                u_joints[0] = joint_positions[self._limb_name+'_j0'] + u_joints[0]*tc
                u_joints[1] = joint_positions[self._limb_name+'_j1'] + u_joints[1]*tc
                u_joints[2] = joint_positions[self._limb_name+'_j2'] + u_joints[2]*tc
                u_joints[3] = joint_positions[self._limb_name+'_j3'] + u_joints[3]*tc
                u_joints[4] = joint_positions[self._limb_name+'_j4'] + u_joints[4]*tc
                u_joints[5] = joint_positions[self._limb_name+'_j5'] + u_joints[5]*tc
                u_joints[6] = joint_positions[self._limb_name+'_j6'] + u_joints[6]*tc
      
                ## POSITION CONTROL
                #joint_positions = dict(zip(self._limb._joint_names[self._limb_name], u_joints))
                joint_positions = dict(zip(self._joint_names, u_joints)) #u_joints = self.get_angles_array(current_pose_msg, hover_distance=0.0) # This is very noisy because of IK
        
                ## APPLY POSITION CONTROL ACTION
                #self.move_to_angles(start_angles=joint_positions, timeout=tc) # Not good inside a control loop
                self.set_joint_position(joint_positions)

            elif self._control=='velocity':
    
                ## VELOCITY CONTROL
                #joint_velocities = dict(zip(self._limb._joint_names[self._limb_name], u_joints))
                joint_velocities = dict(zip(self._joint_names, u_joints))
                   
                ## APPLY VELOCITY CONTROL ACTION
                self.set_joint_velocity(joint_velocities)

            elif self._control=='effort':

                ## EFFORT CONTROL
                #joint_efforts = dict(zip(self._limb._joint_names[self._limb_name], u_joints))
                joint_efforts = dict(zip(self._joint_names, efforts))
      
                ## APPLY EFFORT CONTROL ACTION
                self.set_joint_torque(joint_efforts)
                
            else:
                
                print("Control method not recognized!")

##################################################

#             # Control of intera robot by publishing a command with values and type of control
#             # This is equivalent to set_joint_position(joint_positions) and set_joint_velocity(joint_velocities)
#             if self._control=='position':
#                 ## POSITION CONTROL
#                 self._cmd.mode = self._cmd.POSITION_MODE
#                 self._cmd.names = joint_positions.keys()
#                 self._cmd.position = joint_positions.values()       
# 
#             elif self._control=='velocity':
#                 ## VELOCITY CONTROL
#                 self._cmd.mode = self._cmd.VELOCITY_MODE
#                 self._cmd.names = joint_velocities.keys()
#                 self._cmd.velocity = joint_velocities.values()
# 
#             elif self._control=='effort':
#                 ## EFFOR CONTROL
#                 self._cmd.mode = self._cmd.EFFORT_MODE
#                 self._cmd.names = joint_efforts.keys()
#                 self._cmd.effort = joint_efforts.values()
# 
#             # PUBLISH CMD CONTROL
#             self._cmd.header.stamp = rospy.Time.now()
#             self._pub_cmd.publish(self._cmd)

##################################################

            # Control of intera robot by publishing a command with values and type of control
            # This is equivalent to set_joint_position(joint_positions) and set_joint_velocity(joint_velocities)
            if self._control=='position':
                ## POSITION CONTROL
                self._error.mode = self._error.POSITION_MODE
                self._error.names = joint_positions.keys()
                #self._error.position = joint_positions.values() - self._limb.joint_angles().values()
                self._error.position = [ ( joint_positions[k][0] - self._limb.joint_angles()[k] ) for k in self._limb.joint_angles().keys() ]
 
            elif self._control=='velocity':
                ## VELOCITY CONTROL
                self._error.mode = self._error.VELOCITY_MODE
                self._error.names = joint_velocities.keys()
                #self._error.velocity = joint_velocities.values() - self._limb.joint_velocities()
                self._error.velocity = [ ( joint_velocities[k][0] - self._limb.joint_velocities()[k] ) for k in self._limb.joint_velocities().keys() ]
 
            elif self._control=='effort':
                ## EFFOR CONTROL
                self._error.mode = self._error.EFFORT_MODE
                self._error.names = joint_efforts.keys()
                #self._error.effort = joint_efforts.values() - self._limb.joint_efforts()
                self._error.effort = [ ( joint_efforts[k][0] - self._limb.joint_efforts()[k] ) for k in self._limb.joint_efforts().keys() ]
 
            # PUBLISH CMD CONTROL
            self._error.header.stamp = rospy.Time.now()
            self._pub_error.publish(self._error)

##################################################

            #rospy.sleep(tc)
            rate.sleep()
            #print idx

####################################################################################

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = [limb + '_' + joint for joint in \
            ['j0', 'j1', 'j2', 'j3', 'j4', 'j5', 'j6']]

    def _reset_control_modes(self):
        rate = rospy.Rate(self._rate)
        for _ in xrange(100):
            if rospy.is_shutdown():
                return False
            self._limb.exit_control_mode()
            #self._arm.exit_control_mode()
            self._pub_rate.publish(100)  # 100Hz default joint state rate
            rate.sleep()
        return True
    
    def clean_shutdown(self):
        """Handles ROS shutdown (Ctrl-C) safely."""
        print("\nExiting example...")
        if self._server:
            self.stop()
        #return to normal
        self._reset_control_modes()
#        self.move_to_neutral()
        #if not self._init_state:
        #    print("Disabling robot...")
        #    self._rs.disable()
        
        # Remove models from the scene on shutdown
        delete_gazebo_models()
                      
        return True

####################################################################################

def str2bool(v):
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

####################################################################################
####################################################################################
####################################################################################
####################################################################################
####################################################################################


# class PickAndPlace(object):
#     def __init__(self, limb="right", hover_distance = 0.15, tip_name="right_gripper_tip"):
#         self._limb_name = limb # string
#         self._tip_name = tip_name # string
#         self._hover_distance = hover_distance # in meters
#         self._limb = intera_interface.Limb(limb)
#         self._gripper = intera_interface.Gripper()
#         # verify robot is enabled
#         print("Getting robot state... ")
#         self._rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
#         self._init_state = self._rs.state().enabled
#         print("Enabling robot... ")
#         self._rs.enable()
# 
#     def move_to_start(self, start_angles=None):
#         print("Moving the {0} arm to start pose...".format(self._limb_name))
#         if not start_angles:
#             start_angles = dict(zip(self._joint_names, [0]*7))
#         self._guarded_move_to_joint_position(start_angles)
#         self.gripper_open()
#         
# #     def move_to_neutral(self, timeout=10.0):
# #         print("Moving the {0} arm to neutral pose...".format(self._limb_name))
# #         self._limb.move_to_neutral(timeout)
# #         #self._arm.move_to_neutral() #Sets arm back into a neutral pose.
# #         rospy.sleep(1.0)
# #         print("Running. Ctrl-c to quit")
# 
#     def move_to_start2(self, start_angles=None, timeout=10.0):
#         print("Moving the {0} arm to start pose...".format(self._limb_name))
#         if not start_angles:
#             start_angles = dict(zip(self._joint_names, [0]*7))
#         self._guarded_move_to_joint_position(start_angles, timeout)
#         print("Running. Ctrl-c to quit")
# 
#     def _guarded_move_to_joint_position(self, joint_angles, timeout=5.0):
#         if rospy.is_shutdown():
#             return
#         if joint_angles:
#             self._limb.move_to_joint_positions(joint_angles,timeout=timeout)
#         else:
#             rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")
# 
#     def gripper_open(self):
#         self._gripper.open()
#         rospy.sleep(1.0)
# 
#     def gripper_close(self):
#         self._gripper.close()
#         rospy.sleep(1.0)
# 
#     def _approach(self, pose):
#         approach = copy.deepcopy(pose)
#         # approach with a pose the hover-distance above the requested pose
#         approach.position.z = approach.position.z + self._hover_distance
#         joint_angles = self._limb.ik_request(approach, self._tip_name)
#         self._limb.set_joint_position_speed(0.001)
#         self._guarded_move_to_joint_position(joint_angles)
#         self._limb.set_joint_position_speed(0.1)
# 
#     def _retract(self):
#         # retrieve current pose from endpoint
#         current_pose = self._limb.endpoint_pose()
#         ik_pose = Pose()
#         ik_pose.position.x = current_pose['position'].x
#         ik_pose.position.y = current_pose['position'].y
#         ik_pose.position.z = current_pose['position'].z + self._hover_distance
#         ik_pose.orientation.x = current_pose['orientation'].x
#         ik_pose.orientation.y = current_pose['orientation'].y
#         ik_pose.orientation.z = current_pose['orientation'].z
#         ik_pose.orientation.w = current_pose['orientation'].w
#         self._servo_to_pose(ik_pose)
# 
#     def _servo_to_pose(self, pose, time=4.0, steps=400.0):
#         ''' An *incredibly simple* linearly-interpolated Cartesian move '''
#         r = rospy.Rate(1/(time/steps)) # Defaults to 100Hz command rate
#         current_pose = self._limb.endpoint_pose()
#         ik_delta = Pose()
#         ik_delta.position.x = (current_pose['position'].x - pose.position.x) / steps
#         ik_delta.position.y = (current_pose['position'].y - pose.position.y) / steps
#         ik_delta.position.z = (current_pose['position'].z - pose.position.z) / steps
#         ik_delta.orientation.x = (current_pose['orientation'].x - pose.orientation.x) / steps
#         ik_delta.orientation.y = (current_pose['orientation'].y - pose.orientation.y) / steps
#         ik_delta.orientation.z = (current_pose['orientation'].z - pose.orientation.z) / steps
#         ik_delta.orientation.w = (current_pose['orientation'].w - pose.orientation.w) / steps
#         for d in range(int(steps), -1, -1):
#             if rospy.is_shutdown():
#                 return
#             ik_step = Pose()
#             ik_step.position.x = d*ik_delta.position.x + pose.position.x
#             ik_step.position.y = d*ik_delta.position.y + pose.position.y
#             ik_step.position.z = d*ik_delta.position.z + pose.position.z
#             ik_step.orientation.x = d*ik_delta.orientation.x + pose.orientation.x
#             ik_step.orientation.y = d*ik_delta.orientation.y + pose.orientation.y
#             ik_step.orientation.z = d*ik_delta.orientation.z + pose.orientation.z
#             ik_step.orientation.w = d*ik_delta.orientation.w + pose.orientation.w
#             joint_angles = self._limb.ik_request(ik_step, self._tip_name)
#             if joint_angles:
#                 self._limb.set_joint_positions(joint_angles)
#             else:
#                 rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")
#             r.sleep()
#         rospy.sleep(1.0)
# 
#     def pick(self, pose):
#         if rospy.is_shutdown():
#             return
#         # open the gripper
#         self.gripper_open()
#         # servo above pose
#         self._approach(pose)
#         # servo to pose
#         self._servo_to_pose(pose)
#         if rospy.is_shutdown():
#             return
#         # close gripper
#         self.gripper_close()
#         # retract to clear object
#         self._retract()
# 
#     def place(self, pose):
#         if rospy.is_shutdown():
#             return
#         # servo above pose
#         self._approach(pose)
#         # servo to pose
#         self._servo_to_pose(pose)
#         if rospy.is_shutdown():
#             return
#         # open the gripper
#         self.gripper_open()
#         # retract to clear object
#         self._retract()

        
def main():
 
#     """RSDK Joint/Cartesian Position/Velocity Teleoperation: Cartesian control using Jacobian matrix
#   
#     Commands joint velocities to perform movements in the Cartesian space.
#     Demonstrates Joint Velocity Control Mode.
#     """
 
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-l', '--limb', required=True, choices=['left', 'right'],
        help='send joint trajectory to which limb'
    )
    parser.add_argument(
        '-s', '--sim', type=str2bool, nargs='?', const=True, default=False,
        help='set the mode simulation/real')
    parser.add_argument(
        '-c', '--control', choices=['position', 'velocity', 'effort'], default='position',
        help='set the low-level control position/velocity/effort')
    parser.add_argument(
        '-d', '--device', choices=['falcon', 'phantom', 'xsens'], default='phantom',
        help='set the input control device falcon/phantom/xsens')
    args = parser.parse_args(rospy.myargv()[1:])
    limb = args.limb
    sim = args.sim
    control = args.control
    device = args.device
     
    print("Initializing node... ")
    rospy.init_node("%s_arm_teleop" % (limb,))

    # Load Gazebo Models via Spawning Services
    # Note that the models reference is the /world frame
    # and the IK operates with respect to the /base frame
#    load_gazebo_models()

    teleop = TeleopControl(limb, sim = sim, control = control, device = device, server = False)
    
    rospy.on_shutdown(teleop.clean_shutdown)
    #rospy.on_shutdown(teleop.stop)
         
    # Gripper calibration (only simulation)
#    teleop.gripper_calibrate()
#    rospy.sleep(5.0)
#     teleop.gripper_open()
#     teleop.gripper_close()
    
    # Control loop to test different controllers  
    teleop.loop()
        
    return 0

####################################################################################

if __name__ == '__main__':
    sys.exit(main())
