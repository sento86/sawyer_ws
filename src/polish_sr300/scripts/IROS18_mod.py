#!/usr/bin/env python

"""
TITLE: A SLIDING MODE CONTROL ARCHITECTURE FOR HUMAN-MANIPULATOR COOPERATIVE SURFACE TREATMENT TASKS
"""
"""
Authors: Ernesto Solanes and Luis Gracia
Year:2017-2018
"""

import argparse
import importlib

import rospy
from dynamic_reconfigure.server import Server
from std_msgs.msg import Empty
from geometry_msgs.msg import WrenchStamped
from utilities import *

import intera_interface
from intera_interface import CHECK_VERSION
from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
    SolvePositionFK,
    SolvePositionFKRequest,
)

from std_msgs.msg import Header
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from sensor_msgs.msg import JointState
from operator import itemgetter
from numpy import *
import numpy as np #import math library

from std_msgs.msg import Float64
from operator import itemgetter

class SMC_Class(object):
    """

    """
    def __init__(self, reconfig_server, limb = "right"):
        self._dyn = reconfig_server

        # control parameters
        self._rate = 50.0  # Hz
        self._missed_cmds = 20.0  # Missed cycles before triggering timeout

        self._done = False

        self._joint_vel = np.zeros((7,1))
        self._Sensor = np.zeros((6,1))
        self._Sensor2 = np.zeros((6,1))
        self._Sensor2R = np.zeros((6,1))
        self._Sensor_ref = np.zeros((6,1))
        self._Sensor2_ref = np.zeros((6,1))
        self._sigma_max_ant = np.zeros((6,1))
        self._sigma_min_ant = np.zeros((6,1))
        self._dsigma_dp = np.zeros((6,6))
        self._sigma2_max = np.zeros((6,1))
        self._sigma2_max_ant = np.zeros((6,1))
        self._dsigma2_dp = np.zeros((6,6))
	self._temp_int = np.zeros((6,1))
        self._sm_ant = np.zeros((6,1))
        self._error = np.zeros((6,1))

        self._p_ref = np.zeros((6,1))
        self._pd_ref = np.zeros((6,1))
        self._pdd_ref = np.zeros((6,1))

        self._Ka_map = np.identity(6)
        self._U_plus_map = np.identity(6)
        
        self._U_plus_map[0][0] = 0.0
        self._U_plus_map[1][1] = 0.0
        self._U_plus_map[2][2] = 0.75
        self._U_plus_map[3][3] = 12.0
        self._U_plus_map[4][4] = 12.0
        self._U_plus_map[5][5] = 0.0
      
        self._Ka_map[3][3] = 1.15
        self._Ka_map[4][4] = 1.15

        self._Sensor_ref[0] = 20.0
        self._Sensor_ref[1] = 20.0
        self._Sensor_ref[2] = -10.0

        ## Colaborative mode
        self._Ka2_map = np.identity(6)
        self._U_plus2_map = np.identity(6)
        
        self._U_plus2_map[0][0] = 4.0
        self._U_plus2_map[5][5] = 2.0
        self._Ka2_map[0][0] = 0.5
        self._Ka2_map[5][5] = 0.5
        self._Sensor2_ref[0] = 30.0
        self._Sensor2_ref[5] = 0.0

        #1st Level control parameters
        self._Ka = 0.15
        self._U_plus = 0.06
        self._U_plus_map2 = self._U_plus_map*self._U_plus

        #2nd Level control parameters
        self._Kp = 5.0
        self._Kpd = np.sqrt(4.0)*1.5

        self._U_int_ant = np.zeros((6,1))
        self._phi_tmp_ant = np.zeros((6,1))
        self._phi_tmp_ant_1 = np.zeros((6,1))
        self._phi_tmp_ant_2 = np.zeros((6,1))
        self._phi_tmp_ant_3 = np.zeros((6,1))
        self._phi_tmp_ant_4 = np.zeros((6,1))

        self._U_int_L2_ant = np.zeros((6,1))
        self._phi_tmp_L2_ant = np.zeros((6,1))
        self._phi_tmp_L2_ant_1 = np.zeros((6,1))
        self._phi_tmp_L2_ant_2 = np.zeros((6,1))
        self._phi_tmp_L2_ant_3 = np.zeros((6,1))
        self._phi_tmp_L2_ant_4 = np.zeros((6,1))

        ## Collaborative mode
        self._Ka2 = 0.15
        self._U_plus2 = 0.06
        self._U_plus2_map2 = self._U_plus2_map*self._U_plus2
        self._alpha_max_z = 2.0*(math.pi/180.0)

        #3rd Level control parameters
        self._Kq = 0.75
        self._Kqd = 6*self._Kq 

        #3rd collaborative Level control parameters

        self._threshold = 1.0

        self._Kp_L3 = 1.1
        self._Kpd_L3 = 0.01

        #Gain approach
        self._ASG_opt = 0

        #Auto(1) or Collaborative(0)
        self._Auto = 0

        self._cnt = 0
        self._cnt2 = 2
        self._nContact = True

        self._time = 0.0
        self._step = 0.0005
        self._theta = 0.0
        self._r = 0.1
        #self._scale = 2.0

        #self._time = 0.0
        #self._step = 0.0
        #self._theta = 0.0
        #self._r = 0.0
        self._scale = 2.0

        self._Tp = False
        

        # create our limb instance
        self._limb = intera_interface.Limb(limb)
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = intera_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        print("Running. Ctrl-c to quit")

        print("Subscribing to netft node... ")
    
        self._ig = ForceSensorClass()
        self._sub_netft = rospy.Subscriber("/right_hand_ft_sensor/ft_sensor_topic", WrenchStamped, self._ig)
        print "Suscribed to netft_data_192_168_1_11"
        self._ig2 = ForceSensorClass()
        self._sub_netft2 = rospy.Subscriber("netft_data_192_168_1_10", WrenchStamped, self._ig2)
        print "Suscribed to netft_data_192_168_1_10"

        self._saveData = open("report.txt","w")


    def _IK_request(self,p,Q):

        ns = "ExternalTools/right/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        hdr = Header(stamp = rospy.Time.now(), frame_id = 'base')

        poses = {'right': PoseStamped(
                              header = hdr,
                              pose = Pose(
                              position = Point(
                                            x = p[0],
                                            y = p[1],
                                            z = p[2],
                              ),
                              orientation = Quaternion(
                                            x = Q[1],
                                            y = Q[2],
                                            z = Q[3],
                                            w = Q[0],
                              ),
                          ),
                      ),
                }
            
        ikreq = SolvePositionIKRequest()
    	# Add desired pose for inverse kinematics
    	ikreq.pose_stamp.append(poses['right'])
    	# Request inverse kinematics from base to "right_hand" link
    	ikreq.tip_names.append('right_hand')

    	try:
           rospy.wait_for_service(ns, 5.0)
           resp = iksvc(ikreq)
    	except (rospy.ServiceException, rospy.ROSException), e:
           rospy.logerr("Service call failed: %s" % (e,))

    	limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
    	#rospy.loginfo("\nIK Joint Solution:\n%s", limb_joints)
    	return limb_joints


    def _update_parameters(self):
        for joint in self._limb.joint_names():
            self._springs[joint] = self._dyn.config[joint[-2:] + '_spring_stiffness']
            self._damping[joint] = self._dyn.config[joint[-2:] + '_damping_coefficient']

    def _SMC_acc_Cartesian(self,XYZ,RPY,J):
        
        p = np.zeros((6,1))
        joint_acc_4 = np.zeros((7,1))
        dsigma_dp2 = np.zeros((6,6))
        phi_tmp_L2 = np.sign(self._phi2_max)

        Qd = self._limb.joint_velocities()
        Q  = self._limb.joint_angles()

        q  = np.zeros((7,1))
        qo  = np.zeros((7,1))
        qd = np.zeros((7,1))
 
        for i in range(0,6,1):
            if i < 3:
               p[i] = XYZ[i]
            else:
               p[i] = RPY[i-3]

        if p[3] < 0:
           p[3] = p[3] + 2.0*math.pi

        for i,joint in enumerate(self._limb.joint_names()):
            qo[i]  = self._start_angles[joint]
            q[i] = (self._limb.joint_angles())[joint]
            qd[i] = Qd[joint]
             
        pd = J.dot(qd)
            
        #Saturation of the feedforward error
        for i in range(0,6):
            self._error[i] = self._p_ref[i] - p[i]

        #Error saturation by direction
        m_e = np.sqrt( self._error[0]* self._error[0] +  self._error[1]* self._error[1] +  self._error[2]* self._error[2])
        
        if m_e > 0.025:
           for i in range(0,3):
                self._error[i] =  self._error[i]/m_e*0.025
       
        phi_tmp = np.sign(self._phi_max)

        Kint = np.zeros((6,1))
        Kint[2] = 0.02
        U_int = np.zeros((6,1))

        if self._ASG_opt == 1 and self._nContact == False: # Qualitative discrete ASG
           if phi_tmp[2] == self._phi_tmp_ant[2] and phi_tmp[2] == self._phi_tmp_ant_1[2] and phi_tmp[2] == self._phi_tmp_ant_2[2] and phi_tmp[2] == self._phi_tmp_ant_3[2]:
              U_int[2] = Kint[2]/self._rate + self._U_int_ant[2]
              self._U_plus_map2[2][2] = self._U_plus_map[2][2]*self._U_plus + U_int[2]
              self._U_int_ant[2] = U_int[2]
           elif phi_tmp[2] != self._phi_tmp_ant[2] and phi_tmp[2] != self._phi_tmp_ant_1[2] and phi_tmp[2] != self._phi_tmp_ant_2[2]:# and phi_tmp[2] != self._phi_tmp_ant_3[2]:
              U_int[2] = -Kint[2]/self._rate + self._U_int_ant[2]
              self._U_plus_map2[2][2] = self._U_plus_map[2][2]*self._U_plus + U_int[2]
              self._U_int_ant[2] = U_int[2]

           self._phi_tmp_ant_4[2] = self._phi_tmp_ant_3[2]
           self._phi_tmp_ant_3[2] = self._phi_tmp_ant_2[2]
           self._phi_tmp_ant_2[2] = self._phi_tmp_ant_1[2]
           self._phi_tmp_ant_1[2] = self._phi_tmp_ant[2]
           self._phi_tmp_ant[2] = phi_tmp[2]

        else: # Default: classic fixed gain
	   self._U_plus_map2[2][2] = self._U_plus_map2[2][2]

        #1st Level SMC:
        joint_acc = -linalg.pinv(self._Ka*(self._dsigma_dp.T).dot(J),rcond = 0.01).dot(self._U_plus_map2.dot(phi_tmp))

        #2nd Level trajectory tracking:
        
        # Automatic or Collaborative mode
        if self._Auto == 1 or self._nContact == True:
            N2 = np.eye(7) - linalg.pinv((self._dsigma_dp.T).dot(J),rcond = 0.01).dot((self._dsigma_dp.T).dot(J))
            joint_acc_2 = linalg.pinv(J.dot(N2),rcond = 0.01).dot(self._pdd_ref + self._Kpd*(self._pd_ref - pd) + self._Kp*self._error - J.dot(joint_acc))
            
            #3rf Level self motion:
            N3 = N2.dot(np.eye(7) - np.linalg.pinv(J.dot(N2)).dot(J.dot(N2)))
            joint_acc_3 = linalg.pinv(N3,rcond = 0.01).dot(-self._Kqd*qd + self._Kq*(qo - q) - (joint_acc + joint_acc_2))
            self._joint_vel = self._joint_vel + (joint_acc + joint_acc_2 + joint_acc_3)/self._rate

        elif self._Auto == 0 or self._nContact == False:
            # Level 2: X, Y, Z inequalities activation

            if self._phi2_max[0] > 0:       
               phi_tmp_L2[0] = 1.0
               dsigma_dp2[0][0] = (2.0*self._Sensor2R[0])/(2.0*np.sqrt(self._Sensor2R[0]*self._Sensor2R[0] + self._Sensor2R[1]*self._Sensor2R[1]))
               dsigma_dp2[1][0] = (2.0*self._Sensor2R[1])/(2.0*np.sqrt(self._Sensor2R[0]*self._Sensor2R[0] + self._Sensor2R[1]*self._Sensor2R[1]))
            else:
               phi_tmp_L2[0] = 0.0
               dsigma_dp2[0][0] = 0.0
               dsigma_dp2[1][0] = 0.0

            Kint_L2 = np.zeros((6,1))
            Kint_L2[0] = 0.04

            U_int_L2 = np.zeros((6,1))

            if (phi_tmp_L2[0] == 1 and self._phi_tmp_L2_ant[0] == 1 and self._phi_tmp_L2_ant_2[0] == 1 and self._phi_tmp_L2_ant_3[0] == 1 and self._phi_tmp_L2_ant_4[0] == 1) or (phi_tmp[0] == -1 and self._phi_tmp_L2_ant[0] == -1 and self._phi_tmp_L2_ant_2[0] == -1 and self._phi_tmp_L2_ant_3[0] == -1 and self._phi_tmp_L2_ant_4[0] == -1):
               U_int_L2[0] = Kint_L2[0]/self._rate + self._U_int_L2_ant[0]
               self._U_plus2_map2[0][0] = self._U_plus2_map[0][0]*self._U_plus2 + U_int_L2[0]
               self._U_int_L2_ant[0] = U_int_L2[0]
            elif (phi_tmp_L2[0] == 0 and self._phi_tmp_L2_ant[0] == 1 and self._phi_tmp_L2_ant_1[0] == 0) or (phi_tmp_L2[0] == 0 and self._phi_tmp_L2_ant[0] == -1 and self._phi_tmp_L2_ant_1[0] == 0):
               U_int_L2[0] = -Kint_L2[0]/self._rate + self._U_int_L2_ant[0]
               self._U_plus2_map2[0][0] = self._U_plus2_map[0][0]*self._U_plus2 + U_int_L2[0]
               self._U_int_L2_ant[0] = U_int_L2[0]

            self._phi_tmp_L2_ant_4[0] = self._phi_tmp_L2_ant_3[0]
            self._phi_tmp_L2_ant_3[0] = self._phi_tmp_L2_ant_2[0]
            self._phi_tmp_L2_ant_2[0] = self._phi_tmp_L2_ant_1[0]
            self._phi_tmp_L2_ant_1[0] = self._phi_tmp_L2_ant[0]
            self._phi_tmp_L2_ant[0] = phi_tmp_L2[0]

            N2 = np.eye(7) - linalg.pinv((self._dsigma_dp.T).dot(J),rcond = 0.01).dot((self._dsigma_dp.T).dot(J))
            joint_acc_2 = -linalg.pinv(((dsigma_dp2.T).dot(J)).dot(N2),rcond = 0.01).dot((np.sign(phi_tmp_L2.T).dot(self._U_plus2_map2)).T + J.dot(joint_acc))


            #3rd Level cartesian self motion:        
            N3 = N2.dot(np.eye(7) - np.linalg.pinv(((dsigma_dp2.T).dot(J)).dot(N2)).dot(((dsigma_dp2.T).dot(J)).dot(N2)))
            joint_acc_3 = linalg.pinv(J.dot(N3),rcond = 0.01).dot(self._pdd_ref + self._Kpd*(self._pd_ref - pd) + self._Kp*self._error + np.sign(self._pd_ref - pd + self._Kp/self._Kpd*self._error)*self._Kpd_L3 - J.dot(joint_acc + joint_acc_2))
           

            #4th Level home configuration:
            N4 = N3.dot(np.eye(7) - np.linalg.pinv(J.dot(N3)).dot(J.dot(N3)))
            joint_acc_4 = linalg.pinv(N4,rcond = 0.01).dot(-self._Kqd*qd  +  self._Kq*(qo - q) - (joint_acc + joint_acc_2 + joint_acc_3))
            
            self._joint_vel = self._joint_vel + (joint_acc + joint_acc_2 + joint_acc_3 + joint_acc_4)/self._rate

        cmd = make_cmd(self._limb.joint_names(), self._joint_vel)

        # command new joint torques
        self._limb.set_joint_velocities(cmd) 
        b = ""
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(p.T)),decimals = 4)))
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(pd.T)),decimals = 4)))
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._error.T)),decimals = 4)))
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(qo.T)),decimals = 4)))
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(q.T)),decimals = 4)))
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(qd.T)),decimals = 4)))
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(joint_acc.T)),decimals = 4)))
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(joint_acc_2.T)),decimals = 4)))
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(joint_acc_3.T)),decimals = 4)))
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(joint_acc_4.T)),decimals = 4)))
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray((joint_acc + joint_acc_2 + joint_acc_3 + joint_acc_4).T)),decimals = 4)))
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._joint_vel.T)),decimals = 4)))
        b += " " + str(np.round(self._dsigma_dp[0][0],decimals = 4)) + " " + str(np.round(self._dsigma_dp[1][1],decimals = 4))
        b += " " + str(np.round(self._dsigma_dp[2][2],decimals = 4)) + " " + str(np.round(self._dsigma_dp[3][3],decimals = 4)) 
        b += " " + str(np.round(self._dsigma_dp[4][4],decimals = 4)) + " " + str(np.round(self._dsigma_dp[5][5],decimals = 4))
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(phi_tmp.T)),decimals = 4)))
        b += " " + str(np.round(self._U_plus_map2[2][2],decimals = 4)) + " " + str(np.round(self._U_plus_map2[3][3],decimals = 4)) + " " + str(np.round(self._U_plus_map2[4][4],decimals = 4))
        b += " " + str(np.round(dsigma_dp2[0][0],decimals = 4)) + " " + str(np.round(dsigma_dp2[1][1],decimals = 4))
        b += " " + str(np.round(dsigma_dp2[2][2],decimals = 4)) + " " + str(np.round(dsigma_dp2[3][3],decimals = 4)) 
        b += " " + str(np.round(dsigma_dp2[4][4],decimals = 4)) + " " + str(np.round(dsigma_dp2[5][5],decimals = 4))
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(phi_tmp_L2.T)),decimals = 4)))
        b += " " + str(np.round(self._U_plus2_map2[0][0],decimals = 4)) + " " + str(np.round(self._U_plus_map2[2][2],decimals = 4))

        return b


    def move_to_neutral(self):
        """
        Moves the limb to neutral location.
        """
        self._limb.move_to_neutral(speed=0.1)

    def move_to_position(self, joint_pos):
        cmd = make_cmd(self._limb.joint_names(), joint_pos)
        self._limb.set_joint_position_speed(0.05)
        self._limb.move_to_joint_positions(cmd,timeout=20.0)
        self._limb.set_joint_position_speed(0.3) #speed back to default

    def Control_loop_Cart(self):
        """

        """
 
        # record initial joint angles
        self._start_angles = self._limb.joint_angles()

        # set control rate
        control_rate = rospy.Rate(self._rate)

        # for safety purposes, set the control rate command timeout.
        # if the specified number of command cycles are missed, the robot
        # will timeout and return to Position Control Mode
        self._limb.set_command_timeout((1.0 / self._rate) * self._missed_cmds)
       
        pose = self._limb.endpoint_pose()
        XYZ = pose['position']
        RPY = Quat2rpy(pose['orientation'])

        for i in range(0,6,1):
            if i < 3:
               self._p_ref[i] = XYZ[i]
               self._pd_ref[i] = 0.0
            else:
               self._p_ref[i]=RPY[i-3]
               self._pd_ref[i] = 0.0

        self._pd_ref[2] = -0.015

        if self._p_ref[3] < 0:
           self._p_ref[3] = self._p_ref[3] + 2.0*math.pi


        a = "Period " + str(1/self._rate)
        a += "Sensor Filter (Wc = 73 Hz)" + "\n"
        a += "Time(sec) " + "Sensor (1x6) " + "Sigma_max (1x6) " + "Sigma_min (1x6) " + "Sigma_d_max (1x6) " + "Sigma_d_min (1x6) "
        a += "phi_max (1x6) " + "phi_min (1x6) " + "p (1x6) " + "pd (1x6) "
        a += "qo(1x7) " + "q (1x7) " + "qd (1x7) " + "joint_acc Level 1 (1x7) " + "joint_acc Level 2 (1x7) "
        a += "joint_acc Level 3 (1x7) " + "joint_acc Total (1x7) " + "joint_vel cmd (1x7) " + "dsigma (1x6) "
        a += "phi_tmp (1x6) "

        # loop at specified rate commanding new joint torques
        while not rospy.is_shutdown()  and not self._done:
            
            start = rospy.get_time()

            if not self._rs.state().enabled:
                rospy.logerr("Joint torque example failed to meet "
                             "specified control rate timeout.")
                break

            msg = self._ig.get_msg() #Wrench info
            msg2 = self._ig2.get_msg() #Wrench info

            #####################################
            #guidance sensor
            pose = self._limb.endpoint_pose()

            DH = DenHartStandard(self._limb,self._limb.joint_names())
            XYZ = pose['position']
            RPY = Quat2rpy(pose['orientation'])

            if RPY[0] < 0:
               RPY[0] = RPY[0] + 2.0*math.pi

            Rot = rpy2R(RPY)

            self._Sensor2[1] = msg2.wrench.force.x
            self._Sensor2[0] = msg2.wrench.force.y
            self._Sensor2[2] = - msg2.wrench.force.z
            self._Sensor2[3] = msg2.wrench.torque.x
            self._Sensor2[4] = msg2.wrench.torque.y
            self._Sensor2[5] = - msg2.wrench.torque.z
       
            self._Sensor2R[np.ix_([0,1,2],[0])] = Rot.dot(self._Sensor2[np.ix_([0,1,2],[0])])

            for i in range(3,6,1):
                self._Sensor2R[i] = self._Sensor2[i]

            self._sigma2_max[0] = (self._Sensor2R[0]*self._Sensor2R[0] + self._Sensor2R[1]*self._Sensor2R[1]) - self._Sensor2_ref[0]
            
            self._sigma2_max[5] = (np.absolute(self._Sensor2R[5])-self._Sensor2_ref[5])-self._alpha_max_z # Yaw control in Level 2
                 
            self._sigma2_d_max = (self._sigma2_max - self._sigma2_max_ant)*self._rate
            
            for i in range(0,6,1):
                self._sigma2_max_ant[i] = self._sigma2_max[i]

            self._phi2_max = self._sigma2_max + self._Ka2_map.dot(self._Ka2*self._sigma2_d_max)

            ##############################
            #Polishing sensor
            self._Sensor[0] = msg.wrench.force.x
            self._Sensor[1] = -msg.wrench.force.y
            self._Sensor[2] = msg.wrench.force.z
            self._Sensor[3] = msg.wrench.torque.x*1000.0
            self._Sensor[4] = msg.wrench.torque.y*1000.0
            #self._Sensor[5] = msg.wrench.torque.z*1000.0
            

            self._sigma_max = self._Sensor - self._Sensor_ref
            self._sigma_min = -(self._Sensor + self._Sensor_ref)
                   
            self._sigma_d_max = (self._sigma_max - self._sigma_max_ant)*self._rate
            self._sigma_d_min = (self._sigma_min - self._sigma_min_ant)*self._rate

            self._phi_max = self._sigma_max + self._Ka_map.dot(self._Ka*self._sigma_d_max)
            self._phi_min = self._sigma_min + self._Ka_map.dot(self._Ka*self._sigma_d_min)

            J = jacobn_rec(jacob0(self._limb,self._limb.joint_names(),DH),RPY)
 
            b = self._SMC_acc_Cartesian(XYZ,RPY,J)

            self._sigma_max_ant = self._sigma_max
            self._sigma_min_ant = self._sigma_min
            ###############################

            control_rate.sleep()

            if self._nContact == False:
               self._time += rospy.get_time() - start

            if self._nContact:
                self._p_ref[2] = self._p_ref[2] -0.005/self._rate
		if self._Sensor[2] < self._Sensor_ref[2]:
                   self._dsigma_dp[2][2] = 2.0
                   self._dsigma_dp[3][3] = 15.0
                   self._dsigma_dp[4][4] = 15.0
                   self._pd_ref[2] = 0.0
                   #self._pd_ref[1] = 0.02
                   #self._pd_ref[0] = 0.01
                   self._nContact = False
            else:
                if self._theta > 2.0*np.pi*self._scale:
                   self._theta = 0.0
                   self._cnt2 = self._cnt2 + 1

                if self._phi2_max[0] > 0:
                   self._Tp = True
              
                if self._Tp == False:
                   self._pdd_ref[0] = -self._r/(self._scale*self._scale)*np.cos(self._theta/self._scale)
                   self._pdd_ref[1] = -self._r/(self._scale*self._scale)*np.sin(self._theta/self._scale)
                
                   self._pd_ref[0] = -self._r/self._scale*np.sin(self._theta/self._scale)
                   self._pd_ref[1] =  self._r/self._scale*np.cos(self._theta/self._scale)
                
                   self._p_ref[1] = self._p_ref[1] + self._pd_ref[1]/self._rate
                   self._p_ref[0] = self._p_ref[0] + self._pd_ref[0]/self._rate

                   self._theta += self._step*self._rate

                elif self._Tp == True and np.sqrt(self._error[0]*self._error[0] + self._error[1]*self._error[1]) >= 0.01:
                   
                   self._pdd_ref[0] = 0.0
                   self._pdd_ref[1] = 0.0
                
                   self._pd_ref[0] = 0.0
                   self._pd_ref[1] = 0.0
                
                   self._p_ref[1] = self._p_ref[1]
                   self._p_ref[0] = self._p_ref[0]

                elif self._Tp == True and np.sqrt(self._error[0]*self._error[0] + self._error[1]*self._error[1] + self._error[2]*self._error[2]) < 0.01:
                   self._Tp = False

                a += "\n"+ str(np.round(float(self._time),4))
                a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._Sensor.T)),decimals = 4)))
                a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._sigma_max.T)),decimals = 4)))
                a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._sigma_min.T)),decimals = 4)))
                a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._sigma_d_max.T)),decimals = 4)))
                a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._sigma_d_min.T)),decimals = 4)))
                a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._phi_max.T)),decimals = 4)))
                a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._phi_min.T)),decimals = 4)))
                a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._p_ref.T)),decimals = 4)))
                a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._pd_ref.T)),decimals = 4)))
                a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._pdd_ref.T)),decimals = 4)))
                a += b
                a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._Sensor2.T)),decimals = 4)))
                a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._Sensor2R.T)),decimals = 4)))
                a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._sigma2_max.T)),decimals = 4)))
                a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._sigma2_d_max.T)),decimals = 4)))
                a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._phi2_max.T)),decimals = 4)))
                self._saveData.write(a)
                a = ""

                if self._cnt2 > 100:
                   self._done = True

            if msg.wrench.force.z < -100 or np.absolute(msg.wrench.force.x) > 50 or np.absolute(msg.wrench.force.y) > 50:
         	self._done = True


    def clean_shutdown(self):
        """
        Switches out of joint torque mode to exit cleanly
        """
        print("\nExiting example...")
        self._limb.exit_control_mode()


def main():
    """

    """
    # Querying the parameter server to determine Robot model and limb name(s)
    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    if not valid_limbs:
        rp.log_message(("Cannot detect any limb parameters on this robot. "
                        "Exiting."), "ERROR")
    robot_name = intera_interface.RobotParams().get_robot_name().lower().capitalize()
    # Parsing Input Arguments
    arg_fmt = argparse.ArgumentDefaultsHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt)
    parser.add_argument(
        "-l", "--limb", dest="limb", default=valid_limbs[0],
        choices=valid_limbs,
        help='limb on which to attach joint springs'
        )
    args = parser.parse_args(rospy.myargv()[1:])
    # Grabbing Robot-specific parameters for Dynamic Reconfigure
    config_name = ''.join([robot_name,"JointSpringsExampleConfig"])
    config_module = "intera_examples.cfg"
    cfg = importlib.import_module('.'.join([config_module,config_name]))
    # Starting node connection to ROS
    print("Initializing node... ")
    rospy.init_node("sdk_joint_torque_springs_{0}".format(args.limb))
    dynamic_cfg_srv = Server(cfg, lambda config, level: config)
    js = SMC_Class(dynamic_cfg_srv, limb=args.limb)
    # register shutdown callback
    rospy.on_shutdown(js.clean_shutdown)
    #js.move_to_neutral()
    # Move slowly to an user defined pose
    #joint_pos = np.array([0.0014580078125, -0.5757373046875, 0.0044482421875, 2.041927734375, -0.06414453125, 0.10119921875, 3.3797802734375,],dtype=np.float)
    ###joint_pos = np.array([ 0.01821875, -0.6355078125, -0.0242001953125, 1.66391015625, 0.036236328125, 0.526962890625, 3.286228515625],dtype=np.float)

    #####joint_pos = np.array([0.013896484375, -0.585494140625, -0.01746875, 1.64569140625, 0.0315244140625, 0.4926845703125, 3.2914306640625],dtype=np.float)

    joint_pos = np.array([0.0220263671875, -0.4779541015625, -0.0312109375, 1.3845087890625, 0.0467080078125, 0.6515537109375, 3.284783203125],dtype=np.float)

    js.move_to_position(joint_pos)
    rospy.sleep(1.0)
    js.Control_loop_Cart()


if __name__ == "__main__":
    main()
