#!/usr/bin/env python

"""
TITLE: ROBUST HYBRID POSITION-FORCE CONTROL FOR ROBOTIC SURFACE POLISHING
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

        self._Sensor = np.zeros((6,1))
        self._Sensor_ref = np.zeros((6,1))
        self._sigma_max_ant = np.zeros((6,1))
        self._sigma_min_ant = np.zeros((6,1))
        self._dsigma_dp = np.zeros((6,6))
        self._Forces = np.zeros((6,1))
	self._temp_int = np.zeros((6,1))
        self._sm_ant = np.zeros((6,1))

        self._Ka_map = np.identity(6)
        self._U_plus_map = np.identity(6)

        self._U_plus_map[0][0] = 5.75 #10 chattering free
        self._U_plus_map[1][1] = 5.75 #10 chattering free
        self._U_plus_map[2][2] = 0.35
        self._U_plus_map[3][3] = 14.0
        self._U_plus_map[4][4] = 14.0
        self._U_plus_map[5][5] = 0.0
      
        self._Ka_map[3][3] = 1.3
        self._Ka_map[4][4] = 1.3

        self._Sensor_ref[0] = 20.0
        self._Sensor_ref[1] = 20.0
        self._Sensor_ref[2] = -15.0

        #1st Level control parameters
        self._Ka = 0.15
        self._U_plus = 0.06

        #2nd Level control parameters
        self._Kp = 5.0
        self._Kpd = np.sqrt(4.0)*1.5
        self._p_ref = np.zeros((6,1))
        self._pd_ref = np.zeros((6,1))
        self._pdd_ref = np.zeros((6,1))

        #3rd Level control parameters
        self._Kq = 1.0
        self._Kqd = 0.5

        self._cnt = 0
        self._cnt2 = 2
        self._nContact = True

        self._time = 0.0
        self._step = 0.0005
        self._theta = 0.0
        self._r = 0.1
        self._scale = 2.0

        #Equalities modes:
        # default (0) -> sign(phi)
        # (1) -> sign aprox: (sigma+k*sigma_d)/np.sqrt((sigma+k*sigma_d[2])*(sigma+k*sigma_d)+E*E)
        # (2) -> sign aprox: tanh((sigma+k*sigma_d)/Wgh)
        # (3) -> Quasi-continuous HOSMC [Levant2005]: sign(sigma_d+k*sqrt(abs(sigma))*sign(sigma))
        # (4) -> Twisting r1*np.sign(sigma)+r2*np.sign(sigma_d) [kunusch12]
        # (5) -> Super-Twisting [Utkin16]
        # (6) -> sub-optimal alg. [Kunusch12]
        self._mode_eq = 5

        #Inequalities modes:
        # default (0) -> sign(phi)
        # (1) -> sign aprox: (sigma+k*sigma_d)/np.sqrt((sigma+k*sigma_d[2])*(sigma+k*sigma_d)+E*E)
        # (2) -> sign aprox: tanh((sigma+k*sigma_d)/Wgh)
        self._mode_ineq = 2

        self._joint_vel = np.zeros((7,1))


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

    def _SMC_acc_Cartesian(self,XYZ,RPY,J):
        
        p = np.zeros((6,1))
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
        error = np.zeros((6,1))
        error[3] = self._p_ref[3] - p[3]
	error[4] = self._p_ref[4] - p[4]
        error[5] = self._p_ref[5] - p[5]

        for i in range(0,3):
            if np.abs(self._p_ref[i] - p[i]) > 0.02:
               error[i] = np.sign(self._p_ref[i] - p[i])*0.02
            else:
               error[i] = self._p_ref[i] - p[i]

        phi_tmp = np.sign(self._phi_max)

        #Level 1: Equality
        if self._mode_eq == 1:
           #Chattering-free approach: aprox sign with (sigma+k*sigma_d)/np.sqrt((sigma+k*sigma_d[2])*(sigma+k*sigma_d)+E*E)
           E = 10.0
           kaprox1 = 0.1
           self._U_plus_map[3][3] = 5.0
           self._U_plus_map[4][4] = 5.0
           phi_tmp[2] = (self._sigma_max[2] + kaprox1*self._sigma_d_max[2])/np.sqrt((self._sigma_max[2] + self._sigma_d_max[2])*(self._sigma_max[2] + self._sigma_d_max[2]) + E*E) #self._phi_max[2]/np.sqrt(pself._hi_max[2]*self._phi_max[2] + E*E) 
           phi_tmp[3] = (self._sigma_max[3] + kaprox1*self._sigma_d_max[3])/np.sqrt((self._sigma_max[3] + self._sigma_d_max[3])*(self._sigma_max[3] + self._igma_d_max[3]) + E*E) #self._phi_max[3]/np.sqrt(self._phi_max[3]*self._phi_max[3] + E*E)
           phi_tmp[4] = (self._sigma_max[4] + kaprox1*self._sigma_d_max[4])/np.sqrt(self._(sigma_max[4] + self._sigma_d_max[4])*(self._sigma_max[4] + self._sigma_d_max[4]) + E*E) #self._phi_max[4]/np.sqrt(self._phi_max[4]*self._phi_max[4] + E*E)
        elif self._mode_eq == 2:
           #Chattering-free approach: aprox sign with tanh((sigma+k*sigma_d)/Wgh)
           Wgh = 10.0
           kaprox2_Z = 0.1
           kaprox2_wx = 0.3
           kaprox2_wy = 0.3
           self._U_plus_map[3][3] = 2.5
           self._U_plus_map[4][4] = 2.5
           phi_tmp[2] = np.tanh((self._sigma_max[2] + kaprox2_Z*self._sigma_d_max[2])/Wgh)
           phi_tmp[3] = np.tanh((self._sigma_max[3] + kaprox2_wx*self._sigma_d_max[3])/Wgh)
           phi_tmp[4] = np.tanh((self._sigma_max[4] + kaprox2_wy*self._sigma_d_max[4])/Wgh)
        elif self._mode_eq == 3:
           #Chattering-free approach: Quasi-continuous sign(sigma_d+k*sqrt(abs(sigma))*sign(sigma)) [Levant2005]
           kHOSMC_Z = 5.0
           kHOSMC_wx = 50.0
           kHOSMC_wy = 50.0
           self._U_plus_map[3][3] = 10.0
           self._U_plus_map[4][4] = 10.0
           phi_tmp[2] = np.sign(self._sigma_d_max[2] + kHOSMC_Z*np.sqrt(np.abs(self._sigma_max[2]))*self._sign(self._sigma_max[2]))
           phi_tmp[3] = np.sign(self._sigma_d_max[3] + kHOSMC_wx*np.sqrt(np.abs(self._sigma_max[3]))*self._sign(self._sigma_max[3]))
           phi_tmp[4] = np.sign(self._sigma_d_max[4] + kHOSMC_wy*np.sqrt(np.abs(self._sigma_max[4]))*self._sign(self._sigma_max[4]))
        elif self._mode_eq == 4:
           #Chattering-free approach: Twisting r1*np.sign(sigma)+r2*np.sign(sigma_d) [kunusch12]
           r1_Z = 0.5
           r2_Z = 0.7
           r1_wx = 0.5
	   r2_wx = 0.9
           r1_wy = 0.5
	   r2_wy = 0.9
           self._U_plus_map[3][3] = 5.5
           self._U_plus_map[4][4] = 5.5
           phi_tmp[2] = r1_Z*np.sign(self._sigma_max[2])  + r2_Z*np.sign(self._sigma_d_max[2])
           phi_tmp[3] = (r1_wx*np.sign(self._sigma_max[3]) + r2_wx*np.sign(self._sigma_d_max[3]))
           phi_tmp[4] = (r1_wy*np.sign(self._sigma_max[4]) + r2_wy*np.sign(self._sigma_d_max[4]))
        elif self._mode_eq == 5:
           #Chattering-free approach: Super-Twisting [Utkin16]
           ka_Z = 0.2
	   ka_wx = 0.1
           ka_wy = 0.1
           M_Z  = 0.03
           M_wx = 0.0015
           M_wy = 0.0015
           self._U_plus_map[3][3] = 9.0
           self._U_plus_map[4][4] = 9.0

           if self._nContact == False:
              self._temp_int[2] = self._temp_int[2] + M_Z*np.sign(self._sigma_max[2] + ka_Z*self._sigma_d_max[2])
              self._temp_int[3] = self._temp_int[3] + M_wx*np.sign(self._sigma_max[3] + ka_wx*self._sigma_d_max[3])
              self._temp_int[4] = self._temp_int[4] + M_wy*np.sign(self._sigma_max[4] + ka_wy*self._sigma_d_max[4])
           
           phi_tmp[2] = 0.2*(np.sqrt(np.abs(self._sigma_max[2] + ka_Z*self._sigma_d_max[2]))*sign((self._sigma_max[2] + ka_Z*self._sigma_d_max[2])) + self._temp_int[2])
           phi_tmp[3] = 0.06*(np.sqrt(np.abs(self._sigma_max[3] + ka_Z*self._sigma_d_max[3]))*sign((self._sigma_max[3] + ka_Z*self._sigma_d_max[3])) + self._temp_int[3])
           phi_tmp[4] = 0.06*(np.sqrt(np.abs(self._sigma_max[4] + ka_Z*self._sigma_d_max[4]))*sign((self._sigma_max[4] + ka_Z*self._sigma_d_max[4])) + self._temp_int[4])
        elif mode_eq == 6:
           #Chattering-free approach: sub-optimal alg. [Kunusch12]
           Beta_Z = 1.0
           Beta_wx = 2.1
           Beta_wy = 2.1
           alpha_Z = 10.5
           alpha_wx = 50.1
           alpha_wy = 50.1
           self._U_plus_map[3][3] = 5.0
           self._U_plus_map[4][4] = 5.0
           sm = np.zeros((6,1))
           alpha = np.zeros((6,1))

           if self._sigma_d_max[2] > 0:
              self._sm[2] = self._sigma_max[2]
           else:
              self._sm[2] = self._sm_ant[2]
           if self._sigma_d_max[3] > 0:
              self._m[3] = self._sigma_max[3]
           else:
              self._sm[3] = self._sm_ant[3]
           if sigma_d_max[4] > 0:
              self._sm[4] = self._sigma_max[4]
           else:
              self._sm[4] = self._sm_ant[4]

           if (self._sigma_max[2]-Beta_Z*self._sm[2])*self._sm[2] >= 0:
              alpha[2] = 1.0
           else:
              alpha[2] = alpha_Z
           if (self._sigma_max[3]-Beta_Z*self._sm[3])*self._*sm[3] >= 0:
              alpha[3] =1.0
           else:
              alpha[3] = alpha_wx
           if (self._sigma_max[4]-Beta_Z*self._sm[4])*self._sm[4] >= 0:
              alpha[4] = 1.0
           else:
              alpha[4] = alpha_wy

           phi_tmp[2] = alpha[2]*np.sign(self._sigma_max[2]-Beta_Z*self._sm[2])
           phi_tmp[3] = alpha[3]*np.sign(self._sigma_max[3]-Beta_Z*self._sm[3])
           phi_tmp[4] = alpha[4]*np.sign(self._sigma_max[4]-Beta_Z*self._sm[4])
           
           if self._nContact == False:
              self._sm_ant = self._sm
           
        else:
           #Level 1: Equality
           phi_tmp = np.sign(self._phi_max)

        # Level 1: X, Y inequalities activation
        if self._mode_ineq == 1:
           #Chattering-free approach: aprox sign with phi/sqrt(phi^2+E^2)
           E = 10.0
           kaprox1 = 0.3
           if self._phi_max[1] > 0:       
              phi_tmp[1] = (self._sigma_max[1]+kaprox1*self._sigma_d_max[1])/np.sqrt((self._sigma_max[1]+kaprox1*self._sigma_d_max[1])*(self._sigma_max[1]+kaprox1*self._sigma_d_max[1])+E*E) #phi_max[1]/np.sqrt(phi_max[1]*phi_max[1] + E*E)
              self._dsigma_dp[1][1] = 4.0
           elif phi_min[1] > 0:
              phi_tmp[1] = -(self._sigma_min[1]+kaprox1*self._sigma_d_min[1])/np.sqrt((self._sigma_min[1]+kaprox1*self._sigma_d_min[1])*(self._sigma_min[1]+kaprox1*self._sigma_d_min[1])+E*E) #-phi_min[1]/np.sqrt(phi_min[1]*phi_min[1] + E*E)
              self._dsigma_dp[1][1] = 4.0
           else:
              self._dsigma_dp[1][1] = 0.0
              phi_tmp[1] = 0.0
           if self._phi_max[0] > 0:       
              phi_tmp[0] = (self._sigma_max[0]+kaprox1*self._sigma_d_max[0])/np.sqrt((self._sigma_max[0]+kaprox1*self._sigma_d_max[0])*(self._sigma_max[0]+kaprox1*self._sigma_d_max[0])+E*E) #phi_max[0]/np.sqrt(phi_max[0]*phi_max[0] + E*E)
              self._dsigma_dp[0][0] = 4.0
           elif phi_min[0] > 0:
              phi_tmp[0] = -(self._sigma_min[0]+kaprox1*self._sigma_d_min[0])/np.sqrt((self._sigma_min[0]+kaprox1*self._sigma_d_min[0])*(self._sigma_min[0]+kaprox1*self._sigma_d_min[0])+E*E) #-phi_min[0]/np.sqrt(phi_min[0]*phi_min[0] + E*E)
              self._dsigma_dp[0][0] = 4.0
           else:
              self._dsigma_dp[0][0] = 0.0
              phi_tmp[0] = 0.0
	elif self._mode_ineq == 2:
           #Chattering-free approach: aprox sign with tanh((sigma+k*sigma_d)/Wgh)
           Wgh = 7.0
           kaprox2 = 0.35
           if self._phi_max[1] > 0:       
              phi_tmp[1] = np.tanh((self._sigma_max[1]+kaprox2*self._sigma_d_max[1])/Wgh)
              self._dsigma_dp[1][1] = 4.0
           elif self._phi_min[1] > 0:
              phi_tmp[1] = -np.tanh((self._sigma_min[1]+kaprox2*self._sigma_d_min[1])/Wgh)
              self._dsigma_dp[1][1] = 4.0
           else:
              self._dsigma_dp[1][1] = 0.0
              phi_tmp[1] = 0.0
           if self._phi_max[0] > 0:       
              phi_tmp[0] = np.tanh((self._sigma_max[0]+kaprox2*self._sigma_d_max[0])/Wgh)
              dsigma_dp[0][0] = 4.0
           elif self._phi_min[0] > 0:
              phi_tmp[0] = -np.tanh((self._sigma_min[0]+kaprox2*self._sigma_d_min[0])/Wgh)
              self._dsigma_dp[0][0] = 4.0
           else:
              self._dsigma_dp[0][0] = 0.0
              phi_tmp[0] = 0.0    
	else:
           if self._phi_max[1] > 0:       
              phi_tmp[1] = 1.0
              self._dsigma_dp[1][1] = 4.0
           elif self._phi_min[1] > 0:
              phi_tmp[1] = -1.0
              self._dsigma_dp[1][1] = 4.0
           else:
              self._dsigma_dp[1][1] = 0.0
              phi_tmp[1] = 0.0

           if self._phi_max[0] > 0:       
              phi_tmp[0] = 1.0
              self._dsigma_dp[0][0] = 4.0
           elif self._phi_min[0] > 0:
              phi_tmp[0] = -1.0
              self._dsigma_dp[0][0] = 4.0
           else:
              dsigma_dp[0][0] = 0.0
              self._phi_tmp[0] = 0.0

        #1st Level SMC:
        joint_acc = -linalg.pinv(self._Ka*(self._dsigma_dp.T).dot(J),rcond = 0.01).dot(self._U_plus_map.dot(phi_tmp))*self._U_plus

        #2nd Level trajectory tracking:
        N2 = np.eye(7) - linalg.pinv((self._dsigma_dp.T).dot(J),rcond = 0.01).dot((self._dsigma_dp.T).dot(J))
        joint_acc_2 = linalg.pinv(J.dot(N2),rcond = 0.01).dot(self._pdd_ref + self._Kpd*(self._pd_ref - pd) + self._Kp*error - J.dot(joint_acc))

        #3rf Level self motion:
        N3 = N2.dot(np.eye(7) - np.linalg.pinv(J.dot(N2)).dot(J.dot(N2)))
        joint_acc_3 = linalg.pinv(N3,rcond = 0.01).dot(-self._Kqd*qd + self._Kq*(qo - q) - (joint_acc + joint_acc_2))

        self._joint_vel = self._joint_vel + (joint_acc + joint_acc_2 + joint_acc_3)/self._rate

        cmd = make_cmd(self._limb.joint_names(), self._joint_vel)

        # command new joint torques
        self._limb.set_joint_velocities(cmd) 
        b = ""
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(p.T)),decimals = 4)))
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(pd.T)),decimals = 4)))
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(error.T)),decimals = 4)))
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(qo.T)),decimals = 4)))
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(q.T)),decimals = 4)))
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(qd.T)),decimals = 4)))
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(joint_acc.T)),decimals = 4)))
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(joint_acc_2.T)),decimals = 4)))
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(joint_acc_3.T)),decimals = 4)))
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray((joint_acc + joint_acc_2 + joint_acc_3).T)),decimals = 4)))
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._joint_vel.T)),decimals = 4)))
        b += " " + str(np.round(self._dsigma_dp[0][0],decimals = 4)) + " " + str(np.round(self._dsigma_dp[1][1],decimals = 4))
        b += " " + str(np.round(self._dsigma_dp[2][2],decimals = 4)) + " " + str(np.round(self._dsigma_dp[3][3],decimals = 4)) 
        b += " " + str(np.round(self._dsigma_dp[4][4],decimals = 4)) + " " + str(np.round(self._dsigma_dp[5][5],decimals = 4))
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(phi_tmp.T)),decimals = 4)))

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
 
        self._done = False
        
        #Computing the reference end-effector pose
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

            self._Sensor[0] = msg.wrench.force.x
            self._Sensor[1] = -msg.wrench.force.y
            self._Sensor[2] = msg.wrench.force.z
            self._Sensor[3] = msg.wrench.torque.x*1000.0
            self._Sensor[4] = msg.wrench.torque.y*1000.0
            #self._Sensor[5] = msg.wrench.torque.z*1000.0 #We are not controlling yaw in this example (no tool spin compensation)
            
            self._sigma_max = self._Sensor - self._Sensor_ref
            self._sigma_min = -(self._Sensor + self._Sensor_ref)
                   
            self._sigma_d_max = (self._sigma_max - self._sigma_max_ant)*self._rate
            self._sigma_d_min = (self._sigma_min - self._sigma_min_ant)*self._rate

            self._phi_max = self._sigma_max + self._Ka_map.dot(self._Ka*self._sigma_d_max)
            self._phi_min = self._sigma_min + self._Ka_map.dot(self._Ka*self._sigma_d_min)

            pose = self._limb.endpoint_pose()
            XYZ = pose['position']
            RPY = Quat2rpy(pose['orientation'])

            DH = DenHartStandard(self._limb,self._limb.joint_names())
            J = jacobn_rec(jacob0(self._limb,self._limb.joint_names(),DH),RPY)
 
            b = self._SMC_acc_Cartesian(XYZ,RPY,J)           

            self._sigma_max_ant =  self._sigma_max
            self._sigma_min_ant =  self._sigma_min
            
            control_rate.sleep()

            if  self._nContact == False:
                self._time += rospy.get_time() - start

            if  self._nContact:
                self._p_ref[2] =  self._p_ref[2] -0.005/self._rate
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
                   self._cnt2 = self._cnt2 + 1;

                self._pdd_ref[0] = -self._r/(self._scale*self._scale)*np.cos(self._theta/self._scale)
                self._pdd_ref[1] = -self._r/(self._scale*self._scale)*np.sin(self._theta/self._scale)
                
                self._pd_ref[0] = -self._r/self._scale*np.sin(self._theta/self._scale)
                self._pd_ref[1] =  self._r/self._scale*np.cos(self._theta/self._scale)
                
                self._p_ref[1] = self._p_ref[1] + self._pd_ref[1]/self._rate
                self._p_ref[0] = self._p_ref[0] + self._pd_ref[0]/self._rate
                
                self._theta += self._step*self._rate

                a += "\n"+ str(np.round(float(self._time),4))
                a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._Sensor.T)),decimals = 4)))
                a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._sigma_max.T)),decimals = 4)))
                a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._sigma_min.T)),decimals = 4)))
                a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._sigma_d_max.T)),decimals = 4)))
                a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._sigma_d_min.T)),decimals = 4)))
                a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._phi_max.T)),decimals = 4)))
                a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._phi_min.T)),decimals = 4)))
                a += b
                a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._p_ref.T)),decimals = 4)))
                a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._pd_ref.T)),decimals = 4)))
                a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._pdd_ref.T)),decimals = 4)))
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
    joint_pos = np.array([0.1456845703125, -0.349845703125, -0.05283984375, 1.1399111328125, 0.068462890625, 0.7788173828125, 3.397236328125],dtype=np.float)
    js.move_to_position(joint_pos)
    rospy.sleep(1.0)
    js.Control_loop_Cart()


if __name__ == "__main__":
    main()
