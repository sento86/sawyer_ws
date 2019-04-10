#!/usr/bin/env python

"""
TITLE: FORCE FEEDBACK SLIDING MODE CONTROL FOR ROBOT SURFACE TREATMENT
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

    def __init__(self, reconfig_server, limb = "right"):
        self._dyn = reconfig_server

        # ROS control parameters
        self._rate = 50.0  # Hz
        self._missed_cmds = 20.0  # Missed cycles before triggering timeout
        self._done = False
       
        self._Sensor = np.zeros((6,1))
        self._Sensor_ref = np.zeros((6,1))
        self._sigma_max_ant = np.zeros((6,1))
        self._sigma_min_ant = np.zeros((6,1))
        self._dsigma_dp = np.zeros((6,6))
	self._temp_int = np.zeros((6,1))
        self._sm_ant = np.zeros((6,1))
        self._Ka_map = np.identity(6)
        self._U_plus_map = np.identity(6)
        

        self._U_plus_map[0][0] = 0.0 #6.75 #10 chattering free
        self._U_plus_map[1][1] = 0.0 #6.75 #10 chattering free
        #self._U_plus_map[2][2] = 0.14 #0.35 before - ASG
        self._U_plus_map[2][2] = 0.35
        self._U_plus_map[3][3] = 25.0#12.0 #14.0 before
        self._U_plus_map[4][4] = 25.0#12.0 #14.0 before
        self._U_plus_map[5][5] = 0.0
      
        self._Ka_map[0][0] = 1.0
        self._Ka_map[3][3] = 0.9
        self._Ka_map[4][4] = 0.9

        self._Sensor_ref[0] = 18.0*18.0
        self._Sensor_ref[1] = 18.0*18.0
        self._Sensor_ref[2] = 10.0 #Con Jn 15; #Con Jrec -15.0

        #1st Level control parameters
        self._Ka = 0.15
        self._U_plus = 0.06
        self._U_plus_map2 = self._U_plus_map*self._U_plus

        #2nd Level control parameters
        self._Kp = 5.0
        self._Kpd = np.sqrt(4.0)*1.5
        self._Kpd_L3 = 0.01
        self._p_ref = np.zeros((6,1))
        self._pd_ref = np.zeros((6,1))
        self._pdd_ref = np.zeros((6,1))

        #3rd Level control parameters
        self._Kq = 1.5
        self._Kqd = 0.15

        self._cnt = 0
        self._cnt2 = 2
        self._nContact = True
# 
#         self._time = 0.0
#         self._step = 0.0005
#         self._theta = 0.0
#         self._r = 0.1
#         self._scale = 2.0
        
        self._time = 0.0
        self._step = 0.0
        self._theta = 0.0
        self._r = 0.0
        self._scale = 2.0

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

        print("Subscribing to FT sensor node... ")
        self._ig = ForceSensorClass()
#        self._sub_netft = rospy.Subscriber("netft_data_192_168_1_11", WrenchStamped, self._ig)
        self._sub_netft = rospy.Subscriber("/right/ft_sensor_topic", WrenchStamped, self._ig)
        print "Suscribed to /right/ft_sensor_topic"

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


    def _SMC_acc_Cartesian(self,XYZ,RPY,J,Jn):

        #Level 1: Equality
        phi_tmp = np.sign(self._phi_max)
        phi_tmp[0] = 1

        # Level 1: X, Y inequalities activation
        if self._phi_max[0] > 0: 
           phi_tmp[0] = 1.0
           self._dsigma_dp[0][0] = (2.0*self._Sensor[0])/(2.0*np.sqrt(self._Sensor[0]*self._Sensor[0] + self._Sensor[1]*self._Sensor[1]))
           self._dsigma_dp[1][0] = (2.0*self._Sensor[1])/(2.0*np.sqrt(self._Sensor[0]*self._Sensor[0] + self._Sensor[1]*self._Sensor[1]))
        else:
           phi_tmp[0] = 0.0
           self._dsigma_dp[0][0] = 0.0
           self._dsigma_dp[1][0] = 0.0

        #self._dsigma_dp[0][0] = 0.0
        #self._dsigma_dp[1][0] = 0.0

        if self._nContact == True:
           self._dsigma_dp[0][0] = 0.0
           self._dsigma_dp[1][0] = 0.0
           self._dsigma_dp[2][2] = 0.0
           self._dsigma_dp[3][3] = 0.0
           self._dsigma_dp[4][4] = 0.0

       
        #1st Level SMC:
        #joint_acc = -linalg.pinv(Ka*(dsigma_dp.T).dot(J),rcond = 0.01).dot(U_plus_map2.dot(phi_tmp))
        joint_acc = -linalg.pinv(self._Ka*(self._dsigma_dp.T).dot(Jn),rcond = 0.01).dot(self._U_plus_map2.dot(phi_tmp))

        #2nd Level trajectory tracking:

        p = np.zeros((6,1))
        Qd = self._limb.joint_velocities()
        Q  = self._limb.joint_angles()
        q  = np.zeros((7,1))
        qo  = np.zeros((7,1))
        qd = np.zeros((7,1))
        u_joints = np.zeros((7,1))
 
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

        #N2 = np.eye(7) - linalg.pinv((dsigma_dp.T).dot(J),rcond = 0.01).dot((dsigma_dp.T).dot(J))
        N2 = np.eye(7) - linalg.pinv((self._dsigma_dp.T).dot(Jn),rcond = 0.01).dot((self._dsigma_dp.T).dot(Jn))

        #Saturation of the feedforward error
        error = np.zeros((6,1))

        for i in range(0,6):
           error[i] = self._p_ref[i] - p[i]

        #Error saturation by direction
        m_e = np.sqrt(error[0]*error[0] + error[1]*error[1] + error[2]*error[2])
        
        if m_e > 0.025:
           for i in range(0,3):
               error[i] = error[i]/m_e*0.025

        derror=(self._pd_ref - pd)

        if self._nContact == False:
           self._pdd_ref[2] = 0.0
           self._pdd_ref[3] = 0.0
           self._pdd_ref[4] = 0.0
           #self._pdd_ref[5] = 0.0
       
           derror[2] = 0.0
           derror[3] = 0.0
           derror[4] = 0.0
           #derror[5] = 0.0

           error[2] = 0.0
           error[3] = 0.0
           error[4] = 0.0
           #error[5] = 0.0
           #print nContact
        
        joint_acc_2 = linalg.pinv(J.dot(N2),rcond = 0.01).dot(self._pdd_ref + self._Kpd*derror + self._Kp*error + np.sign(derror + self._Kp/self._Kpd*error)*self._Kpd_L3 - J.dot(joint_acc))

        #3rf Level self motion:

        N3 = N2.dot(np.eye(7) - np.linalg.pinv(J.dot(N2),rcond = 0.01).dot(J.dot(N2)))

        joint_acc_3 = linalg.pinv(N3,rcond = 0.01).dot(-self._Kqd*qd + self._Kq*(qo - q) - (joint_acc + joint_acc_2))

        self._joint_vel = self._joint_vel + (joint_acc + joint_acc_2 + joint_acc_3)/self._rate

        #cmd = make_cmd(self._limb.joint_names(), self._joint_vel)

        # command new joint torques
        #self._limb.set_joint_velocities(cmd)
        
        for i,joint in enumerate(self._limb.joint_names()):
            u_joints[i] = q[i] + self._joint_vel[i]/self._rate
        
        cmd = make_cmd(self._limb.joint_names(), u_joints)

         ## APPLY POSITION CONTROL ACTION
        self._limb.set_joint_positions(cmd)



        
        
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
        b += " " + str(np.round(self._U_plus_map2[2][2],decimals = 4)) + " " + str(np.round(self._U_plus_map2[3][3],decimals = 4)) + " " + str(np.round(self._U_plus_map2[4][4],decimals = 4))

        return b


    def move_to_neutral(self):
        """
        Moves the limb to neutral location.
        """
        self._limb.move_to_neutral(speed=0.1)

    def move_to_position(self, joint_pos):
        cmd = make_cmd(self._limb.joint_names(), joint_pos)
        self._limb.set_joint_position_speed(0.05)
        self._limb.move_to_joint_positions(cmd,timeout=3.0)
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

        self._p_ref_Z_o = XYZ[2]

        self._pd_ref[2] = -0.015

        if self._p_ref[3] < 0:
           self._p_ref[3] = self._p_ref[3] + 2.0*math.pi

        #Save data: header
        a = "Period " + str(1/self._rate)
        a += "Sensor Filter (Wc = 73 Hz)" + "\n"
        a += "Time(sec) " + "Sensor (1x6) " + "SensorR (1x6) " + "Sigma_max (1x6) " + "Sigma_min (1x6) " + "Sigma_d_max (1x6) " + "Sigma_d_min (1x6) "
        a += "phi_max (1x6) " + "phi_min (1x6) " + "p (1x6) " + "pd (1x6) "
        a += "qo(1x7) " + "q (1x7) " + "qd (1x7) " + "joint_acc Level 1 (1x7) " + "joint_acc Level 2 (1x7) "
        a += "joint_acc Level 3 (1x7) " + "joint_acc Level 4 (1x7) " + "joint_acc Total (1x7) " + "joint_vel cmd (1x7) " + "dsigma_1stLevel: alpha beta (1x2) "
        a += "dsigma_2ndLevel: Ftotal Gamma (1x4) "


        # loop at specified rate commanding new joint torques
        while not rospy.is_shutdown()  and not self._done:
            
            start = rospy.get_time()

            if not self._rs.state().enabled:
                rospy.logerr("Joint torque example failed to meet "
                             "specified control rate timeout.")
                break

            msg = self._ig.get_msg() #Wrench info

            #self._Sensor[0] = -msg.wrench.force.y 
            #self._Sensor[1] = msg.wrench.force.x 
            #self._Sensor[2] = -msg.wrench.force.z
            #self._Sensor[4] = msg.wrench.torque.x
            #self._Sensor[3] = -msg.wrench.torque.y
            
            self._Sensor[0] = -msg.wrench.force.y 
            self._Sensor[1] = msg.wrench.force.x 
            self._Sensor[2] = -msg.wrench.force.z
            self._Sensor[3] = msg.wrench.torque.x
            self._Sensor[4] = msg.wrench.torque.y
            
            
            self._sigma_max = self._Sensor - self._Sensor_ref
            self._sigma_max[0] = (self._Sensor[0]*self._Sensor[0] + self._Sensor[1]*self._Sensor[1]) - self._Sensor_ref[0]            
                   
            self._sigma_d_max = (self._sigma_max - self._sigma_max_ant)*self._rate

            self._phi_max = self._sigma_max + self._Ka_map.dot(self._Ka*self._sigma_d_max)

            pose = self._limb.endpoint_pose()
            RPY = Quat2rpy(pose['orientation'])
            XYZ = pose['position']

            DH = DenHartStandard(self._limb,self._limb.joint_names())
            Jn = jacobn(self._limb,self._limb.joint_names(),DH)
            J = jacobn_rec(jacob0(self._limb,self._limb.joint_names(),DH),RPY)
 
            b = self._SMC_acc_Cartesian(XYZ,RPY,J,Jn)           

            self._sigma_max_ant = self._sigma_max
            
            control_rate.sleep()

            if self._nContact == False:
               self._time += rospy.get_time() - start

            if self._nContact:
                self._p_ref[2] = self._p_ref[2] -0.005/self._rate

		if np.abs(self._Sensor[2]) > np.abs(self._Sensor_ref[2]):
                   self._dsigma_dp[2][2] = 2.0
                   self._dsigma_dp[3][3] = 15.0
                   self._dsigma_dp[4][4] = 15.0
                   self._pd_ref[2] = 0.0
                   self._p_ref[2] = self._p_ref_Z_o
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
                a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._sigma_d_max.T)),decimals = 4)))
                a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._phi_max.T)),decimals = 4)))
                a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._p_ref.T)),decimals = 4)))
                a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._pd_ref.T)),decimals = 4)))
                a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._pdd_ref.T)),decimals = 4)))
                a += b
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
    #joint_pos = np.array([-0.006259765625, -1.1291142578125, 0.0050166015625, 1.9478134765625, 0.0013515625, 0.7445380859375, 3.3209453125],dtype=np.float)
    joint_pos = np.array([-0.00615625, -0.6968154296875, 0.0101787109375, 2.0974033203125, -0.0526416015625, 0.1697822265625, 3.3739580078125],dtype=np.float)
    js.move_to_position(joint_pos)
    rospy.sleep(1.0)
    js.Control_loop_Cart()


if __name__ == "__main__":
    main()

