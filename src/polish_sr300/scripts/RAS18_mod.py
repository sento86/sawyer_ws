#!/usr/bin/env python

"""
TITLE: HUMAN-ROBOT COLLABORATION FOR SAFE OBJECT TRANSPORTATION USING FORCE FEEDBACK
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

        # ROS control parameters
        self._rate = 50.0  # Hz
        self._missed_cmds = 20.0  # Missed cycles before triggering timeout

        #High level control parameters
        self._Sensor = np.zeros((6,1))
        self._sigma_max = np.zeros((6,1))
        self._sigma_min = np.zeros((6,1))
        self._SensorR = np.zeros((6,1))
        self._Sensor_ref = np.zeros((6,1))
        self._sigma_max_ant = np.zeros((6,1))
        self._sigma_min_ant = np.zeros((6,1))
        self._Forces = np.zeros((6,1))
        
        self._Sensor_ref[0] = 4.0 #Fx_max
        self._Sensor_ref[1] = 1.0 #Fy_max
        self._Sensor_ref[2] = 0.0 #Fz_ref
        self._Sensor_ref[3] = math.pi - 0.0*math.pi/180.0 #math.pi + 30.0*math.pi/180.0 # # #Roll_ref
        self._Sensor_ref[4] = 0.0 + 0.0*math.pi/180.0 #0.0 - 0.0*math.pi/180.0 # # #Pitch_ref

        self._alpha_max = 1.0*(math.pi/180.0)
        self._alpha_max_z = 2.0*(math.pi/180.0)

        self._WC = 0.0 # object weight compensation in N. Just in case you forget to compensate the sensor bias

        #1st Level control parameters
        self._Ka = 0.05
        self._U_plus = 0.035

        #2nd Level control parameters
        self._Ka_map = np.identity(6)
        self._U_plus_map = np.identity(6)
        self._U_plus_map2 = np.identity(6)

        self._U_plus_map[0][0] = 0.0
        self._U_plus_map[1][1] = 0.0
        self._U_plus_map[2][2] = 0.0
        self._U_plus_map[3][3] = 0.1
        self._U_plus_map[4][4] = 0.1
        self._U_plus_map[5][5] = 0.1

        self._U_plus_map2 = self._U_plus_map*self._U_plus

        self._Ka_map[0][0] = 0.5
        self._Ka_map[3][3] = 0.75
        self._Ka_map[4][4] = 0.75
        self._Ka_map[5][5] = 0.75

        self._FH = np.zeros((6,1))
        self._Fh = np.zeros((6,1))
        self._Fh[0] = 8.0 #10N
        self._Fh[1] = 8.0 #10N
        self._Fh[2] = 8.0 #10N
        self._Fh[3] = 0.5 #10N
        self._Fh[4] = 0.5 #10N
        self._Fh[5] = 0.5 #1Nm

        self._Vr = np.zeros((6,1))
        self._Vr[0] = 0.3 #0.1m/s
        self._Vr[1] = 0.3 #0.1m/s
        self._Vr[2] = 0.3 #0.1m/s
        self._Vr[3] = 0.5 #0.1rad/s
        self._Vr[4] = 0.5 #0.1rad/s
        self._Vr[5] = 0.5 #0.1rad/s
 
        self._Cd = np.zeros((6,6))
        self._Cd[0][0] = self._Fh[0]/self._Vr[0]
        self._Cd[1][1] = self._Fh[1]/self._Vr[1]
        self._Cd[2][2] = self._Fh[2]/self._Vr[2]
        self._Cd[3][3] = self._Fh[3]/self._Vr[3]
        self._Cd[4][4] = self._Fh[4]/self._Vr[4]
        self._Cd[5][5] = self._Fh[5]/self._Vr[5]

        self._tau = np.zeros((6,1))
        self._tau[0] = 0.25 #0.2s
        self._tau[1] = 0.25 #0.2s
        self._tau[2] = 0.25 #0.2s
        self._tau[3] = 0.25 #0.2s
        self._tau[4] = 0.25 #0.2s
        self._tau[5] = 0.25 #0.2s

        self._Md = np.zeros((6,6))
        self._Md[0][0] = self._Cd[0][0]*self._tau[0]
        self._Md[1][1] = self._Cd[1][1]*self._tau[1]
        self._Md[2][2] = self._Cd[2][2]*self._tau[2]
        self._Md[3][3] = self._Cd[3][3]*self._tau[3]
        self._Md[4][4] = self._Cd[4][4]*self._tau[4]
        self._Md[5][5] = self._Cd[5][5]*self._tau[5]

        self._u2_plus = 0.01

        #3rd Level control parameters
        self._Kq = 0.75
        self._Kqd = 6*self._Kq #0.2

        # create our limb instance
        self._limb = intera_interface.Limb(limb)
        self._joint_vel = np.zeros((7,1))
         
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

    	return limb_joints

    def _SMC_acc_Cartesian(self,XYZ,RPY,Jn,J):
        
        dsigma_dp = np.zeros((6,6))
        phi_tmp = np.zeros((6,1))
        p = np.zeros((6,1))

        #Computing the pose in Cartesian and Joint workspaces
        for i in range(0,6,1):
            if i < 3:
               p[i] = XYZ[i]
            else:
               p[i] = RPY[i-3]
        if p[3] < 0:
           p[3] = p[3] + 2.0*math.pi

        Qd = self._limb.joint_velocities()
        Q  = self._limb.joint_angles()
        q  = np.zeros((7,1))
        qo  = np.zeros((7,1))
        qd = np.zeros((7,1))
        for i,joint in enumerate(self._limb.joint_names()):
            qo[i]  = self._start_angles[joint]
            q[i] = (self._limb.joint_angles())[joint]
            qd[i] = Qd[joint]
             
        pd = J.dot(qd)
        pdn = Jn.dot(qd)

        # Level 1: R, P and Y inequalities activation
        if self._phi_max[3] > 0:       
           phi_tmp[3] = 1.0
           dsigma_dp[3][3] = 1.0
        elif self._phi_min[3] > 0:
           phi_tmp[3] = 1.0
           dsigma_dp[3][3] = -1.0
        else:
           dsigma_dp[3][3] = 0.0
           phi_tmp[3] = 0.0

        if self._phi_max[4] > 0:       
           phi_tmp[4] = 1.0
           dsigma_dp[4][4] = 1.0
        elif self._phi_min[4] > 0:
           phi_tmp[4] = 1.0
           dsigma_dp[4][4] = -1.0
        else:
           dsigma_dp[4][4] = 0.0
           phi_tmp[4] = 0.0

        # For constraining Yaw
        #if self._phi_max[5] > 0:       
         #  phi_tmp[5] = 1.0
         #  dsigma_dp[5][5] = 1.0
        #elif self._phi_min[5] > 0:
        #   phi_tmp[5] = 1.0
        #   dsigma_dp[5][5] = -1.0
        #else:
        #   phi_tmp[5] = 0.0
         #  dsigma_dp[5][5] = 0.0


        #1st Level SMC:
        joint_acc = -linalg.pinv(self._Ka*(dsigma_dp.T).dot(J),rcond = 0.01).dot(self._U_plus_map.dot(phi_tmp))*self._U_plus # Anulada por dsigma_dp = 0.0

        #2nd Level Hybrid:
        N1 = np.eye(7) - linalg.pinv((dsigma_dp.T).dot(J),rcond = 0.01).dot((dsigma_dp.T).dot(J))
        joint_acc_2 = linalg.pinv((self._Md.dot(Jn)).dot(N1),rcond = 0.01).dot(self._FH-(self._Cd.dot(Jn)).dot(qd)-self._u2_plus*np.sign((self._Cd.dot(Jn)).dot(qd)-self._FH) - (self._Md.dot(Jn)).dot(joint_acc))
        
        #3rd Level Singularity Avoidance/Home position:
        N2 = N1.dot(np.eye(7) - np.linalg.pinv(Jn.dot(N1)).dot(Jn.dot(N1)))
        joint_acc_3 = linalg.pinv(N2,rcond = 0.01).dot(-self._Kqd*qd  +  self._Kq*(qo - q) - (joint_acc + joint_acc_2))

        #Computing the velocity accion command
        self._joint_vel = self._joint_vel + (joint_acc + joint_acc_2 + joint_acc_3)/self._rate
        cmd = make_cmd(self._limb.joint_names(), self._joint_vel)

        # command new joint torques
        self._limb.set_joint_velocities(cmd)
        b = ""
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(p.T)),decimals = 4)))
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(pd.T)),decimals = 4)))
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(pdn.T)),decimals = 4)))
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(qo.T)),decimals = 4)))
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(q.T)),decimals = 4)))
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(qd.T)),decimals = 4)))
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(joint_acc.T)),decimals = 4)))
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(joint_acc_2.T)),decimals = 4)))
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(joint_acc_3.T)),decimals = 4)))
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray((joint_acc + joint_acc_2 + joint_acc_3).T)),decimals = 4)))
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._joint_vel.T)),decimals = 4)))
        b += " " + str(np.round(dsigma_dp[3][3],decimals = 4)) + " " + str(np.round(dsigma_dp[4][4],decimals = 4)) + " " + str(np.round(dsigma_dp[5][5],decimals = 4))
        for i in range(0,6,1):
            for j in range(0,7,1):
                b += " " + str(np.round(J[i,j],decimals = 4))
        for i in range(0,6,1):
            for j in range(0,7,1):
                b += " " + str(np.round(Jn[i,j],decimals = 4))
        b += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(phi_tmp.T)),decimals = 4)))

        return b

    def move_to_neutral(self,sp):
        """
        Moves the limb to neutral location.
        """
        self._limb.move_to_neutral(speed=sp)

    def Control_loop_Cart(self):
        np.set_printoptions(suppress = True)
 
        # record initial joint angles
        self._start_angles = self._limb.joint_angles()

        # set control rate
        control_rate = rospy.Rate(self._rate)

        # for safety purposes, set the control rate command timeout.
        # if the specified number of command cycles are missed, the robot
        # will timeout and return to Position Control Mode
        self._limb.set_command_timeout((1.0 / self._rate) * self._missed_cmds)
        done = False
        
        #Setting the reference value of yaw
        pose = self._limb.endpoint_pose()
        XYZ = pose['position']
        RPY = Quat2rpy(pose['orientation'])
        if RPY[0] < 0:
           RPY[0] = RPY[0] + 2.0*math.pi

        self._Sensor_ref[5] = RPY[2] #Yaw_ref

        time = 0.0   
        
        #Save data: header
        a = "Period " + str(1/self._rate)
        a += "Sensor Filter (Wc = 73 Hz)" + "\n"
        a += "Time(sec) " + "Sensor (1x6) " + "SensorR (1x6) " + "Sigma_max (1x6) " + "Sigma_min (1x6) " + "Sigma_d_max (1x6) " + "Sigma_d_min (1x6) "
        a += "phi_max (1x6) " + "phi_min (1x6) " + "p (1x6) " + "pd (1x6) "
        a += "qo(1x7) " + "q (1x7) " + "qd (1x7) " + "joint_acc Level 1 (1x7) " + "joint_acc Level 2 (1x7) "
        a += "joint_acc Level 3 (1x7) " + "joint_acc Level 4 (1x7) " + "joint_acc Total (1x7) " + "joint_vel cmd (1x7) " + "dsigma_1stLevel: alpha beta (1x2) "
        a += "dsigma_2ndLevel: Ftotal Gamma (1x4) "

        # loop at specified rate commanding new joint torques
        while not rospy.is_shutdown()  and not done:
            
            start = rospy.get_time()

            if not self._rs.state().enabled:
                rospy.logerr("Program failed to meet specified control rate timeout.")
                break

            msg = self._ig.get_msg() #Wrench info

            #WARNING: LEVEL 1 USES J_rec AND LEVEL 2 USES Jn, SO THE SENSOR DATA MUST FEET THE FRAME IN EACH CASE.
            #Sensor data for constraining roll and pitch according to the base's frame (we use J rectified)
            self._Sensor[3] = msg.wrench.torque.x
            self._Sensor[4] = msg.wrench.torque.y
            #self._Sensor[5] = - msg.wrench.torque.z #For constraining yaw

            #Sensor data for guidind the robot according to the end-effector's frame (we use Jn)
            self._FH[0] = msg.wrench.force.y 
            self._FH[1] = -msg.wrench.force.x 
            self._FH[2] = (msg.wrench.force.z - self._WC)
            self._FH[5] = msg.wrench.torque.z #Comment this line and uncomment the line 269 for constraining yaw
           
            #Rotation transformation to meet the base's frame
            pose = self._limb.endpoint_pose()
            DH = DenHartStandard(self._limb,self._limb.joint_names())
            XYZ = pose['position']
            RPY = Quat2rpy(pose['orientation'])
            if RPY[0] < 0:
               RPY[0] = RPY[0] + 2.0*math.pi
            Rot = rpy2R(RPY)
            self._SensorR[np.ix_([0,1,2],[0])] = Rot.dot(self._Sensor[np.ix_([0,1,2],[0])])
            for i in range(3,6,1):
                self._SensorR[i] = self._Sensor[i]

            #Computing Sigma_max and Sigma_min according to eq. ()            
            self._sigma_max[0] = (self._SensorR[0]*self._SensorR[0] + self._SensorR[1]*self._SensorR[1] + self._SensorR[2]*self._SensorR[2]) - self._Sensor_ref[0]
            self._sigma_max[3] = (RPY[0]-self._Sensor_ref[3]) - self._alpha_max
            self._sigma_max[4] = (RPY[1]-self._Sensor_ref[4]) - self._alpha_max
            self._sigma_max[5] = (RPY[2]-self._Sensor_ref[5]) - self._alpha_max
            #sigma_max[5] = (np.absolute(Sensor[5])-Sensor_ref[5])-alpha_max_z # For constraining yaw
            self._sigma_min[3] = -(RPY[0] - self._Sensor_ref[3]) - self._alpha_max
            self._sigma_min[4] = -(RPY[1] - self._Sensor_ref[4]) - self._alpha_max
            self._sigma_min[5] = -(RPY[2] - self._Sensor_ref[5]) - self._alpha_max # Yaw control in Level 1 for Exp 2
             
            #Computing the derivatives Sigma_max and Sigma_min  
            self._sigma_d_max = (self._sigma_max - self._sigma_max_ant)*self._rate
            self._sigma_d_min = (self._sigma_min - self._sigma_min_ant)*self._rate
            
            #Saving sigma for next iteration
            for i in range(0,6,1):
                self._sigma_max_ant[i] = self._sigma_max[i]
                self._sigma_min_ant[i] = self._sigma_min[i]

            #Computing phi_max and phi_min according to eq. ()   
            self._phi_max = self._sigma_max + self._Ka_map.dot(self._Ka*self._sigma_d_max)
            self._phi_min = self._sigma_min + self._Ka_map.dot(self._Ka*self._sigma_d_min)

            #Computing J and Jn
            Jn = jacobn(self._limb,self._limb.joint_names(),DH)
            J = jacobn_rec(jacob0(self._limb,self._limb.joint_names(),DH),RPY)
 
            #Non-conventional SM controller
            b = self._SMC_acc_Cartesian(XYZ,RPY,Jn,J)           
            
            control_rate.sleep() #Set in 20ms

            time += rospy.get_time() - start

            #Saving data:
            a += "\n"+ str(np.round(float(time),4))
            a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._Sensor.T)),decimals = 4)))
            a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._SensorR.T)),decimals = 4)))
            a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._FH.T)),decimals = 4)))
            a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._sigma_max.T)),decimals = 4)))
            a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._sigma_min.T)),decimals = 4)))
            a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._sigma_d_max.T)),decimals = 4)))
            a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._sigma_d_min.T)),decimals = 4)))
            a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._phi_max.T)),decimals = 4)))
            a += " " + ' '.join(map(str,np.around(np.squeeze(np.asarray(self._phi_min.T)),decimals = 4)))
            a += b
            self._saveData.write(a)
            a = ""

            if np.absolute(msg.wrench.force.z) > 40 or np.absolute(msg.wrench.force.x) > 40 or np.absolute(msg.wrench.force.y) > 40:
         	done = True


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
    js.move_to_neutral(0.05)
    rospy.sleep(5.0)
    js.Control_loop_Cart()


if __name__ == "__main__":
    main()
