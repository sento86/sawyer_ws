#!/usr/bin/env python

"""
Title: FK_IK_Sample
Author: Luke Ramos
Year: 2018

Description:
Used as an exercise to practice using IK in the context of the Sawyer robot.
I will practice subscribing to joint states in order to perform FK.
Similarly, I will practice subscribing to end point state to perform IK.
Using IK, I will input a point in 3D space in order to determine an appropriate
pose for sawyer, a 7dof manipulator arm.
If this program works as intended, it will also be able to move to the desired pose, if appropriate

sdk.rethinkrobotics.com/intera/API_Reference

"""
import argparse
import importlib

import rospy
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import WrenchStamped
from utilities import *

from geometry_msgs.msg import (
	PoseStamped,
	Pose,
	Point,
	Quaternion,
)

from std_msgs.msg import (Header, String, Float64, Empty)
from sensor_msgs.msg import JointState
from operator import itemgetter
import numpy as np

import intera_interface
from intera_interface import CHECK_VERSION

from intera_core_msgs.srv import (
	SolvePositionIK,
	SolvePositionIKRequest,
#	SolvePositionFK,
#	SolvePositionFKRequest,
)

class IK_sample(object):

	def __init__(self, reconfig_server, limb = "right"):
		self._dyn = reconfig_server

		# control parameters
		self._rate = 1000.0  # Hz
		self._missed_cmds = 20.0  # Missed cycles before triggering timeout

		# create our limb instance
		self._limb = intera_interface.Limb(limb)

		# initialize parameters
		self._springs = dict()
		self._damping = dict()
		self._start_angles = dict()

		# create cuff disable publisher
		cuff_ns = 'robot/limb/' + limb + '/suppress_cuff_interaction'
		self._pub_cuff_disable = rospy.Publisher(cuff_ns, Empty, queue_size=1)

		# verify robot is enabled
		print("Getting robot state... ")
		self._rs = intera_interface.RobotEnable(CHECK_VERSION)
		self._init_state = self._rs.state().enabled
		print("Enabling robot... ")
		self._rs.enable()
		print("Running. Ctrl-c to quit")

	def _update_parameters(self):
		for joint in self._limb.joint_names():
			self._springs[joint] = self._dyn.config[joint[-2:] + '_spring_stiffness']
			self._damping[joint] = self._dyn.config[joint[-2:] + '_damping_coefficient']

	def _update_forces(self):
		"""
		Calculates the current angular difference between the start position
		and the current joint positions applying the joint torque spring forces
		as defined on the dynamic reconfigure server.
		"""
		# get latest spring constants
		self._update_parameters()

		# disable cuff interaction
		self._pub_cuff_disable.publish()

		# create our command dict
		cmd = dict()
		# record current angles/velocities
		cur_pos = self._limb.joint_angles()
		cur_vel = self._limb.joint_velocities()
		# calculate current forces
		for joint in self._start_angles.keys():
			# spring portion
			cmd[joint] = self._springs[joint] * (self._start_angles[joint] -
													cur_pos[joint])
			# damping portion
			cmd[joint] -= self._damping[joint] * cur_vel[joint]
		# command new joint torques
		self._limb.set_joint_torques(cmd)

	def move_to_neutral(self):
		"""
		Moves the limb to neutral location.
		"""
		self._limb.move_to_neutral()
		
	def move_to_position(self, joint_pos):
		cmd = make_cmd(self._limb.joint_names(), joint_pos)
		self._limb.set_joint_position_speed(0.05)
		self._limb.move_to_joint_positions(cmd,timeout=20.0)
		self._limb.set_joint_position_speed(0.3) #speed back to default

	def clean_shutdown(self):
		"""
		Switches out of joint torque mode to exit cleanly
		"""
		print("\nExiting example...")
		self._limb.exit_control_mode()
	

	"""
	Start Ramos
	"""
	def ik_service_client(self, limb = "right"):
#	def ik_service_client(limb = "right"):
#		ns = "Externaltools/right/PositionKinematicsNode/IKService"
		ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"

		iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
		ikreq = SolvePositionIKRequest()

		print("Performing Inverse Kinematics")

		hdr = Header(stamp=rospy.Time.now(), frame_id='base')
                poses = {'right': PoseStamped(
                                                header = hdr,
                                                pose = Pose(
                                                position = Point(
                                                                x = 0.650628752997,
                                                                y = 0,
                                                                z = 0.417447307078,
                                                        ),
                                                orientation = Quaternion(
                                                                x = 0.704020578925,
                                                                y = 0.710172716916,
                                                                z = 0.00244101361829,
                                                                w = 0.00194372088834,
                                                        ),
                                                ),
                                        ),
                                }

		print("poses checkpoint")

                print(poses)

		ikreq.pose_stamp.append(poses[limb])
		ikreq.tip_names.append('right_hand')

		print("ikreq checkpoint")
		print("using simple IK Service Client from ik_service_client example")

		try:
			print("trying resp")
			rospy.wait_for_service(ns, 5.0)
			resp = iksvc(ikreq)
			print("resp successful")
		except (rospy.ServiceException, rospy.ROSException), e:
			rospy.logerr("Service call failed: %s" % (e,))
			print("resp didnt work")
			return False

			print("resp checkpoint")

		if (resp.result_type[0] > 0):
			seed_str = {
						ikreq.SEED_USER: 'User Provided Seed',
						ikreq.SEED_CURRENT: 'Current Joint Angles',
						ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
						}.get(resp.result_type[0], 'None')
			rospy.loginfo("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %(seed_str,))
			# Format solution into Limb API-compatible dictionary
			limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
			rospy.loginfo("\nIK Joint Solution:\n%s", limb_joints)
			rospy.loginfo("------------------")
			rospy.loginfo("Response Message:\n%s", resp)
		else:
			rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
			rospy.logerr("Result Error %d", resp.result_type[0])
			return False
		
		print "test for right_j0", resp.joints[0].position[0]

		global sub_right_j0
		global sub_right_j1
		global sub_right_j2
		global sub_right_j3
		global sub_right_j4
		global sub_right_j5
		global sub_right_j6

		sub_right_j0 = resp.joints[0].position[0]
		sub_right_j1 = resp.joints[0].position[1]
		sub_right_j2 = resp.joints[0].position[2]
		sub_right_j3 = resp.joints[0].position[3]
		sub_right_j4 = resp.joints[0].position[4]
		sub_right_j5 = resp.joints[0].position[5]
		sub_right_j6 = resp.joints[0].position[6]
		
		
#		joint_pos = np.array([sub_right_j0, sub_right_j1, sub_right_j2, sub_right_j3, sub_right_j4, sub_right_j5, sub_right_j6],dtype=np.float)
#		print(sub_right_j0)
#		print(resp.joints[0].position[0])
#		print "joint_pos = ", joint_pos
		return True
		
def main():
#	IK = IK_sample(dynamic_cfg_srv, limb=args.limb)
#	IK = IK_sample()

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
	IK = IK_sample(dynamic_cfg_srv, limb=args.limb)
	# register shutdown callback
	rospy.on_shutdown(IK.clean_shutdown)
	
#	rospy.init_node("rsdk_ik_service_client")

	if IK.ik_service_client():
		rospy.loginfo("Simple IK call passed!")
		print "Moving to desired location"
		joint_pos = np.array([sub_right_j0, sub_right_j1, sub_right_j2, sub_right_j3, sub_right_j4, sub_right_j5, sub_right_j6],dtype=np.float)
#		joint_pos = np.array([resp.joints[0].position[0], sub_right_j1, sub_right_j2, sub_right_j3, sub_right_j4, sub_right_j5, sub_right_j6],dtype=np.float)
		print(joint_pos)
		IK.move_to_position(joint_pos)
	else:
		rospy.logerr("Simple IK call FAILED")

#	joint_pos = np.array([-0.00615625, -0.6968154296875, 0.0101787109375, 2.0974033203125, -0.0526416015625, 0.1697822265625, 3.3739580078125],dtype=np.float)
#	joint_pos = np.array([sub_right_j0, sub_right_j1, sub_right_j2, sub_right_j3, sub_right_j4, sub_right_j5, sub_right_j6],dtype=np.float)

#	print(IK.ik_service_client.resp)	
#	print(joint_pos)
		
#	IK.move_to_position(joint_pos)
#	print("Moving to desired position")

if __name__ == '__main__':
#	initialise()
	main()
