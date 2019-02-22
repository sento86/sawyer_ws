#!/usr/bin/env python

"""
Title: surface_follow.py
Author: Tarek Taha
Year: 12-06-2018

"""
import rospy
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import argparse
import importlib
from roslib import message
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import PoseStamped
from utilities import *
import geometry_msgs.msg
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import (Header, String, Float64, Empty)
from sensor_msgs.msg import (JointState, PointCloud2)
from operator import itemgetter
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import tf
from tf import (TransformerROS, TransformListener)
import tf2_ros
import intera_interface
from intera_interface import CHECK_VERSION
import tf2_ros
import tf2_geometry_msgs
from visualization_msgs.msg import Marker

from intera_core_msgs.srv import (
	SolvePositionIK,
	SolvePositionIKRequest,
        SolvePositionFK,
        SolvePositionFKRequest,
)

class SurfaceFollow(object):
	def __init__(self, reconfig_server, limb = "right"):
		self._dyn = reconfig_server

                # create our limb instance
		self._limb = intera_interface.Limb(limb)

		# verify robot is enabled
		print("Getting robot state... ")
		self._rs = intera_interface.RobotEnable(CHECK_VERSION)
		self._init_state = self._rs.state().enabled
		print("Enabling robot... ")
		self._rs.enable()

                self._performedMotion = False
                self.marker = Marker()

                self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
                self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

		print("Running. Ctrl-c to quit")
		
                self.polishTouchPointSub = rospy.Subscriber("/polishTouchPose", PoseStamped, self.pose_callback)
                self.markerPub           = rospy.Publisher("/polishTouchGotoPose", Marker)
                self.flagPub             = rospy.Publisher("/flag_topic", String)
                rospy.on_shutdown(self.clean_shutdown)

                rate = rospy.Rate(100)
                while not rospy.is_shutdown():
                    self.flagPub.publish(str(self._performedMotion))
                    if self._performedMotion:
                        self.markerPub.publish(self.marker)
                    rate.sleep()

        def pose_callback(self, msg):
            rospy.loginfo("Received at goal message!")
            # Copying for simplicity
            position = msg.pose.position
            quat     = msg.pose.orientation
            rospy.loginfo("Point Position: [ %f, %f, %f ]"%(position.x, position.y, position.z))
            rospy.loginfo("Quat Orientation: [ %f, %f, %f, %f]"%(quat.x, quat.y, quat.z, quat.w))
            transform = self.tf_buffer.lookup_transform('base',
                                                   msg.header.frame_id, #source frame
                                                   rospy.Time(0), #get the tf at first available time
                                                   rospy.Duration(1.0)) #wait for 1 second

            pose_transformed = tf2_geometry_msgs.do_transform_pose(msg, transform)

            self.marker.header.frame_id = pose_transformed.header.frame_id
            self.marker.type    = self.marker.ARROW
            self.marker.action  = self.marker.ADD
            self.marker.scale.x = 0.25
            self.marker.scale.y = 0.03
            self.marker.scale.z = 0.03
            self.marker.color.a = 1.0
            self.marker.color.r = 1.0
            self.marker.color.g = 1.0
            self.marker.color.b = 0.0
            self.marker.pose.orientation = pose_transformed.pose.orientation
            self.marker.pose.position    = pose_transformed.pose.position
            self.marker.id = 0

            hdr = msg.header
            poses = {'right': pose_transformed,}

#            self.flagPub.publish(str(self._performedMotion))

            if not self._performedMotion:
                self._performedMotion = True;
                succ, resp = self.ik_solver(poses)
                if succ:
                        rospy.loginfo("Simple IK call passed!")
                        print "Moving to joint location"
                        joint_pos = np.array(resp.joints[0].position)
                        self.move_to_joint_position(joint_pos)
                else:
                        rospy.logerr("Simple IK call FAILED")

#            print type(self._performedMotion)
#            self.flagPub.publish(self._performedMotion)
#            self.flagPub.publish(str(self._performedMotion))

        def move_to_neutral(self):
		"""
		Moves the limb to neutral location.
		"""
		self._limb.move_to_neutral()
		
        def move_to_joint_position(self, joint_pos):
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
	

        def ik_solver(self, poses, limb = "right"):
		ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
		iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
		ikreq = SolvePositionIKRequest()

		print("Performing Inverse Kinematics")

		ikreq.pose_stamp.append(poses[limb])
		ikreq.tip_names.append('right_hand')

                try:
                        print("Calling IK Service")
			rospy.wait_for_service(ns, 5.0)
			resp = iksvc(ikreq)
		except (rospy.ServiceException, rospy.ROSException), e:
			rospy.logerr("Service call failed: %s" % (e,))
                        return False,resp

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
                        return False,resp

                return True,resp

def main():
	# Querying the parameter server to determine Robot model and limb name(s)
	rp = intera_interface.RobotParams()
	valid_limbs = rp.get_limb_names()
	if not valid_limbs:
		rp.log_message(("Cannot detect any limb parameters on this robot. "
						"Exiting."), "ERROR")
	robot_name = intera_interface.RobotParams().get_robot_name().lower().capitalize()

        print "Robot name:",robot_name, " Valid Limbs:",valid_limbs

        # Grabbing Robot-specific parameters for Dynamic Reconfigure
        config_name = ''.join([robot_name,"JointSpringsExampleConfig"])
        config_module = "intera_examples.cfg"
        moduleStr = '.'.join([config_module,config_name])
        print "Module to import:", moduleStr
        cfg = importlib.import_module('.'.join([config_module,config_name]))

        # Starting node connection to ROS
        print("Initializing node... ")
        rospy.init_node("rsdk_ik_service_client")

	dynamic_cfg_srv = Server(cfg, lambda config, level: config)
        IK = SurfaceFollow(dynamic_cfg_srv, limb=valid_limbs[0])

if __name__ == '__main__':
#	initialise()
	main()
