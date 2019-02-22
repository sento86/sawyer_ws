#!/usr/bin/env python
import roslib;
import rospy
import tf

if __name__ == '__main__':
  rospy.init_node("test_tf_listener")
  listener = tf.TransformListener()
  while not rospy.is_shutdown():
    try:
      now = rospy.Time.now()
      #listener.waitForTransform('/base', '/depth_link', now, rospy.Duration(1))
      #(trans,rot) = listener.lookupTransform("/base", "/depth_link", now)
      listener.waitForTransform('/base', '/depth_link', now, rospy.Duration(1))
      (trans,rot) = listener.lookupTransform("/base", "/depth_link", now)
      print trans, rot
    except (tf.LookupException, tf.ConnectivityException):
      print "Something went wrong"
