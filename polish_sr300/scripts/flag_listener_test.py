#!/usr/bin/env python

import rospy
import argparse
import importlib
import numpy as np
import roslaunch
from std_msgs.msg import (Header, String, Float64, Empty)
from operator import itemgetter
from roslib import message

class flag_listen(object):
    def __init__(self):
        print("init checkpoint")
        rospy.init_node('flag_listener', anonymous=True)
        # subscriber to flag
        self.flagSubscriber = rospy.Subscriber("/flag_topic", String, self.flag_callback)
        self.flag = False
#        rospy.spin()

    def flag_callback(self, msg):
#        rospy.loginfo("flag received")
        print(msg)
        if msg.data == str("True"):
            rospy.loginfo("flag received")
            print("hello world")
            self.flag = True
            if self.flag == True:
                print("hello again")
                # insert TCST17_LR.py call here
            else:
                print("goodbye again")
#                rospy.spin()
        else:
            rospy.logerr("flag not received")
            print("goodbye world")
#            rospy.spin()

#    def flag_result(self):
#        if self.flag == True:
#            print("hello again")
#        else:
#            print("goodbye again")
#            rospy.spin()

#def callback(msg):
#    print(msg)
#    if msg.data == str("True"):
#        print("hello world")
#    else:
#        print("goodbye world")

def main():
#    placeholder
    print("placeholder")
    FC = flag_listen()
#    FC.flag_result()
    rospy.spin()

#    rospy.init_node('flag_listener', anonymous=True)
#    rospy.Subscriber("/flag_topic", String, callback)

if __name__ == '__main__':
#	initialise()
    main()

# taken from http://wiki.ros.org/roslaunch/API%20Usage
#    package = 'polish_sr300'
#    exectuable = 'TCST17_LR.py'
#    node = roslaunch.core.Node(package, executable)

#    launch = roslaunch.scriptapi.ROSLaunch()
#    launch.start()

#    process = launch.launch(node)
#    print process.is_alive()
#    process.stop()
