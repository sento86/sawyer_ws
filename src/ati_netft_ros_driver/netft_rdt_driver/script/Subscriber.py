#!/usr/bin/env python
import roslib; roslib.load_manifest('geometry_msgs') 
import rospy

from geometry_msgs.msg import WrenchStamped
    
def callback(msg):
    print "callback"
    print msg
   
if __name__ == '__main__':
    rospy.init_node("listener", anonymous=True)
    print "Ini listener"
    rospy.Subscriber("netft_data", WrenchStamped, callback)
    rospy.spin()
