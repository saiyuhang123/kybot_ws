#!/usr/bin/env python
 
import rospy
from std_msgs.msg import String
 
def callback(msg):
    print(msg.data)
 
rospy.init_node('description_listener')
rospy.Subscriber("/object_position", String, callback)
rospy.spin()
