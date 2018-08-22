#!/usr/bin/env python

import rospy
from owr_messages.msg import science

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " I got this data %s", data.data)

def listener():
    rospy.init_node('science_module_listener', anonymous=True)
    rospy.Subscriber('science/data', science, callback=callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
