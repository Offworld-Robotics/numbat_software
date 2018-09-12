#!/usr/bin/env python

"""
Publishes data to science module topics
@author: (Original Author) Sajid Ibne Anower
@authors: (Editors)
Purpose: Publisher for science module
@copyright: This code is released under the MIT License.
Copyright BLUEsat UNSW, 2018
"""

import rospy
from owr_messages.msg import science

def talker():
    pub = rospy.Publisher('/science/request', int, queue_size=10)
    rospy.init_node('science_module_publisher', anonymous=True)
    rate = rospy.Rate(10) # Hz
    while not rospy.is_shutdown():
        # dummy
        science_msg = 23
        rospy.loginfo(science_msg)
        pub.publish(science_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
