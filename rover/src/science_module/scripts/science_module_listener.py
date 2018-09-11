#!/usr/bin/env python

"""
Processes data from science module topics
@author: (Original Author) Sajid Ibne Anower
@authors: (Editors)
Purpose: Subscriber for science module
@copyright: This code is released under the MIT [GPL for embeded] License.
Copyright BLUEsat UNSW, 2018
"""

import rospy
from owr_messages.msg import science

# This is where the processing would happen.
# Awaiting further specifications on exactly what should happen here
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " I got this data %s", data)

def listener():
    rospy.init_node('science_module_listener', anonymous=True)
    rospy.Subscriber('science/data', science, callback=callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
