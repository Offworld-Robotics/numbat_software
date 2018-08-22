#!/usr/bin/env python

import rospy
from owr_messages.msg import science

def talker():
    pub = rospy.Publisher('science/request', science, queue_size=10)
    rospy.init_node('science_module_publisher', anonymous=True)
    rate = rospy.Rate(10) # Hz
    while not rospy.is_shutdown():
        my_msg = 'Hello world %s' % (rospy.get_time())
        rospy.loginfo(my_msg)
        pub.publish(my_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
