#!/usr/bin/env python

import rospy
from owr_messages.msg import science

def talker():
    pub = rospy.Publisher('science/request', science, queue_size=10)
    rospy.init_node('science_module_publisher', anonymous=True)
    rate = rospy.Rate(10) # Hz
    while not rospy.is_shutdown():
        science_msg = science()
        science_msg.temperature = 23
        science_msg.humidity = 56
        science_msg.mag_x = 12.67
        science_msg.mag_y = 45.78
        science_msg.mag_z = 133.89
        science_msg.colour_temperature = 12
        science_msg.illuminance = 89
        rospy.loginfo(science_msg)
        pub.publish(science_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
