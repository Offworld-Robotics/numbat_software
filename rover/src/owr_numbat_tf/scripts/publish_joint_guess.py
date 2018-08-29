#!/usr/bin/env python
"""
Python subscriber to guess the position of joints based on their mode

"""
import rospy

from std_msgs.msg import Float64, Int16
from sensor_msgs.msg import JointState

LOOP_RATE_HZ = 10.0

pub = None
sub = None

def joint_callback(msg):
    """
    Handles the joint control command callback

    @param msg: the message
    """
    out = JointState()
    out.header.time = rospy.get_rostime()

    pass

if __name__ == '__main__':
    rospy.init_node('joint_guesser')
    sub  = rospy.Subscriber('/input/command', Float64, joint_callback, queue_size=1)
    pub = rospy.Publisher("/joint_states", JointState)

    joint_name = rospy.get_param("/joint_guesser/joint_name", None)
    joint_delay_ns = rospy.get_param("/joint_guesser/joint_delay_ns", 0.0)
    joint_delay = rospy.Duration.from_nsec(joint_delay_ns)


    rospy.spin();
