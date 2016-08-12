#!/usr/bin/env python
"""
Python subscriber to handle a controlled lidar tilt

"""
import rospy

from std_msgs.msg import Float64, Int16

LOOP_RATE_HZ = 10.0

if __name__ == '__main__':
    pub = rospy.Publisher('/laser_tilt_joint_controller/command', Float64, queue_size=1)
    pubMode = rospy.Publisher("/owr/lidar_gimble_mode", Int16, queue_size=1, latch=True)
    rospy.init_node('lidar_sweep')
    rate = rospy.Rate(LOOP_RATE_HZ) # no point doing more than 10hz, as the board dosen't update that fast

    rangeMin = rospy.get_param("/lidar_sweep/min", 0.0)
    rangeMax = rospy.get_param("/lidar_sweep/max", 100.785398) # default is 45 deg
    inc = rospy.get_param("/lidar_sweep/increment_second", 0.009) # defualt is 0.5deg/sec
    inc = inc/LOOP_RATE_HZ
    pos = rangeMin
    direction = 1
    # spin
    while not rospy.is_shutdown():
        pubMode.publish(2)
        pos += inc * direction
        if pos > rangeMax:
            direction = -1
            pos = rangeMax
        elif pos < rangeMin:
            direction = 1
            pos = rangeMax

        pub.publish(pos)
        rate.sleep()
