#!/usr/bin/env python
"""
Python subscriber to convert a magnetic field vector to a odometry heading

"""
import rospy
import tf2_ros
import tf2_geometry_msgs


from geometry_msgs.msg import PoseStamped, Vector3Stamped, PoseWithCovarianceStamped, Quaternion

pub = None
tfBuffer = None
listener = None

def poseCallback(mag):
    pose = PoseWithCovarianceStamped()
    pose.header = mag.header
    pose.pose.orientation = Quaternion(mag.x, mag.y, mag.z)
    # TODO: https://cdn-shop.adafruit.com/datasheets/AN203_Compass_Heading_Using_Magnetometers.pdf
    pub.publish(pose)

if __name__ == '__main__':
    rospy.init_node("point_converter")
    rospy.Subscriber("/mti/sensor/magnetic", Vector3Stamped, callback=poseCallback)
    pub = rospy.Publisher("/owr/sensors/mag", PoseWithCovarianceStamped, latch=True, queue_size=10)


    rospy.spin()
