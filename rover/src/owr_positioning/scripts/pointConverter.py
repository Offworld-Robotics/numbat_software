#!/usr/bin/env python
"""
Python subscriber to convert a pose from the transform frame in its header to the map frame

"""
import rospy
import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import PoseStamped

pub = None
tfBuffer = None
listener = None

def poseCallback(pose):
    try:
        trans = tfBuffer.lookup_transform(pose.header.frame_id, "map", rospy.Time(0))
        newPose = tf2_geometry_msgs.do_transform_pose(pose,trans)
        pub.publish(newPose)
        rospy.loginfo(newPose)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr("exception thrown %s", e)
        return

if __name__ == '__main__':
    rospy.init_node("point_converter")
    rospy.Subscriber("/owr/test_point_convert", PoseStamped, callback=poseCallback)
    pub = rospy.Publisher("/owr/converted_test_point", PoseStamped, latch=True, queue_size=10)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rospy.spin()


