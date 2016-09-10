#!/usr/bin/env python

# Python 2/3 compatibility
from __future__ import print_function

import rospy
import roslib
import numpy as np
import cv2
import math
import sys

from common import draw_str
from time import clock

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import Twist

def angle_between(a,b):
    return np.arctan2(b[1], b[0]) - np.arctan2(a[1], a[0])

prev_gray = None
translation_current = np.matrix('0;0');
scale_x = 1
scale_y = 1
rotation = 0
bridge = None
pub = None

prev_time = None

pixels_per_metre_traversed = 2310

linear_velocity_scale = 1.0/pixels_per_metre_traversed
rotation_velocity_scale = 1.0

def image_callback(image):

    global prev_gray
    global translation_current
    global scale_x
    global scale_y
    global rotation
    global bridge
    global pub
    global prev_time
    global linear_velocity_scale
    global rotation_velocity_scale

    try:
        frame = bridge.imgmsg_to_cv2(image, "passthrough")
    except CvBridgeError as e:
        print(e)

    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    vis = frame.copy()

    if prev_gray != None:
        affineTransform = cv2.estimateRigidTransform(prev_gray, frame_gray, False);

        if(affineTransform != None):
            translation_delta = affineTransform[:,[2]]

            scale_x_delta = np.linalg.norm(affineTransform[:,[0]])
            scale_y_delta = np.linalg.norm(affineTransform[:,[1]])

            rot1 = np.matrix('0;1')
            rot2 = np.dot(affineTransform[:,[0,1]], np.matrix('0;1'))

            rotation_delta = angle_between(rot1.A1, rot2.A1);

            translation_current = np.add(translation_current, translation_delta);
            scale_x *= scale_x_delta
            scale_y *= scale_y_delta
            rotation += rotation_delta

            """
            draw_str(vis, (20, 20), 'TX: ' + str(translation_current.item(0,0)))
            draw_str(vis, (20, 40), 'TY: ' + str(translation_current.item(1,0)))
            draw_str(vis, (20, 60), 'SX: ' + str(scale_x))
            draw_str(vis, (20, 80), 'SY: ' + str(scale_y))
            draw_str(vis, (20, 100), 'RAD: ' + str(rotation))
            draw_str(vis, (20, 120), 'DEG: ' + str(np.rad2deg(rotation)))
            """

            time_scale = 0
            current_time = rospy.Time.now()
            print(current_time.nsecs)

            if prev_time:
                time_scale = (1.0 / 1e9*(current_time - prev_time).nsecs)

            prev_time = current_time

            try:
                my_twist = Twist()
                my_twist.linear.x = translation_delta.item(0,0) * linear_velocity_scale * time_scale
                my_twist.linear.y = translation_delta.item(1,0) * linear_velocity_scale * time_scale
                my_twist.linear.z = 0

                my_twist.angular.x = 0
                my_twist.angular.y = 0
                my_twist.angular.z = rotation_delta * rotation_velocity_scale * time_scale

                pub.publish(my_twist)
                rospy.loginfo(my_twist)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr("exception thrown %s", e)
                return


    prev_gray = frame_gray
    cv2.imshow('optical_localization', vis)

if __name__ == '__main__':
    rospy.init_node("optical_localization")
    rospy.Subscriber("/optical_localization_cam/image_raw", Image, callback=image_callback)
    pub = rospy.Publisher("/owr/optical_localization_twist", Twist, latch=True, queue_size=10)
    bridge = CvBridge()

    while not rospy.is_shutdown():
        rospy.spin()

    cv2.destroyAllWindows()
