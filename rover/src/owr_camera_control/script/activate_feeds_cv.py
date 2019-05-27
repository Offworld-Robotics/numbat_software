#!/usr/bin/env python
import rospy
import rospkg
import os
import subprocess
import cv2 
from owr_messages.msg import stream


#called when the message is recived
def callback(data):
    print data
    if data.on:
        rospy.loginfo("turn on /dev/video%d", data.stream)
        #set the parameters for the cameras
        camera_name = "cam" + str(data.stream) #setting name of camera topic
        video_stream_provider = str(data.stream) #number for camera whose stream we want to open
        
        subprocess.Popen([ "roslaunch","video_stream_opencv", "camera.launch", "camera_name:=" + camera_name, "video_stream_provider:=" + video_stream_provider])

    else:
        rospy.loginfo("closing /dev/video%d", data.stream)
        subprocess.call(["rosnode","kill","/camera_" + str(data.stream)]);
        #subprocess.call(["rosnode","kill","gscam_publisher/cam" + str(data.stream)]);

        return
        
def activateFeeds():
    rospy.init_node('activateFeeds')
    rospy.Subscriber("owr/control/activateFeeds", stream, callback)
    print "running"
    video = cv2.VideoCapture(0)
    video2 = cv2.VideoCapture(0)

    #loops while ros is running
    rospy.spin()
    
    
if __name__ == '__main__':
    rospy.loginfo("running activate_feeds")
    activateFeeds()
