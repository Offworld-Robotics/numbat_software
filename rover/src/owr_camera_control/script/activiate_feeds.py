#!/usr/bin/env python
import rospy
import os
from subprocess import call
from owr_camera_control.msg import stream

#called when the message is recived
def callback(data):

    if data.on:
        os.environ["GSCAM_CONFIG"] = "v4l2src device=/dev/video"+str(data.stream)+" ! video/x-raw-rgb,framerate=30/1 ! ffmpegcolorspace"
        call([ "rosrun","gscam", "gscam","__name=" + str(data.stream) + "_camera","gscam_publisher:=\/cam" + str(data.stream)])
    else:
        call(["rosnode","kill","/gscam_publisher"]);
        return
        
def activateFeeds():
    rospy.init_node('activateFeeds')
    rospy.Subscriber("control/activateFeeds", stream, callback)
    
    #loops while ros is running
    rospy.spin()
    
    
if __name__ == '__main__':
    activateFeeds()
